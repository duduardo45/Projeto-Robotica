#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

// ============================================================================
// CONFIGURABLE CONSTANTS - Easily adjustable settings
// ============================================================================

// Pin definitions - Left wheel
#define ENCODER_LEFT_PIN 36
#define PWM_LEFT_PIN 38
#define HBL_IN1 41
#define HBL_IN2 42

// Pin definitions - Right wheel
#define ENCODER_RIGHT_PIN 35
#define PWM_RIGHT_PIN 37
#define HBR_IN1 39
#define HBR_IN2 40

// Encoder debounce
#define DEBOUNCE_US 5000UL

// Wheel geometry - Left wheel
#define WHEEL_RADIUS_MM_LEFT 49.67
#define PULSES_PER_ROTATION_LEFT 64.0

// Wheel geometry - Right wheel
#define WHEEL_RADIUS_MM_RIGHT 49.67
#define PULSES_PER_ROTATION_RIGHT 64.0

// Control loop parameters
#define CONTROL_PERIOD_MS 50
#define KP_LEFT 300.0
#define KI_LEFT 80.0
#define KP_RIGHT 300.0
#define KI_RIGHT 80.0
#define PWM_MIN 0
#define PWM_MAX 255
#define SPEED_DEADBAND 0.01
#define INTEGRATOR_CLAMP 50.0

// Telemetry period
#define TELEMETRY_PERIOD_MS 1000

// MQTT topics
#define TOPIC_CMD_SPEED "motor/speed"
#define TOPIC_LEFT_ROTATION_COUNT "encoder/left/rotation_count"
#define TOPIC_LEFT_FREQUENCY "encoder/left/frequency"
#define TOPIC_LEFT_VELOCITY "encoder/left/velocity"
#define TOPIC_RIGHT_ROTATION_COUNT "encoder/right/rotation_count"
#define TOPIC_RIGHT_FREQUENCY "encoder/right/frequency"
#define TOPIC_RIGHT_VELOCITY "encoder/right/velocity"

// WiFi credentials
#include "credentials.h"

// MQTT Broker configuration
const char *mqtt_server = "192.168.0.146";
const int mqtt_port = 1883;
const char *mqtt_client_id = "esp32_encoder";

WiFiClient espClient;
PubSubClient client(espClient);

// ============================================================================
// Wheel Control Structure
// ============================================================================

struct WheelControl
{
  int pwmPin;
  int in1;
  int in2;
  int encoderPin;
  volatile unsigned long count;
  volatile unsigned long lastIsrMicros;
  unsigned long lastCountControl;
  unsigned long lastCountTelemetry;
  unsigned long lastTelemetryTime;
  double kp;
  double ki;
  double integrator;
  double targetSpeed;
  double measuredSpeed;
  double pulsesPerRotation;
  double wheelRadiusMm;
};

WheelControl leftWheel;
WheelControl rightWheel;

// ============================================================================
// Interrupt Service Routines
// ============================================================================

void IRAM_ATTR isrLeft()
{
  unsigned long now = micros();
  if ((now - leftWheel.lastIsrMicros) >= DEBOUNCE_US)
  {
    leftWheel.count++;
    leftWheel.lastIsrMicros = now;
  }
}

void IRAM_ATTR isrRight()
{
  unsigned long now = micros();
  if ((now - rightWheel.lastIsrMicros) >= DEBOUNCE_US)
  {
    rightWheel.count++;
    rightWheel.lastIsrMicros = now;
  }
}

// ============================================================================
// WiFi Setup
// ============================================================================

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// ============================================================================
// MQTT Callback
// ============================================================================

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, TOPIC_CMD_SPEED) != 0)
  {
    return;
  }
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error)
  {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  double leftSpeed = doc["left"] | 0.0;
  double rightSpeed = doc["right"] | 0.0;

  leftWheel.targetSpeed = leftSpeed;
  rightWheel.targetSpeed = rightSpeed;

  Serial.print("Received speeds - Left: ");
  Serial.print(leftSpeed);
  Serial.print(" m/s, Right: ");
  Serial.print(rightSpeed);
  Serial.println(" m/s");
}

// ============================================================================
// MQTT Reconnect
// ============================================================================

void reconnect_mqtt()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");

    const char *mqtt_user = "robotica";
    const char *mqtt_pass = "robotica123";

    if (client.connect(mqtt_client_id, mqtt_user, mqtt_pass))
    {
      Serial.println("connected");
      client.subscribe(TOPIC_CMD_SPEED);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// ============================================================================
// Control Loop (PI Controller)
// ============================================================================

void updateWheelControl(WheelControl &wheel, unsigned long dtMs)
{
  unsigned long countCopy;
  noInterrupts();
  countCopy = wheel.count;
  interrupts();

  unsigned long deltaCount = countCopy - wheel.lastCountControl;
  wheel.lastCountControl = countCopy;

  double dt = dtMs / 1000.0;
  double rotationCount = deltaCount / wheel.pulsesPerRotation;
  double frequency_Hz = rotationCount / dt;
  wheel.measuredSpeed = frequency_Hz * 2 * M_PI * (wheel.wheelRadiusMm / 1000.0);

  bool forward = (wheel.targetSpeed >= 0);
  digitalWrite(wheel.in1, forward ? HIGH : LOW);
  digitalWrite(wheel.in2, forward ? LOW : HIGH);

  double targetAbs = fabs(wheel.targetSpeed);
  double error = targetAbs - wheel.measuredSpeed;

  if (targetAbs < SPEED_DEADBAND)
  {
    wheel.integrator = 0;
  }
  else
  {
    wheel.integrator += error * dt;
    wheel.integrator = constrain(wheel.integrator, -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP);
  }

  double u = (wheel.kp * error) + (wheel.ki * wheel.integrator);
  int pwm = (int)round(constrain(u, (double)PWM_MIN, (double)PWM_MAX));
  analogWrite(wheel.pwmPin, pwm);
}

// ============================================================================
// Telemetry Publishing
// ============================================================================

void publishTelemetry(WheelControl &wheel, const char *topicRotation,
                      const char *topicFreq, const char *topicVel,
                      const char *label)
{
  unsigned long countCopy;
  noInterrupts();
  countCopy = wheel.count;
  interrupts();

  unsigned long deltaCount = countCopy - wheel.lastCountTelemetry;
  wheel.lastCountTelemetry = countCopy;

  unsigned long now = millis();
  unsigned long deltaTimeMs = now - wheel.lastTelemetryTime;
  wheel.lastTelemetryTime = now;

  if (deltaTimeMs == 0)
    deltaTimeMs = TELEMETRY_PERIOD_MS;

  double rotationCount = deltaCount / wheel.pulsesPerRotation;
  double frequency_Hz = rotationCount / (deltaTimeMs / 1000.0);
  double speed_m_s = frequency_Hz * 2 * M_PI * (wheel.wheelRadiusMm / 1000.0);

  Serial.print(label);
  Serial.print(" - Tick: ");
  Serial.print(deltaCount);
  Serial.print("\tRotations: ");
  Serial.print(rotationCount, 4);
  Serial.print("\tFreq (Hz): ");
  Serial.print(frequency_Hz, 4);
  Serial.print("\tVel (m/s): ");
  Serial.print(speed_m_s, 4);
  Serial.println(" m/s");

  char msg_buffer[50];

  snprintf(msg_buffer, sizeof(msg_buffer), "%.4f", rotationCount);
  client.publish(topicRotation, msg_buffer);

  snprintf(msg_buffer, sizeof(msg_buffer), "%.4f", frequency_Hz);
  client.publish(topicFreq, msg_buffer);

  snprintf(msg_buffer, sizeof(msg_buffer), "%.4f", speed_m_s);
  client.publish(topicVel, msg_buffer);
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting dual-wheel control...");

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize left wheel structure
  leftWheel.pwmPin = PWM_LEFT_PIN;
  leftWheel.in1 = HBL_IN1;
  leftWheel.in2 = HBL_IN2;
  leftWheel.encoderPin = ENCODER_LEFT_PIN;
  leftWheel.count = 0;
  leftWheel.lastIsrMicros = 0;
  leftWheel.lastCountControl = 0;
  leftWheel.lastCountTelemetry = 0;
  leftWheel.lastTelemetryTime = millis();
  leftWheel.kp = KP_LEFT;
  leftWheel.ki = KI_LEFT;
  leftWheel.integrator = 0;
  leftWheel.targetSpeed = 0;
  leftWheel.measuredSpeed = 0;
  leftWheel.pulsesPerRotation = PULSES_PER_ROTATION_LEFT;
  leftWheel.wheelRadiusMm = WHEEL_RADIUS_MM_LEFT;

  // Initialize right wheel structure
  rightWheel.pwmPin = PWM_RIGHT_PIN;
  rightWheel.in1 = HBR_IN1;
  rightWheel.in2 = HBR_IN2;
  rightWheel.encoderPin = ENCODER_RIGHT_PIN;
  rightWheel.count = 0;
  rightWheel.lastIsrMicros = 0;
  rightWheel.lastCountControl = 0;
  rightWheel.lastCountTelemetry = 0;
  rightWheel.lastTelemetryTime = millis();
  rightWheel.kp = KP_RIGHT;
  rightWheel.ki = KI_RIGHT;
  rightWheel.integrator = 0;
  rightWheel.targetSpeed = 0;
  rightWheel.measuredSpeed = 0;
  rightWheel.pulsesPerRotation = PULSES_PER_ROTATION_RIGHT;
  rightWheel.wheelRadiusMm = WHEEL_RADIUS_MM_RIGHT;

  // Configure pins - Left wheel
  pinMode(HBL_IN1, OUTPUT);
  pinMode(HBL_IN2, OUTPUT);
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  digitalWrite(HBL_IN1, HIGH);
  digitalWrite(HBL_IN2, LOW);
  analogWrite(PWM_LEFT_PIN, 0);

  // Configure pins - Right wheel
  pinMode(HBR_IN1, OUTPUT);
  pinMode(HBR_IN2, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  digitalWrite(HBR_IN1, HIGH);
  digitalWrite(HBR_IN2, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isrRight, RISING);

  // Setup WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop()
{
  static unsigned long lastControlTime = 0;
  unsigned long now = millis();

  // MQTT connection check and message processing
  if (!client.connected())
  {
    reconnect_mqtt();
  }
  client.loop();

  // Control loop
  if (now - lastControlTime >= CONTROL_PERIOD_MS)
  {
    unsigned long dtMs = now - lastControlTime;
    lastControlTime = now;
    updateWheelControl(leftWheel, dtMs);
    updateWheelControl(rightWheel, dtMs);
  }

  // Telemetry loop (1 Hz) - check each wheel independently
  if (now - leftWheel.lastTelemetryTime >= TELEMETRY_PERIOD_MS)
  {
    publishTelemetry(leftWheel, TOPIC_LEFT_ROTATION_COUNT, TOPIC_LEFT_FREQUENCY,
                     TOPIC_LEFT_VELOCITY, "LEFT");
  }
  if (now - rightWheel.lastTelemetryTime >= TELEMETRY_PERIOD_MS)
  {
    publishTelemetry(rightWheel, TOPIC_RIGHT_ROTATION_COUNT, TOPIC_RIGHT_FREQUENCY,
                     TOPIC_RIGHT_VELOCITY, "RIGHT");
  }
}
