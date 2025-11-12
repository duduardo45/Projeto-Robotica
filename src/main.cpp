#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define ENCODER_PIN 47
#define H_BRIDGE_IN1 41
#define H_BRIDGE_IN2 42
#define PWM_PIN 48
#define DEBOUNCE_US 5000UL // microseconds

// --- NOVO: defina o raio da roda em milímetros aqui ---
#define WHEEL_RADIUS_MM 49.67    // <--- ajuste para o raio real da sua roda (em mm)
#define PULSES_PER_ROTATION 64.0 // DEVERIA SER 32.0, COMASSIM?

// WiFi credentials - UPDATE THESE
#include "credentials.h"

// MQTT Broker configuration - UPDATE THE IP ADDRESS
const char *mqtt_server = "192.168.0.146"; // Replace with your computer's local IP address
const int mqtt_port = 1883;
const char *mqtt_client_id = "esp32_encoder";

// MQTT topics
const char *topic_rotation_count = "encoder/rotation_count";
const char *topic_frequency = "encoder/frequency";
const char *topic_velocity = "encoder/velocity";

WiFiClient espClient;
PubSubClient client(espClient);

volatile unsigned long counter = 0;
volatile unsigned long lastInterruptMicros = 0; // ISR-safe timestamp

unsigned long instanteAtual;
unsigned long instanteAnterior;
unsigned long contadorAnterior = 0;
unsigned long instanteAnterior2;
int motorSpeed = 0;

void IRAM_ATTR ISR_function()
{
  // Read micros() inside ISR is OK on typical Arduino/ESP
  unsigned long now = micros();
  // simple refractory debounce
  if ((now - lastInterruptMicros) >= DEBOUNCE_US)
  {
    counter++;
    lastInterruptMicros = now;
  }
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
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

void setup()
{
  instanteAnterior = millis();
  instanteAnterior2 = millis();

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(H_BRIDGE_IN1, OUTPUT);
  pinMode(H_BRIDGE_IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  // pinMode(ENCODER_PIN, INPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);

  digitalWrite(H_BRIDGE_IN1, HIGH);
  digitalWrite(H_BRIDGE_IN2, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), ISR_function, RISING);

  // Setup WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void toggle_engine()
{
  if (motorSpeed > 120) // Check the variable, not the pin
  {
    motorSpeed = 0;
  }
  else
  {
    motorSpeed = 240;
  }
  analogWrite(PWM_PIN, motorSpeed); // Write the new state to the pin
}

void reconnect_mqtt()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");

    const char *mqtt_user = "robotica";
    const char *mqtt_pass = "robotica123";
    // Attempt to connect
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_pass))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
  instanteAtual = millis();

  if (instanteAtual > instanteAnterior2 + 5000)
  {
    toggle_engine();
    instanteAnterior2 = instanteAtual;
  }

  if (instanteAtual > instanteAnterior + 1000)
  {
    // Check MQTT connection and reconnect if needed
    if (!client.connected())
    {
      reconnect_mqtt();
    }
    client.loop(); // Process MQTT messages

    unsigned long counterCopy; // Use the same type as counter

    // --- Start Critical Section ---
    noInterrupts();
    counterCopy = counter;
    interrupts();
    // --- End Critical Section ---

    unsigned long deltaCount = counterCopy - contadorAnterior;
    unsigned long deltaTimeMs = instanteAtual - instanteAnterior; // ms

    // Calculo da frequência (Hz)
    double rotationCount = deltaCount / PULSES_PER_ROTATION;
    double frequency_Hz = rotationCount / (deltaTimeMs / 1000.0);

    // Calculo da velocidade linear (m/s)
    double speed_m_s = frequency_Hz * 2 * M_PI * (WHEEL_RADIUS_MM / 1000.0);

    // Impressão formatada
    Serial.print("Tick count: ");
    Serial.print(deltaCount);
    Serial.print("\t");
    Serial.print("Rotation count: ");
    Serial.print(rotationCount);
    Serial.print("\t");
    Serial.print("Freq (Hz): ");
    Serial.print(frequency_Hz, 4);
    Serial.print("\t");

    Serial.print("Vel (m/s): ");
    Serial.print(speed_m_s, 4);
    Serial.println(" m/s\t");

    // Publish to MQTT
    char msg_buffer[50];

    // Publish rotation count
    snprintf(msg_buffer, sizeof(msg_buffer), "%.4f", rotationCount);
    client.publish(topic_rotation_count, msg_buffer);

    // Publish frequency
    snprintf(msg_buffer, sizeof(msg_buffer), "%.4f", frequency_Hz);
    client.publish(topic_frequency, msg_buffer);

    // Publish velocity
    snprintf(msg_buffer, sizeof(msg_buffer), "%.4f", speed_m_s);
    client.publish(topic_velocity, msg_buffer);

    // atualizar anteriores
    contadorAnterior = counterCopy; // Use the same copied value
    instanteAnterior = instanteAtual;
  }
}
