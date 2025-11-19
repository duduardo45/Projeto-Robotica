#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <esp_camera.h>

#include "camera_pins.h"
#include "credentials.h"

// ===================================================================================
// CONFIGURATION
// ===================================================================================

// --- Networking ---
const char *WS_HOST = "192.168.0.146";
const uint16_t WS_PORT = 8000;
const char *WS_PATH = "/ws/robot";
const unsigned long WS_RECONNECT_INTERVAL_MS = 5000;

// --- Physics & Tuning ---
// Left Wheel
#define PIN_ENC_L 18
#define PIN_PWM_L 15
#define PIN_IN1_L 16
#define PIN_IN2_L 17
#define TUNING_L {0.0, 0.0, 0.0}  // {kP, kI, kF}

// Right Wheel (Preserved but commented)
// #define PIN_ENC_R   35
// #define PIN_PWM_R   7
// #define PIN_IN1_R   5
// #define PIN_IN2_R   6
// #define TUNING_R    {0.0, 0.0, 0.0}

#define WHEEL_RADIUS_MM 49.67
#define PULSES_PER_ROT 64.0
#define PWM_LIMITS {0, 255}
#define SPEED_DEADBAND 0.01
#define INTEGRATOR_CLAMP 50.0
#define SPEED_FILTER_ALPHA 0.4

// --- Timing ---
#define CONTROL_PERIOD_MS 10
#define TELEMETRY_PERIOD_MS 200
#define FRAME_STREAM_PERIOD_MS 200

// ===================================================================================
// DATA STRUCTURES
// ===================================================================================

using namespace websockets;

struct __attribute__((packed)) FramePacketHeader {
  uint32_t magic = 0x46524D31;  // "FRM1"
  uint16_t version = 1;
  uint16_t reserved = 0;
  uint32_t frameId;
  uint32_t timestampMs;
  uint16_t width;
  uint16_t height;
  uint32_t payloadLength;
};

struct PidConfig {
  double kp, ki, kf;
};

// ===================================================================================
// WHEEL CONTROLLER CLASS
// ===================================================================================

class WheelController {
 public:
  // Hardware Pins
  const int pinPwm, pinIn1, pinIn2, pinEnc;

  // State
  volatile unsigned long encoderCount = 0;
  unsigned long lastEncoderCount = 0;

  double targetSpeed = 0.0;    // m/s
  double measuredSpeed = 0.0;  // m/s (filtered)
  double rawSpeed = 0.0;       // m/s (instant)
  double measuredRpm = 0.0;
  double integrator = 0.0;

  WheelController(int pwm, int in1, int in2, int enc, PidConfig pid)
      : pinPwm(pwm), pinIn1(in1), pinIn2(in2), pinEnc(enc), _pid(pid) {
    // Pre-calculate physics constant
    double circumference = 2 * M_PI * (WHEEL_RADIUS_MM / 1000.0);
    _metersPerPulse = circumference / PULSES_PER_ROT;
  }

  void begin() {
    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);
    pinMode(pinPwm, OUTPUT);
    pinMode(pinEnc, INPUT);
    stop();
  }

  // Call from ISR
  void IRAM_ATTR handleInterrupt() { encoderCount++; }

  void stop() {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinPwm, 0);
    targetSpeed = 0;
    integrator = 0;
  }

  void update(unsigned long dtMs) {
    // 1. Atomic Encoder Read
    unsigned long currentCount;
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    unsigned long delta = currentCount - lastEncoderCount;
    lastEncoderCount = currentCount;

    // 2. Physics Calc
    double dtSec = dtMs / 1000.0;
    if (dtSec <= 0) dtSec = 0.001;

    double instantSpeed = ((double)delta / dtSec) * _metersPerPulse;
    rawSpeed = instantSpeed;

    // Low Pass Filter
    measuredSpeed = (SPEED_FILTER_ALPHA * instantSpeed) +
                    ((1.0 - SPEED_FILTER_ALPHA) * measuredSpeed);
    measuredRpm = (measuredSpeed / _metersPerPulse) * 60.0 /
                  PULSES_PER_ROT;  // Simplified RPM math

    // 3. PID Control
    bool forward = (targetSpeed >= 0);
    digitalWrite(pinIn1, forward ? HIGH : LOW);
    digitalWrite(pinIn2, forward ? LOW : HIGH);

    double targetAbs = fabs(targetSpeed);
    double error = targetAbs - measuredSpeed;

    // Integral Anti-windup
    if (targetAbs < SPEED_DEADBAND) {
      integrator = 0;
    } else {
      integrator += error * dtSec;
      integrator = constrain(integrator, -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP);
    }

    double u =
        (_pid.kf * targetAbs) + (_pid.kp * error) + (_pid.ki * integrator);
    int pwm = (int)constrain(round(u), 0.0, 255.0);

    if (targetAbs < 0.01) pwm = 0;

    // NOTE: VOU HARDCODAR O PWM PARA 100 PRA PODER TUNAR O KF, NA PROXIMA VEZ
    // QUE EU FOR NO LAB

    // pwm = 100;
    analogWrite(pinPwm, pwm);
  }

 private:
  PidConfig _pid;
  double _metersPerPulse;
};

// ===================================================================================
// GLOBALS & INSTANCES
// ===================================================================================

WebsocketsClient wsClient;
bool cameraReady = false;
uint32_t frameSequence = 0;

// Instantiate Left Wheel
WheelController leftWheel(PIN_PWM_L, PIN_IN1_L, PIN_IN2_L, PIN_ENC_L, TUNING_L);

// Instantiate Right Wheel (Commented)
// WheelController rightWheel(PIN_PWM_R, PIN_IN1_R, PIN_IN2_R, PIN_ENC_R,
// TUNING_R);

// ISR Wrappers (Required because we can't attach class methods directly to
// interrupts)
void IRAM_ATTR isrLeft() { leftWheel.handleInterrupt(); }
// void IRAM_ATTR isrRight() { rightWheel.handleInterrupt(); }

// ===================================================================================
// NETWORK HELPERS
// ===================================================================================

void connectWiFi() {
  Serial.printf("Connecting to %s", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected: %s\n", WiFi.localIP().toString().c_str());
}

void handleIncomingJson(String data) {
  JsonDocument doc;
  if (deserializeJson(doc, data)) {
    Serial.println(F("JSON Error"));
    return;
  }

  if (doc["type"] == "command") {
    leftWheel.targetSpeed = doc["left"] | 0.0;
    // rightWheel.targetSpeed = doc["right"] | 0.0;
    Serial.printf("Target Left: %.3f\n", leftWheel.targetSpeed);
  }
}

void manageWebSocket() {
  static unsigned long lastAttempt = 0;
  if (wsClient.available()) {
    wsClient.poll();
    return;
  }

  if (millis() - lastAttempt > WS_RECONNECT_INTERVAL_MS) {
    lastAttempt = millis();
    Serial.printf("WS Connect: %s:%d\n", WS_HOST, WS_PORT);
    wsClient.connect(WS_HOST, WS_PORT, WS_PATH);
  }
}

// ===================================================================================
// TELEMETRY & CAMERA
// ===================================================================================

void sendTelemetry() {
  if (!wsClient.available()) return;

  static unsigned long lastTime = 0;
  unsigned long now = millis();
  if (now - lastTime < TELEMETRY_PERIOD_MS) return;
  lastTime = now;

  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["timestamp"] = now;

  JsonObject l = doc["left"].to<JsonObject>();
  l["speed"] = leftWheel.measuredSpeed;
  l["rpm"] = leftWheel.measuredRpm;
  l["target"] = leftWheel.targetSpeed;
  l["int"] = leftWheel.integrator;

  // Commented Right Wheel Telemetry
  // JsonObject r = doc["right"].to<JsonObject>();
  // r["speed"] = rightWheel.measuredSpeed; ...

  String payload;
  serializeJson(doc, payload);
  wsClient.send(payload);
}

void setupCamera() {
  camera_config_t config;

  // 1. Pin Mappings (These constants come from camera_pins.h)
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0;
  config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2;
  config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4;
  config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6;
  config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;

  // 2. Signal Clock & Format
  config.xclk_freq_hz = 20000000;  // 20MHz XCLK
  config.pixel_format = PIXFORMAT_GRAYSCALE;

  // 3. Frame Settings
  config.frame_size = FRAMESIZE_QVGA;

  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // 4. Initialize
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // 5. Optional: Flip frame if camera is mounted upside down
  sensor_t *s = esp_camera_sensor_get();
  // s->set_vflip(s, 1);
  // s->set_hmirror(s, 1);

  cameraReady = true;  // <--- IMPORTANT: This allows loop() to stream
  Serial.println("Camera Configured Successfully");
}

void streamCamera(unsigned long now) {
  static unsigned long lastFrame = 0;

  // Basic throttle and readiness checks
  if (!cameraReady || !wsClient.available() ||
      (now - lastFrame < FRAME_STREAM_PERIOD_MS))
    return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  if (fb->format == PIXFORMAT_GRAYSCALE) {
    size_t hSize = sizeof(FramePacketHeader);
    size_t totalSize = hSize + fb->len;

    // Allocate buffer for Header + Raw Data
    uint8_t *packet = (uint8_t *)malloc(totalSize);

    if (packet) {
      FramePacketHeader header;
      header.magic = 0x46524D31;
      header.frameId = frameSequence++;
      header.timestampMs = now;
      header.width = fb->width;
      header.height = fb->height;
      header.payloadLength = fb->len;

      // Copy header
      memcpy(packet, &header, hSize);
      // Copy raw grayscale data
      memcpy(packet + hSize, fb->buf, fb->len);

      wsClient.sendBinary((const char *)packet, totalSize);

      free(packet);
      lastFrame = now;
    } else {
      Serial.println("Camera Malloc Failed");
    }
  }

  esp_camera_fb_return(fb);
}

// ===================================================================================
// MAIN
// ===================================================================================

void setup() {
  neopixelWrite(RGB_BUILTIN, 50, 0, 50);  // Purple
  Serial.begin(115200);
  delay(1000);

  // Hardware Setup
  leftWheel.begin();
  attachInterrupt(digitalPinToInterrupt(leftWheel.pinEnc), isrLeft, RISING);

  // rightWheel.begin();
  // attachInterrupt(digitalPinToInterrupt(rightWheel.pinEnc), isrRight,
  // RISING);

  connectWiFi();

  setupCamera();

  // WebSocket Callbacks
  wsClient.onMessage([](WebsocketsMessage msg) {
    if (msg.isText()) handleIncomingJson(msg.data());
  });

  wsClient.onEvent([](WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      Serial.println("WS Connected");
      wsClient.send("{\"type\":\"hello\",\"role\":\"esp32\"}");
    }
  });

  neopixelWrite(RGB_BUILTIN, 0, 50, 0);  // Green
}

void loop() {
  unsigned long now = millis();

  // 1. Network
  manageWebSocket();

  // 2. Control Loop
  static unsigned long lastControl = 0;
  if (now - lastControl >= CONTROL_PERIOD_MS) {
    unsigned long dt = now - lastControl;
    lastControl = now;
    leftWheel.update(dt);
    // rightWheel.update(dt);
  }

  // 3. Reporting
  sendTelemetry();
  // streamCamera(now);
}