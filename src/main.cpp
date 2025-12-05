#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <esp_camera.h>

#include "camera_pins.h"
#include "credentials.h"
// DO NOT DO NEOPIXEL WRITE BECAUSE WE COULD BE USING ITS PIN
//
// CONFIGURATION
//

// --- Networking ---
const char *WS_HOST = "192.168.0.146";
// const char *WS_HOST = "10.233.189.135";
const uint16_t WS_PORT = 8000;
const char *WS_PATH = "/ws/robot";
const int64_t WS_RECONNECT_INTERVAL_MICROS = 5e6;

// --- Physics & Tuning ---
// Left Wheel
#define PIN_ENC_L 21
#define PIN_PWM_L 1
#define PIN_IN1_L 44
#define PIN_IN2_L 2
#define PWM_CHAN_L 2  // Channel 0 is taken by Camera! Using 2.
#define TUNING_L {70.0, 10.0, 40.0, 30.0}  // {ks, kf, kp, ki}

// Right Wheel. it has a bigger ks because it has more static friction
#define PIN_ENC_R 47
#define PIN_PWM_R 42
#define PIN_IN1_R 41
#define PIN_IN2_R 48
#define PWM_CHAN_R 3
#define TUNING_R {120.0, 10.0, 40.0, 30.0}

#define WHEEL_RADIUS_MM 49.67
#define PULSES_PER_ROT 64.0
#define PWM_LIMITS {0, 255}
#define SPEED_DEADBAND 0.01
#define INTEGRATOR_CLAMP 6.0
// SPEED_FILTER_ALPHA the lower you make it, the slower is the exponential
// average:
#define SPEED_FILTER_ALPHA 0.1

// --- Timing ---
#define CONTROL_PERIOD_MICROS 200e3
#define TELEMETRY_PERIOD_MICROS 50e3
#define FRAME_STREAM_PERIOD_MICROS 50e3
#define PWM_FREQ 5000
#define PWM_RES 8

//

// DATA STRUCTURES
//

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
  double ks, kf, kp, ki;
};

//

// WHEEL CONTROLLER CLASS
//

class WheelController {
 public:
  const int pwmChannel;
  const int pinPwm;
  const int pinIn1;
  const int pinIn2;
  const int pinEnc;
  // State
  volatile unsigned long encoderCount = 0;
  unsigned long lastEncoderCount = 0;

  double targetSpeed = 0.0;    // m/s
  double filteredSpeed = 0.0;  // m/s (filtered)
  double rawSpeed = 0.0;       // m/s (instant)
  double filteredRpm = 0.0;
  double integrator = 0.0;

  double debug_p = 0;
  double debug_i = 0;
  double debug_f = 0;
  double debug_out = 0;

  WheelController(const PidConfig pid, const int pwmChannel, const int pinPwm,
                  const int pinIn1, const int pinIn2, const int pinEnc)
      : _pid(pid),
        pwmChannel(pwmChannel),
        pinPwm(pinPwm),
        pinIn1(pinIn1),
        pinIn2(pinIn2),
        pinEnc(pinEnc) {
    // Pre-calculate physics constant
    double circumference = 2 * M_PI * (WHEEL_RADIUS_MM / 1000.0);
    _metersPerPulse = circumference / PULSES_PER_ROT;
  }

  void begin() {
    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);
    pinMode(pinEnc, INPUT);

    // --- Use ledcAttachPin ---
    ledcSetup(pwmChannel, PWM_FREQ, PWM_RES);
    ledcAttachPin(pinPwm, pwmChannel);
    ledcWrite(pwmChannel, 0);
    stop();
  }

  // Call from ISR
  void IRAM_ATTR handleInterrupt() { encoderCount++; }

  void stop() {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    ledcWrite(pwmChannel, 0);
    targetSpeed = 0;
    integrator = 0;
  }

  void update(int64_t nowMicros) {
    // 1. Atomic Encoder Read
    unsigned long currentCount;
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    // 2. High Precision Time Calculation
    int64_t currentMicros = esp_timer_get_time();
    int64_t dtMicros = currentMicros - lastMicros;
    lastMicros = currentMicros;  // Update this immediately

    // Sanity check (prevent divide by zero or overflow on first run)
    if (dtMicros < 100) return;

    double dtSec = dtMicros / 1000000.0;

    // 3. Calculate Speed
    long delta = currentCount - lastEncoderCount;
    lastEncoderCount = currentCount;

    double instantSpeed = 0.0;

    if (delta != 0) {
      // We moved! Update the speed normally
      instantSpeed = ((double)delta / dtSec) * _metersPerPulse;
      lastPulseMicros = currentMicros;  // Reset timeout timer
    } else {
      // We did NOT move this loop.
      // Check how long it has been since the last move.
      // If > 100ms (0.1s), force speed to 0.
      if ((currentMicros - lastPulseMicros) > 100000) {
        instantSpeed = 0.0;
        filteredSpeed = 0.0;  // Force filter reset
      } else {
        // We haven't moved *this* X ms, but we moved recently.
        // Keep instantSpeed as 0, but let the Low Pass Filter
        // naturally decay the measuredSpeed down.
        instantSpeed = 0.0;
      }
    }

    rawSpeed = instantSpeed;

    // 4. Low Pass Filter
    filteredSpeed = (SPEED_FILTER_ALPHA * instantSpeed) +
                    ((1.0 - SPEED_FILTER_ALPHA) * filteredSpeed);

    filteredRpm = (filteredSpeed / _metersPerPulse) * 60.0 / PULSES_PER_ROT;

    // 5. Control Logic
    bool forward = (targetSpeed >= 0);
    digitalWrite(pinIn1, forward ? HIGH : LOW);
    digitalWrite(pinIn2, forward ? LOW : HIGH);

    double targetAbs = fabs(targetSpeed);
    double error = targetAbs - filteredSpeed;

    // Integral Anti-windup
    if (targetAbs < SPEED_DEADBAND) {
      integrator = 0;
    } else {
      integrator += error * dtSec;
      integrator = constrain(integrator, -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP);
    }

    // 6. Calculate Output

    double output = 0.0;
    if (targetAbs > 0.01) {
      // Break it down for logging
      debug_f = _pid.kf * targetAbs;
      debug_p = _pid.kp * error;
      debug_i = _pid.ki * integrator;

      output = _pid.ks + debug_f + debug_p + debug_i;
    } else {
      debug_f = 0;
      debug_p = 0;
      debug_i = 0;
    }

    debug_out = output;  // Store unconstrained output

    int pwm = (int)constrain(round(output), 0.0, 255.0);

    ledcWrite(pwmChannel, pwm);
  }

 private:
  PidConfig _pid;
  double _metersPerPulse;
  int64_t lastMicros = 0;
  int64_t lastPulseMicros = 0;
};

//

// GLOBALS & INSTANCES
//

WebsocketsClient wsClient;
bool cameraReady = false;
uint32_t frameSequence = 0;
// Instantiate Left Wheel
WheelController leftWheel(TUNING_L, PWM_CHAN_L, PIN_PWM_L, PIN_IN1_L, PIN_IN2_L,
                          PIN_ENC_L);

// Instantiate Right Wheel
WheelController rightWheel(TUNING_R, PWM_CHAN_R, PIN_PWM_R, PIN_IN1_R,
                           PIN_IN2_R, PIN_ENC_R);

// ISR Wrappers (Required because we can't attach class methods directly to
// interrupts)
void IRAM_ATTR isrLeft() { leftWheel.handleInterrupt(); }
void IRAM_ATTR isrRight() { rightWheel.handleInterrupt(); }

//

// NETWORK HELPERS
//

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
    rightWheel.targetSpeed = doc["right"] | 0.0;
    Serial.printf("Target Left: %.3f\n", leftWheel.targetSpeed);
    Serial.printf("Target Right: %.3f\n", rightWheel.targetSpeed);
  }
}

void manageWebSocket() {
  static int64_t lastAttempt = 0;
  if (wsClient.available()) {
    wsClient.poll();
    return;
  }

  if (esp_timer_get_time() - lastAttempt > WS_RECONNECT_INTERVAL_MICROS) {
    lastAttempt = esp_timer_get_time();
    Serial.printf("WS Connect: %s:%d\n", WS_HOST, WS_PORT);
    wsClient.connect(WS_HOST, WS_PORT, WS_PATH);
  }
}

//

//         // TELEMETRY & CAMERA
//         //

void sendTelemetry() {
  if (!wsClient.available()) return;

  static int64_t lastTime = 0;
  int64_t now = esp_timer_get_time();
  if (now - lastTime < TELEMETRY_PERIOD_MICROS) return;
  lastTime = now;

  JsonDocument doc;
  doc["message_type"] = "telemetry";
  doc["timestamp"] = now;

  JsonObject left = doc["left"].to<JsonObject>();
  left["encoder"] = leftWheel.encoderCount;
  left["raw_speed"] = leftWheel.rawSpeed;
  left["filtered_speed"] = leftWheel.filteredSpeed;
  left["target_speed"] = leftWheel.targetSpeed;

  left["debug_f"] = leftWheel.debug_f;
  left["debug_p"] = leftWheel.debug_p;
  left["debug_i"] = leftWheel.debug_i;
  left["debug_out"] = leftWheel.debug_out;

  JsonObject right = doc["right"].to<JsonObject>();
  right["encoder"] = rightWheel.encoderCount;
  right["raw_speed"] = rightWheel.rawSpeed;
  right["filtered_speed"] = rightWheel.filteredSpeed;
  right["target_speed"] = rightWheel.targetSpeed;

  right["debug_f"] = rightWheel.debug_f;
  right["debug_p"] = rightWheel.debug_p;
  right["debug_i"] = rightWheel.debug_i;
  right["debug_out"] = rightWheel.debug_out;

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
  config.pin_sccb_sda = CAM_PIN_SCCB_SDA;
  config.pin_sccb_scl = CAM_PIN_SCCB_SCL;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;

  config.xclk_freq_hz = 20000000;  // 20MHz XCLK

  // config.pixel_format = PIXFORMAT_GRAYSCALE;

  // // 3. Frame Settings
  // config.frame_size = FRAMESIZE_QVGA;

  // config.fb_count = 1;
  // config.fb_location = CAMERA_FB_IN_PSRAM;
  // config.grab_mode = CAMERA_GRAB_LATEST;

  config.pixel_format = PIXFORMAT_JPEG;  // Was PIXFORMAT_GRAYSCALE
  config.frame_size = FRAMESIZE_QVGA;    // 320x240
  config.jpeg_quality =
      12;               // 0-63. 10-12 is good balance. Lower is higher quality.
  config.fb_count = 2;  // Use 2 buffers for smoother streaming
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

void streamCamera(int64_t now) {
  static int64_t lastFrame = 0;

  // 1. Throttle check
  if (!cameraReady || !wsClient.available() ||
      (now - lastFrame < FRAME_STREAM_PERIOD_MICROS))
    return;

  // --- START TIMING ---
  int64_t t_start = esp_timer_get_time();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  int64_t t_captured = esp_timer_get_time();  // Time after capture

  // if (fb->format == PIXFORMAT_GRAYSCALE) {
  if (fb->format == PIXFORMAT_JPEG) {
    size_t hSize = sizeof(FramePacketHeader);
    size_t totalSize = hSize + fb->len;

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

      int64_t t_copied = esp_timer_get_time();  // Time after memory copy

      // Send data
      wsClient.sendBinary((const char *)packet, totalSize);

      int64_t t_sent = esp_timer_get_time();  // Time after network send

      free(packet);
      lastFrame = now;

      // --- PRINT DIAGNOSTICS ---
      // Serial.printf(
      //     "TIMING (ms) -> Capture: %lu | Copy: %lu | Network: %lu | Total: "
      //     "%lu\n",
      //     (t_captured - t_start),   // How long the camera took to give a
      //     frame (t_copied - t_captured),  // How long malloc + memcpy took
      //     (t_sent - t_copied),      // How long WiFi took to push data
      //     (t_sent - t_start)        // Total time blocked
      // );
      Serial.printf(
          "JPEG TIMING (us) -> Capture: %lu | Copy: %lu | Network: %lu | "
          "Total: %lu | Size: %u bytes\n",
          (t_captured - t_start), (t_copied - t_captured), (t_sent - t_copied),
          (t_sent - t_start),
          fb->len  // Look at how small this number is now!
      );
    } else {
      Serial.println("Camera Malloc Failed");
    }
  }

  esp_camera_fb_return(fb);
}

//
// MAIN
//

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Hardware Setup
  leftWheel.begin();
  attachInterrupt(digitalPinToInterrupt(leftWheel.pinEnc), isrLeft, CHANGE);

  rightWheel.begin();
  attachInterrupt(digitalPinToInterrupt(rightWheel.pinEnc), isrRight, CHANGE);

  connectWiFi();

  setupCamera();

  // WebSocket Callbacks
  wsClient.onMessage([](WebsocketsMessage msg) {
    if (msg.isText()) handleIncomingJson(msg.data());
  });
}

void loop() {
  int64_t now = esp_timer_get_time();

  // 1. Network
  manageWebSocket();

  // 2. Control Loop
  static int64_t lastControl = 0;
  if (now - lastControl >= CONTROL_PERIOD_MICROS) {
    int64_t dt = now - lastControl;
    lastControl = now;
    leftWheel.update(dt);
    rightWheel.update(dt);
  }

  // 3. Reporting
  sendTelemetry();
  streamCamera(now);
}