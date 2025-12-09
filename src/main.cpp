#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_camera.h>

#include "camera_pins.h"
#include "credentials.h"
// DO NOT DO NEOPIXEL WRITE BECAUSE WE COULD BE USING ITS PIN
//
// CONFIGURATION
//

// --- Networking ---
const uint16_t WS_PORT = 8000;

const char *AP_SSID = "ESP32_Robot_AP";
const char *AP_PASS = "robot1234";  // Must be at least 8 chars

// #define WHEELS_DISABLED
// --- Physics & Tuning ---
// Left Wheel
#define PIN_ENC_L 21
#define PIN_PWM_L 1
#define PIN_IN1_L 44
#define PIN_IN2_L 2
#define PWM_CHAN_L 2  // Channel 0 is taken by Camera! Using 2.
#define TUNING_L {120.0, 10.0, 40.0, 30.0}  // {ks, kf, kp, ki}

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

#define MAX_ACCEL 0.4
#define DITHER_MAGNITUDE 70.0

// --- Timing ---
#define CONTROL_PERIOD_MICROS 10e3
#define TELEMETRY_PERIOD_MICROS 50e3
#define FRAME_STREAM_PERIOD_MICROS 200e3
#define PWM_FREQ 5000
#define PWM_RES 8
#define HEARTBEAT_TIMEOUT_MICROS 3000e3
//

// DATA STRUCTURES
//

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// Mutex for protecting variables shared between loop() and motorTask()
portMUX_TYPE sharedVarMux = portMUX_INITIALIZER_UNLOCKED;

esp_timer_handle_t controlTimer;

struct PidConfig {
  double ks, kf, kp, ki;
};

// --- THREAD SAFETY BRIDGE ---
struct MotorCommand {
  double targetL;
  double targetR;
  int64_t lastHeartbeat;
};

struct MotorState {
  double encL, rawL, filtL, targetL;
  double encR, rawR, filtR, targetR;
  double debug_fL, debug_pL, debug_iL, debug_outL;
  double debug_fR, debug_pR, debug_iR, debug_outR;
};

// Global instances of the bridge data
MotorCommand sharedCommand = {0, 0, 0};
MotorState sharedState = {0};

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

  double targetSpeed = 0.0;     // m/s
  double internalTarget = 0.0;  // m/s (ramped target)
  double filteredSpeed = 0.0;   // m/s (filtered)
  double rawSpeed = 0.0;        // m/s (instant)
  double filteredRpm = 0.0;
  double integrator = 0.0;
  bool ditherState = false;

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
  void IRAM_ATTR handleInterrupt() {
#ifdef WHEELS_DISABLED
    encoderCount = 0;
#else
    encoderCount++;
#endif
  }

  void stop() {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    ledcWrite(pwmChannel, 0);
    targetSpeed = 0;
    internalTarget = 0;
    integrator = 0;
  }

  void update() {
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
      if ((currentMicros - lastPulseMicros) > 200e3) {
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

    // 5. Ramping (The Wedge)
    // Move the internal target towards the real target slowly
    double max_step = MAX_ACCEL * dtSec;

    if (targetSpeed > internalTarget) {
      internalTarget += max_step;
      if (internalTarget > targetSpeed) internalTarget = targetSpeed;
    } else if (targetSpeed < internalTarget) {
      internalTarget -= max_step;
      if (internalTarget < targetSpeed) internalTarget = targetSpeed;
    }

    // 6. Control Logic
    bool forward = (targetSpeed >= 0);
    digitalWrite(pinIn1, forward ? HIGH : LOW);
    digitalWrite(pinIn2, forward ? LOW : HIGH);

    double targetAbs = fabs(internalTarget);
    double error = targetAbs - filteredSpeed;

    // Integral Anti-windup
    if (targetAbs < SPEED_DEADBAND) {
      integrator = 0;
    } else {
      integrator += error * dtSec;
      integrator = constrain(integrator, -INTEGRATOR_CLAMP, INTEGRATOR_CLAMP);
    }

    // 7. Calculate Output

    double pid_out = 0.0;
    if (targetAbs > 0.01) {
      // Break it down for logging
      debug_f = _pid.kf * targetAbs;
      debug_p = _pid.kp * error;
      debug_i = _pid.ki * integrator;

      pid_out = _pid.ks + debug_f + debug_p + debug_i;
    } else {
      debug_f = 0;
      debug_p = 0;
      debug_i = 0;
    }

    // 8. Dithering (The Lubricant)
    // Flip sign every loop (20ms).
    // This creates a 25Hz vibration.
    int dither = (ditherState) ? DITHER_MAGNITUDE : -DITHER_MAGNITUDE;
    ditherState = !ditherState;  // Toggle for next time
    // Only dither if we are trying to move
    if (fabs(internalTarget) < 0.001) dither = 0;
    double finalOutput = pid_out + dither;

    debug_out = finalOutput;  // Store unconstrained output

    int pwm = (int)constrain(round(finalOutput), 0.0, 255.0);

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

WebSocketsServer webSocket = WebSocketsServer(WS_PORT);
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

void setupAccessPoint() {
  Serial.println("Setting up Access Point...");

  // Set WiFi to Access Point Mode
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  // Configure the AP
  // (Optional: You can configure specific IP, Gateway, Subnet here if needed,
  // but default 192.168.4.1 is fine)

  bool result = WiFi.softAP(AP_SSID, AP_PASS);

  if (result) {
    Serial.println("AP Created Successfully");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());  // Should print 192.168.4.1
  } else {
    Serial.println("AP Creation Failed!");
  }
}

void handleIncomingJson(String data) {
  JsonDocument doc;
  if (deserializeJson(doc, data)) {
    Serial.println(F("JSON Error"));
    return;
  }

  const char *messageType = doc["message_type"] | "";

  if (strcmp(messageType, "command") == 0) {
    double newLeft = doc["left"] | 0.0;
    double newRight = doc["right"] | 0.0;

    portENTER_CRITICAL(&sharedVarMux);
    sharedCommand.targetL = newLeft;
    sharedCommand.targetR = newRight;
    portEXIT_CRITICAL(&sharedVarMux);

    Serial.printf("Target Left: %.3f\n", leftWheel.targetSpeed);
    Serial.printf("Target Right: %.3f\n", rightWheel.targetSpeed);

  } else if (strcmp(messageType, "heartbeat") == 0) {
    portENTER_CRITICAL(&sharedVarMux);
    sharedCommand.lastHeartbeat = esp_timer_get_time();
    portEXIT_CRITICAL(&sharedVarMux);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                    ip[1], ip[2], ip[3], payload);
    } break;
    case WStype_TEXT: {
      String data = String((char *)payload);
      handleIncomingJson(data);
    } break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

//

//         // TELEMETRY & CAMERA
//         //

void sendTelemetry() {
  if (webSocket.connectedClients() == 0) return;

  static int64_t lastTime = 0;
  int64_t now = esp_timer_get_time();
  if (now - lastTime < TELEMETRY_PERIOD_MICROS) return;
  lastTime = now;

  MotorState stateSnapshot;
  portENTER_CRITICAL(&sharedVarMux);
  stateSnapshot = sharedState;
  portEXIT_CRITICAL(&sharedVarMux);

  JsonDocument doc;
  doc["message_type"] = "telemetry";
  doc["timestamp"] = now;

  JsonObject left = doc["left"].to<JsonObject>();
  left["encoder"] = stateSnapshot.encL;
  left["raw_speed"] = stateSnapshot.rawL;
  left["filtered_speed"] = stateSnapshot.filtL;
  left["target_speed"] = stateSnapshot.targetL;
  left["debug_f"] = stateSnapshot.debug_fL;
  left["debug_p"] = stateSnapshot.debug_pL;
  left["debug_i"] = stateSnapshot.debug_iL;
  left["debug_out"] = stateSnapshot.debug_outL;

  JsonObject right = doc["right"].to<JsonObject>();
  right["encoder"] = stateSnapshot.encR;
  right["raw_speed"] = stateSnapshot.rawR;
  right["filtered_speed"] = stateSnapshot.filtR;
  right["target_speed"] = stateSnapshot.targetR;
  right["debug_f"] = stateSnapshot.debug_fR;
  right["debug_p"] = stateSnapshot.debug_pR;
  right["debug_i"] = stateSnapshot.debug_iR;
  right["debug_out"] = stateSnapshot.debug_outR;

  String payload;
  serializeJson(doc, payload);
  webSocket.broadcastTXT(payload);
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
  config.pixel_format = PIXFORMAT_JPEG;

  config.fb_location = CAMERA_FB_IN_PSRAM;

  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality =
      10;  // 0-63. 10-12 is good balance. Lower is higher quality
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // 4. Initialize
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  Serial.println("Camera Configured Successfully");
}

void streamCamera(int64_t now) {
  static int64_t lastFrame = 0;

  if (now - lastFrame < FRAME_STREAM_PERIOD_MICROS) return;
  if (webSocket.connectedClients() == 0) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Couldn't get fb");
    return;
  };

  int64_t sendStart = esp_timer_get_time();
  webSocket.broadcastBIN(fb->buf, fb->len);
  int64_t sendEnd = esp_timer_get_time();
  int64_t sendDuration = sendEnd - sendStart;
  Serial.printf("[Send] Frame size: %zu bytes, Time: %lld us (%.3f ms)\n",
                fb->len, sendDuration, sendDuration / 1000.0);
  lastFrame = now;

  esp_camera_fb_return(fb);
}

// Task Handle to manage the task if needed
TaskHandle_t motorTaskHandle = NULL;

// void motorTask(void *parameter) {
//   const TickType_t xFrequency = pdMS_TO_TICKS(20);
//   TickType_t xLastWakeTime = xTaskGetTickCount();

//   for (;;) {
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);

//     // --- PHASE 1: IMPORT DATA (CRITICAL SECTION) ---
//     double tL, tR;
//     int64_t hb;

//     portENTER_CRITICAL(&sharedVarMux);
//     tL = sharedCommand.targetL;
//     tR = sharedCommand.targetR;
//     hb = sharedCommand.lastHeartbeat;
//     portEXIT_CRITICAL(&sharedVarMux);

//     // --- PHASE 2: LOGIC & PHYSICS (NO LOCKS REQUIRED) ---
//     // The wheels are "private" to this task now. safely write to them.

//     // Failsafe Logic
//     int64_t now = esp_timer_get_time();
//     if (hb > 0 && (now - hb) > HEARTBEAT_TIMEOUT_MICROS) {
//       leftWheel.stop();
//       rightWheel.stop();
//       // Optional: Update shared heartbeat to 0 to signal stop?
//       // Usually not strictly necessary if client sends new hb later.
//     } else {
//       leftWheel.targetSpeed = tL;
//       rightWheel.targetSpeed = tR;

//       leftWheel.update();
//       rightWheel.update();
//     }

//     // --- PHASE 3: EXPORT DATA (CRITICAL SECTION) ---
//     portENTER_CRITICAL(&sharedVarMux);
//     sharedState.encL = leftWheel.encoderCount;
//     sharedState.filtL = leftWheel.filteredSpeed;
//     sharedState.targetL = leftWheel.targetSpeed;
//     sharedState.debug_fL = leftWheel.debug_f;
//     sharedState.debug_pL = leftWheel.debug_p;
//     sharedState.debug_iL = leftWheel.debug_i;
//     sharedState.debug_outL = leftWheel.debug_out;

//     sharedState.encR = rightWheel.encoderCount;
//     sharedState.filtR = rightWheel.filteredSpeed;
//     sharedState.targetR = rightWheel.targetSpeed;
//     sharedState.debug_fR = rightWheel.debug_f;
//     sharedState.debug_pR = rightWheel.debug_p;
//     sharedState.debug_iR = rightWheel.debug_i;
//     sharedState.debug_outR = rightWheel.debug_out;
//     portEXIT_CRITICAL(&sharedVarMux);
//   }
// }

void IRAM_ATTR onControlLoop(void *arg) {
  // --- PHASE 1: IMPORT DATA (CRITICAL SECTION) ---
  double tL, tR;
  int64_t hb;

  // Use a spinlock (portMUX) which is safe in ISRs
  portENTER_CRITICAL_ISR(&sharedVarMux);
  tL = sharedCommand.targetL;
  tR = sharedCommand.targetR;
  hb = sharedCommand.lastHeartbeat;
  portEXIT_CRITICAL_ISR(&sharedVarMux);

  // --- PHASE 2: LOGIC & PHYSICS ---
  int64_t now = esp_timer_get_time();

  // Failsafe Logic
  if (hb > 0 && (now - hb) > HEARTBEAT_TIMEOUT_MICROS) {
    leftWheel.stop();
    rightWheel.stop();
  } else {
    leftWheel.targetSpeed = tL;
    rightWheel.targetSpeed = tR;

    leftWheel.update();
    rightWheel.update();
  }

  // --- PHASE 3: EXPORT DATA ---
  portENTER_CRITICAL_ISR(&sharedVarMux);
  sharedState.encL = leftWheel.encoderCount;
  sharedState.filtL = leftWheel.filteredSpeed;
  sharedState.targetL = leftWheel.targetSpeed;
  sharedState.debug_fL = leftWheel.debug_f;
  sharedState.debug_pL = leftWheel.debug_p;
  sharedState.debug_iL = leftWheel.debug_i;
  sharedState.debug_outL = leftWheel.debug_out;

  sharedState.encR = rightWheel.encoderCount;
  sharedState.filtR = rightWheel.filteredSpeed;
  sharedState.targetR = rightWheel.targetSpeed;
  sharedState.debug_fR = rightWheel.debug_f;
  sharedState.debug_pR = rightWheel.debug_p;
  sharedState.debug_iR = rightWheel.debug_i;
  sharedState.debug_outR = rightWheel.debug_out;
  portEXIT_CRITICAL_ISR(&sharedVarMux);
}

//
// MAIN
//

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupCamera();

  setupAccessPoint();

  // Hardware Setup
  leftWheel.begin();
  attachInterrupt(digitalPinToInterrupt(leftWheel.pinEnc), isrLeft, CHANGE);

  rightWheel.begin();
  attachInterrupt(digitalPinToInterrupt(rightWheel.pinEnc), isrRight, CHANGE);

  // WebSocket Server Setup
  webSocket.onEvent(webSocketEvent);
  webSocket.begin();
  IPAddress IP = WiFi.softAPIP();
  Serial.print("WebSocket server started on ws://");
  Serial.print(IP);
  Serial.print(":");
  Serial.println(WS_PORT);

  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &onControlLoop, .name = "control_loop"};

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &controlTimer));

  // Start the timer.
  // 10000 micros = 10ms (100Hz Control Loop).
  // This gives a 50Hz Dither frequency, which is much smoother.
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(controlTimer, CONTROL_PERIOD_MICROS));

  Serial.println("High-Precision Control Loop Started");
}

void loop() {
  int64_t now = esp_timer_get_time();

  webSocket.loop();

  sendTelemetry();
  streamCamera(now);
}