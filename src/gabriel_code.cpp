// #include <WebSocketsServer.h>
// #include <WiFi.h>
// #include <esp_camera.h>

// #include "camera_pins.h"

// // Note: Your original code used custom pins in the struct.
// // I have preserved YOUR specific pin mapping below.

// const char* network = "ponto_de_acesso";
// const char* password = "senha_foda";

// WebSocketsServer webSocket = WebSocketsServer(81);

// unsigned long lastFrameTime = 0;
// #define FRAME_INTERVAL 100  // Minimum ms between frames

// // --- FUNCTIONS ---

// void frame() {
//   camera_fb_t* frameBuffer = esp_camera_fb_get();

//   if (!frameBuffer) {
//     Serial.println("Camera capture failed!");
//     return;
//   }

//   // Send binary video data to all connected clients
//   int64_t t_before = esp_timer_get_time();
//   webSocket.broadcastBIN(frameBuffer->buf, frameBuffer->len);
//   int64_t t_after = esp_timer_get_time();
//   Serial.printf("Time to send frame: %lld us\n",
//                 (unsigned long long)t_after - t_before);
//   esp_camera_fb_return(frameBuffer);  // Release memory
// }

// void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload,
//                     size_t length) {
//   switch (type) {
//     case WStype_DISCONNECTED:
//       Serial.printf("[%u] Disconnected!\n", num);
//       break;
//     case WStype_CONNECTED: {
//       IPAddress ip = webSocket.remoteIP(num);
//       Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
//                     ip[1], ip[2], ip[3], payload);
//     } break;
//     case WStype_TEXT:
//       // We are just streaming video, but we print text if received for
//       // debugging
//       Serial.printf("[%u] get Text: %s\n", num, payload);
//       break;
//     case WStype_BIN:
//       Serial.printf("[%u] get binary length: %u\n", num, length);
//       break;
//     case WStype_ERROR:
//     case WStype_FRAGMENT_TEXT_START:
//     case WStype_FRAGMENT_BIN_START:
//     case WStype_FRAGMENT:
//     case WStype_FRAGMENT_FIN:
//       break;
//   }
// }

// // --- SETUP ---

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   // WiFi Setup (AP Mode)
//   WiFi.softAP(network, password);
//   IPAddress IP = WiFi.softAPIP();
//   Serial.print("AP IP address: ");
//   Serial.println(IP);

//   // WebSocket Setup
//   webSocket.onEvent(webSocketEvent);
//   webSocket.begin();
//   Serial.println("WebSocket server started on ws://" + IP.toString() +
//   ":81");

//   camera_config_t config;

//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = CAM_PIN_D0;
//   config.pin_d1 = CAM_PIN_D1;
//   config.pin_d2 = CAM_PIN_D2;
//   config.pin_d3 = CAM_PIN_D3;
//   config.pin_d4 = CAM_PIN_D4;
//   config.pin_d5 = CAM_PIN_D5;
//   config.pin_d6 = CAM_PIN_D6;
//   config.pin_d7 = CAM_PIN_D7;
//   config.pin_xclk = CAM_PIN_XCLK;
//   config.pin_pclk = CAM_PIN_PCLK;
//   config.pin_vsync = CAM_PIN_VSYNC;
//   config.pin_href = CAM_PIN_HREF;
//   config.pin_sccb_sda = CAM_PIN_SCCB_SDA;
//   config.pin_sccb_scl = CAM_PIN_SCCB_SCL;
//   config.pin_pwdn = CAM_PIN_PWDN;
//   config.pin_reset = CAM_PIN_RESET;

//   config.xclk_freq_hz = 20000000;  // 20MHz XCLK

//   // config.pixel_format = PIXFORMAT_GRAYSCALE;
//   // config.frame_size = FRAMESIZE_QVGA;
//   // config.jpeg_quality = 10;

//   // config.fb_count = 1;
//   // config.fb_location = CAMERA_FB_IN_PSRAM;
//   // config.grab_mode = CAMERA_GRAB_LATEST;

//   config.pixel_format = PIXFORMAT_JPEG;
//   config.frame_size = FRAMESIZE_QVGA;
//   config.jpeg_quality =
//       10;  // 0-63. 10-12 is good balance. Lower is higher quality
//   config.fb_count = 2;
//   config.grab_mode = CAMERA_GRAB_LATEST;

//   // Camera Init
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK) {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     while (true);
//   }
// }

// // --- LOOP ---

// void loop() {
//   webSocket.loop();

//   unsigned long now = millis();

//   // Broadcast a new frame if enough time has passed
//   if (now - lastFrameTime > FRAME_INTERVAL) {
//     frame();
//     lastFrameTime = now;
//   }
// }