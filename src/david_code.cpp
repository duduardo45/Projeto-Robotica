// #include <Arduino.h>

// #define diam 50 /* Em mm */
// #define EncoderPin 18
// #define dt_ms 200 /* Delay para medir o tempo */
// #define PI 3.1415f
// #define PULSES_PER_ROT 64.0f

// #define IN1 16
// #define IN2 17
// #define ENA 15

// volatile int encoder = 0; /* Medicao atual do encoder */
// float rotacoes = 0;
// float vel = 0;
// unsigned long t = 0;

// void IRAM_ATTR encoder_inc() { encoder++; }

// void setup() {
//   delay(3000);

//   neopixelWrite(RGB_BUILTIN, 50, 0, 50);

//   Serial.begin(115200);

//   while (!Serial) {
//     delay(10);
//   }

//   delay(1000);
//   neopixelWrite(RGB_BUILTIN, 0, 50, 0);

//   Serial.println("oioioioi");
//   pinMode(EncoderPin, INPUT);
//   attachInterrupt(digitalPinToInterrupt(EncoderPin), encoder_inc, RISING);
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);

//   pinMode(ENA, OUTPUT);

//   t = millis();
//   Serial.println("oioioioi");
// }

// void loop() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);

//   analogWrite(ENA, 100);

//   if (millis() - t > dt_ms) {
//     rotacoes = (encoder / PULSES_PER_ROT);
//     vel = rotacoes * (60000 / dt_ms);
//     Serial.print("Vel: ");
//     Serial.println(vel);
//     Serial.print("Rotacoes: ");
//     Serial.println(rotacoes);
//     Serial.print("Encoder: ");
//     Serial.println(encoder);
//     t = millis();
//     encoder = 0;
//   }
// }