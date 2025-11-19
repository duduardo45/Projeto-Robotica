/**
 * Camera pin mapping for ESP32-CAM (AI Thinker) modules.
 * Adjust these definitions if you are using a different board or wiring.
 */
#pragma once

#ifndef CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_AI_THINKER
#endif

#if defined(CAMERA_MODEL_AI_THINKER)
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5

#define CAM_PIN_D7 16
#define CAM_PIN_D6 17
#define CAM_PIN_D5 18
#define CAM_PIN_D4 12
#define CAM_PIN_D3 10
#define CAM_PIN_D2 8
#define CAM_PIN_D1 9
#define CAM_PIN_D0 11
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#else
#error "Camera pinout not defined for this board"
#endif
