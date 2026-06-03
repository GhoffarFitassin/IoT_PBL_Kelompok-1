#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include "esp_camera.h"

// ---------------------------------------------------------------------------
// Camera Module - ESP32-CAM
// ---------------------------------------------------------------------------

// Initialize the camera with AI-Thinker ESP32-CAM configuration
bool cameraInit();

// Capture a single frame and return frame buffer pointer
// Caller must call cameraReleaseFrame() when done
camera_fb_t* cameraCaptureFrame();

// Release frame buffer after use
void cameraReleaseFrame(camera_fb_t* fb);

// Encode frame buffer to base64 string
// Returns allocated buffer that must be freed by caller
// Returns nullptr on allocation failure
char* cameraEncodeBase64(camera_fb_t* fb, size_t* outLen);

#endif // CAMERA_H
