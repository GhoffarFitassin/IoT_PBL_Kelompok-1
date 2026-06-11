#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include "esp_camera.h"

// ---------------------------------------------------------------------------
// Camera Module - ESP32-CAM with crash-safe capture
// ---------------------------------------------------------------------------

// Initialize the camera with AI-Thinker ESP32-CAM configuration
bool cameraInit();

// Deinitialize the camera (pull PWDN high to reset sensor hardware)
// Safe to call even if camera is hung
void cameraDeinit();

// Capture a single frame and return frame buffer pointer.
// forceFresh: if true, discards cached buffer to ensure fresh frame
// Caller must call cameraReleaseFrame() when done
// Returns nullptr on timeout or hardware failure (does NOT hang forever)
camera_fb_t* cameraCaptureFrame(bool forceFresh = false);

// Release frame buffer after use
void cameraReleaseFrame(camera_fb_t* fb);

// Encode frame buffer to base64 string
// Returns allocated buffer that must be freed by caller
// Returns nullptr on allocation failure
char* cameraEncodeBase64(camera_fb_t* fb, size_t* outLen);
#endif // CAMERA_H