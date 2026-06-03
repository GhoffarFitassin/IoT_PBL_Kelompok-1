#include "camera.h"
#include "esp_task_wdt.h"
#include <WebSocketsClient.h>

// Access to WebSocket from main.cpp to keep connection alive during camera ops
extern WebSocketsClient webSocket;

// ---------------------------------------------------------------------------
// Camera Pin Configuration -- AI-Thinker ESP32-CAM
// Replace the entire block below for a different board.
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ---------------------------------------------------------------------------
// Image / PSRAM Settings
// ---------------------------------------------------------------------------
#define JPEG_QUALITY 12
#define FRAME_SIZE FRAMESIZE_SVGA

// ---------------------------------------------------------------------------
// Public Functions
// ---------------------------------------------------------------------------

bool cameraInit()
{
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;
  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;
  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;
  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    cfg.frame_size = FRAME_SIZE;
    cfg.jpeg_quality = JPEG_QUALITY;
    cfg.fb_count = 2;
  }
  else
  {
    cfg.frame_size = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK)
    return false;
  return true;
}

camera_fb_t* cameraCaptureFrame(bool forceFresh)
{
  // If fresh frame is needed (e.g., server request), flush cached buffer first
  if (forceFresh)
  {
    camera_fb_t *discard = esp_camera_fb_get();
    if (discard)
    {
      esp_camera_fb_return(discard);
      delay(5); // Brief delay to allow new frame capture
      webSocket.loop(); // Keep connection alive during delay
    }
  }
  
  // Try up to 3 times with 1s timeout each (max 3s total)
  // Faster failure detection keeps device responsive to server
  for (int attempt = 0; attempt < 3; attempt++)
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) // 1 second timeout per attempt
    {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb)
        return fb;
      
      esp_task_wdt_reset(); // Feed watchdog during camera capture wait
      delay(50); // Poll every 50ms - don't hammer camera hardware
      webSocket.loop(); // Keep connection alive during polling
    }
    
    // Attempt failed - brief delay before retry
    if (attempt < 2)
    {
      esp_task_wdt_reset();
      delay(100); // 100ms between retry attempts
      webSocket.loop(); // Keep connection alive between retries
    }
  }
  
  // All 3 attempts failed
  return nullptr;
}

void cameraReleaseFrame(camera_fb_t* fb)
{
  if (fb)
  {
    esp_camera_fb_return(fb);
  }
}

char* cameraEncodeBase64(camera_fb_t* fb, size_t* outLen)
{
  if (!fb)
    return nullptr;

  // Base64 encoding characters
  static const char b64chars[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  
  size_t inLen = fb->len;
  size_t b64Len = ((inLen + 2) / 3) * 4 + 1;
  char *b64Buf = (char *)malloc(b64Len);
  
  if (!b64Buf)
    return nullptr;

  uint8_t *in = fb->buf;
  char *out = b64Buf;
  
  for (size_t i = 0; i < inLen; i += 3)
  {
    uint32_t n = ((uint32_t)in[i] << 16) | 
                 (i + 1 < inLen ? (uint32_t)in[i + 1] << 8 : 0) | 
                 (i + 2 < inLen ? (uint32_t)in[i + 2] : 0);
    *out++ = b64chars[(n >> 18) & 63];
    *out++ = b64chars[(n >> 12) & 63];
    *out++ = (i + 1 < inLen) ? b64chars[(n >> 6) & 63] : '=';
    *out++ = (i + 2 < inLen) ? b64chars[n & 63] : '=';
  }
  *out = '\0';

  if (outLen)
    *outLen = b64Len;

  return b64Buf;
}
