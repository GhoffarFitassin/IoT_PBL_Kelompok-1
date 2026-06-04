#include "camera.h"
#include "esp_task_wdt.h"

// ---------------------------------------------------------------------------
// Camera Pin Configuration -- AI-Thinker ESP32-CAM
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
// Timeout & Failure Constants
// ---------------------------------------------------------------------------
#define CAPTURE_TASK_TIMEOUT_MS 1000
#define CAPTURE_TASK_STACK (3 * 1024)

// ---------------------------------------------------------------------------
// CaptureTaskData — STATIC allocation so the capture task can never write
// to a freed stack frame if the caller times out and returns early.
// Only one capture runs at a time so a single static instance is safe.
// ---------------------------------------------------------------------------
typedef struct
{
  camera_fb_t *fb;
  volatile bool done;
  bool forceFresh;
} CaptureTaskData;

static CaptureTaskData s_captureData;

// ---------------------------------------------------------------------------
// FreeRTOS capture task — pinned to core 0, isolated from ws_task & loop()
// ---------------------------------------------------------------------------
static void captureTaskFunc(void *pvParams)
{
  CaptureTaskData *d = (CaptureTaskData *)pvParams;

  if (d->forceFresh)
  {
    camera_fb_t *discard = esp_camera_fb_get();
    if (discard)
    {
      esp_camera_fb_return(discard);
      vTaskDelay(pdMS_TO_TICKS(25));
    }
  }

  d->fb = esp_camera_fb_get();
  d->done = true; // visible to caller: write after fb so compiler can't reorder
  vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// captureWithTimeout
//
// Spawns captureTaskFunc on core 0 and waits up to timeoutMs.
// Uses the static s_captureData to avoid dangling-pointer UB when the caller
// times out before the task writes its result.
// If the task times out it is killed — the camera DMA will eventually finish
// and write to s_captureData.fb, but done=true is never set so the next call
// to captureWithTimeout reinitialises s_captureData before spawning again,
// meaning the stale write is harmless.
// ---------------------------------------------------------------------------
static camera_fb_t *captureWithTimeout(bool forceFresh, unsigned long timeoutMs)
{
  s_captureData.fb = nullptr;
  s_captureData.done = false;
  s_captureData.forceFresh = forceFresh;

  TaskHandle_t taskHandle = NULL;
  BaseType_t created = xTaskCreatePinnedToCore(
      captureTaskFunc,
      "cam_cap",
      CAPTURE_TASK_STACK,
      &s_captureData,
      3, // moderate priority; below ws_task (5), above loop() (1)
      &taskHandle,
      0 // core 0 — isolated from WebSocket and Arduino loop
  );

  if (created != pdPASS)
  {
    // OOM fallback: synchronous capture on calling core
    unsigned long t0 = millis();
    while (millis() - t0 < timeoutMs)
    {
      camera_fb_t *fb = esp_camera_fb_get();
      esp_task_wdt_reset();
      if (fb)
        return fb;
      vTaskDelay(pdMS_TO_TICKS(25));
    }
    return nullptr;
  }

  // Wait for task completion or timeout
  unsigned long t0 = millis();
  while (!s_captureData.done)
  {
    if (millis() - t0 >= timeoutMs)
    {
      vTaskDelete(taskHandle);
      // s_captureData is static — safe to leave; will be reset on next call
      return nullptr;
    }
    esp_task_wdt_reset();
    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  return s_captureData.fb;
}

// ---------------------------------------------------------------------------
// cameraInit
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
    cfg.fb_count = 1;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  }
  else
  {
    cfg.frame_size = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK)
    return false;

  return true;
}

camera_fb_t *cameraCaptureFrame(bool forceFresh)
{

  camera_fb_t *fb = captureWithTimeout(forceFresh, CAPTURE_TASK_TIMEOUT_MS);
  return fb;
}

// ---------------------------------------------------------------------------
// cameraReleaseFrame
// ---------------------------------------------------------------------------
void cameraReleaseFrame(camera_fb_t *fb)
{
  if (fb)
    esp_camera_fb_return(fb);
}

// ---------------------------------------------------------------------------
// cameraEncodeBase64
// ---------------------------------------------------------------------------
char *cameraEncodeBase64(camera_fb_t *fb, size_t *outLen)
{
  if (!fb)
    return nullptr;

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
