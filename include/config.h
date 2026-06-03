#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Environment variables (injected at build time)
// ---------------------------------------------------------------------------
#ifndef ENV_WIFI_SSID
#define ENV_WIFI_SSID ""
#endif

#ifndef ENV_WIFI_PASSWORD
#define ENV_WIFI_PASSWORD ""
#endif

#ifndef ENV_WS_HOST
#define ENV_WS_HOST ""
#endif

#ifndef ENV_WS_PORT
#define ENV_WS_PORT ""
#endif

#ifndef ENV_WS_SSL_ENABLED
#define ENV_WS_SSL_ENABLED ""
#endif

#ifndef WS_SSL_ENABLED
#define WS_SSL_ENABLED 0
#endif

#ifndef ENV_DEVICE_UUID
#define ENV_DEVICE_UUID ""
#endif

// ---------------------------------------------------------------------------
// WiFi Configuration
// ---------------------------------------------------------------------------
extern const char *WIFI_SSID;
extern const char *WIFI_PASSWORD;

// ---------------------------------------------------------------------------
// Server & Device Identity
// ---------------------------------------------------------------------------
extern const char *WS_HOST;
extern const uint16_t WS_PORT;
extern const char *WS_PATH;
extern const char *DEVICE_UUID;

// ---------------------------------------------------------------------------
// Protocol Constants
// ---------------------------------------------------------------------------
#define IMAGE_CHUNK_SIZE (8 * 1024)
#define TEMP_INTERVAL_DEFAULT 60
#define COUNT_INTERVAL_DEFAULT 60
#define ACK_TIMEOUT_MS 30000
#define RECONNECT_DELAY_MS 5000
#define LIVESTREAM_FPS -1  // max frames per second during live stream; -1 = uncapped

// ---------------------------------------------------------------------------
// Device Mode
// ---------------------------------------------------------------------------
enum DeviceMode
{
  MODE_NORMAL,
  MODE_FOCUS,
  MODE_IDLE
};

DeviceMode parseMode(const char *s, DeviceMode def = MODE_NORMAL);
const char *modeStr(DeviceMode m);

// ---------------------------------------------------------------------------
// Class LED State
// ---------------------------------------------------------------------------
enum ClassLedState
{
  CLASS_LED_OFF,
  CLASS_LED_UNUSED,
  CLASS_LED_USED
};

// ---------------------------------------------------------------------------
// Global State (defined in main.cpp)
// ---------------------------------------------------------------------------
extern DeviceMode deviceMode;
extern String previewRoomId;

// ---------------------------------------------------------------------------
// Utility Functions
// ---------------------------------------------------------------------------
bool hasConfigValue(const char *value);
String makeUploadId();

#endif // CONFIG_H
