#include "config.h"

// ---------------------------------------------------------------------------
// WiFi Configuration
// ---------------------------------------------------------------------------
const char *WIFI_SSID = ENV_WIFI_SSID;
const char *WIFI_PASSWORD = ENV_WIFI_PASSWORD;

// ---------------------------------------------------------------------------
// Server & Device Identity
// ---------------------------------------------------------------------------
const char *WS_HOST = ENV_WS_HOST;
const char *WS_PATH = "/ws";
const char *DEVICE_UUID = ENV_DEVICE_UUID;

// ---------------------------------------------------------------------------
// Device Mode Utilities
// ---------------------------------------------------------------------------
DeviceMode parseMode(const char *s, DeviceMode def)
{
  if (!s)
    return def;
  if (strcmp(s, "normal") == 0)
    return MODE_NORMAL;
  if (strcmp(s, "focus") == 0)
    return MODE_FOCUS;
  if (strcmp(s, "idle") == 0)
    return MODE_IDLE;
  return def;
}

const char *modeStr(DeviceMode m)
{
  switch (m)
  {
  case MODE_FOCUS:
    return "focus";
  case MODE_IDLE:
    return "idle";
  default:
    return "normal";
  }
}

// ---------------------------------------------------------------------------
// Utility Functions
// ---------------------------------------------------------------------------
bool hasConfigValue(const char *value)
{
  return value && value[0] != '\0';
}

String makeUploadId()
{
  String id = DEVICE_UUID;
  id.replace("-", "");
  id += '_';
  id += String(millis());
  return id;
}
