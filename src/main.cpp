#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"

// Import all modules
#include "config.h"
#include "temperature.h"
#include "led_control.h"
#include "camera.h"
#include "ability_handler.h"

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
WebSocketsClient webSocket;

volatile bool wsConnected = false;
volatile bool handshakeDone = false;

DeviceMode deviceMode = MODE_NORMAL;
String previewRoomId = "";
bool liveStreamEnabled = false;
int tempIntervalSec = TEMP_INTERVAL_DEFAULT;
int countIntervalSec = COUNT_INTERVAL_DEFAULT;

unsigned long lastTempMs = 0;
unsigned long lastImageMs = 0;
bool imageInFlight = false;
String imageInFlightUploadId = "";

// ---------------------------------------------------------------------------
// Acknowledgment (ACK) Management - Optimized Linear Map
// ---------------------------------------------------------------------------
#define ACK_MAP_SIZE 8

enum AckKind
{
  ACK_NONE,
  ACK_IMAGE_START,
  ACK_IMAGE_COMPLETE
};

struct PendingAck
{
  bool active = false;
  bool resolved = false;
  bool accepted = false;
  AckKind kind = ACK_NONE;
  String uploadId = "";
};

static PendingAck ackMap[ACK_MAP_SIZE];

// Helper to avoid heavy string comparisons where possible
static AckKind getAckKind(const char *kindStr)
{
  if (strcmp(kindStr, "image.start") == 0)
    return ACK_IMAGE_START;
  if (strcmp(kindStr, "image.complete") == 0)
    return ACK_IMAGE_COMPLETE;
  return ACK_NONE;
}

// O(N) linear scan (N=8). Faster and avoids heap fragmentation from String concat.
static PendingAck *ackAlloc(AckKind kind, const String &uploadId = "")
{
  if (kind == ACK_NONE)
    return nullptr;

  for (int i = 0; i < ACK_MAP_SIZE; i++)
  {
    PendingAck &slot = ackMap[i];

    // Duplicate guard: already waiting on this exact payload
    if (slot.active && !slot.resolved && slot.kind == kind && slot.uploadId == uploadId)
    {
      return nullptr;
    }

    // Free or already-resolved slot -- claim it
    if (!slot.active || slot.resolved)
    {
      slot.active = true;
      slot.resolved = false;
      slot.accepted = false;
      slot.kind = kind;
      slot.uploadId = uploadId;
      return &slot;
    }
  }

  // Map is strictly full. Returning nullptr safely aborts the send instead
  // of maliciously evicting a pending ACK that another thread/loop is waiting on.
  return nullptr;
}

static void ackDispatch(const char *kindStr, const char *uploadId, bool accepted)
{
  AckKind kind = getAckKind(kindStr);
  if (kind == ACK_NONE)
    return;

  for (int i = 0; i < ACK_MAP_SIZE; i++)
  {
    PendingAck &slot = ackMap[i];
    if (slot.active && !slot.resolved && slot.kind == kind && slot.uploadId == uploadId)
    {
      slot.accepted = accepted;
      slot.resolved = true;
      return;
    }
  }
}

static bool ackImageActive(const String &uploadId)
{
  for (int i = 0; i < ACK_MAP_SIZE; i++)
  {
    PendingAck &slot = ackMap[i];
    if (slot.active && !slot.resolved &&
        (slot.kind == ACK_IMAGE_START || slot.kind == ACK_IMAGE_COMPLETE) &&
        slot.uploadId == uploadId)
    {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Deferred action flags
// ---------------------------------------------------------------------------
static bool pendingSendHandshake = false;
static bool pendingSendAbilityDecl = false;
static bool pendingSendCapture = false;

static String pendingAbilityMethod;
static String pendingAbilityName;
static String pendingAbilityRequestId;
static ClassLedState pendingAbilityClassLightState = CLASS_LED_OFF;
static bool pendingAbilityNoOutput = false;

// ---------------------------------------------------------------------------
// Send helpers
// ---------------------------------------------------------------------------

// Wraps a JsonDocument in the protocol envelope and sends as TEXT frame.
static bool sendPayload(JsonDocument &doc, bool initial = false)
{
  if (!wsConnected)
    return false;

  JsonDocument env;
  env["type"] = initial ? "subscribe" : "push";
  env["channel"] = "device.client";
  env["payload"] = doc;

  String out;
  serializeJson(env, out);
  webSocket.sendTXT(out);
  return true;
}

static bool waitAck(PendingAck *slot)
{
  if (!slot)
    return false; // duplicate was suppressed or map full; treat as rejected

  unsigned long t0 = millis();
  while (!slot->resolved)
  {
    // Early exit if WebSocket disconnected while waiting
    if (!wsConnected)
    {
      slot->active = false;
      return false;
    }
    webSocket.loop();
    yield();
    if (millis() - t0 > ACK_TIMEOUT_MS)
    {
      slot->active = false;
      return false;
    }
    // Feed the watchdog so a long wait doesn't cause a random reset
    esp_task_wdt_reset();
    taskYIELD();
  }

  bool accepted = slot->accepted;
  slot->active = false; // Important: explicitly free the slot once resolved
  return accepted;
}

// ---------------------------------------------------------------------------
// Protocol -- handshake
// ---------------------------------------------------------------------------
static void sendHandshake()
{
  JsonDocument doc;
  doc["kind"] = "handshake";
  doc["uuid"] = DEVICE_UUID;
  sendPayload(doc, /*initial=*/true);
}

// ---------------------------------------------------------------------------
// Protocol -- ability declaration
// ---------------------------------------------------------------------------
static void sendAbilityDeclaration()
{
  JsonDocument doc;
  generateAbilityDeclaration(doc);
  sendPayload(doc);
}

// ---------------------------------------------------------------------------
// Protocol -- temperature telemetry
// ---------------------------------------------------------------------------
static void sendTemperature()
{
  float t, h;

  // Combined read: ONE sensor cycle, WDT safety, failure backoff built in
  esp_task_wdt_reset();
  if (!readTemperatureAndHumidity(t, h))
    return;
  esp_task_wdt_reset();

  setTemperatureLed(t);

  JsonDocument doc;
  doc["kind"] = "temperature";
  doc["uuid"] = DEVICE_UUID;
  doc["temperature"] = t;
  doc["humidity"] = h;
  doc["mode"] = modeStr(deviceMode);
  if (previewRoomId.length())
    doc["roomId"] = previewRoomId;

  sendPayload(doc);
}

// ---------------------------------------------------------------------------
// Protocol -- chunked image upload
// ---------------------------------------------------------------------------
static void sendChunkedImage(bool isRequested = false)
{
  if (imageInFlight && !isRequested)
  {
    return;
  }

  // Check if camera is temporarily disabled due to repeated failures
  unsigned long now = millis();

  camera_fb_t *fb = cameraCaptureFrame(isRequested);
  if (!fb)
    return;

  imageInFlight = true;
  String uploadId = makeUploadId();
  imageInFlightUploadId = uploadId;
  size_t totalBytes = fb->len;

  PendingAck *ackStart = ackAlloc(ACK_IMAGE_START, uploadId);
  PendingAck *ackComplete = nullptr;

  // ---- 1. image.start ----------------------------------------------------
  {
    JsonDocument doc;
    doc["kind"] = "image.start";
    doc["uuid"] = DEVICE_UUID;
    doc["uploadId"] = uploadId;
    doc["mimeType"] = "image/jpeg";
    doc["totalBytes"] = (int)totalBytes;
    doc["chunkSize"] = IMAGE_CHUNK_SIZE;
    if (!sendPayload(doc))
    {
      if (ackStart)
        ackStart->active = false;
      goto cleanup;
    }
  }

  if (!waitAck(ackStart))
  {
    goto cleanup;
  }

  // ---- 2. Binary chunks --------------------------------------------------
  {
    size_t offset = 0;
    int sendFailCount = 0;
    while (offset < totalBytes)
    {
      esp_task_wdt_reset();
      size_t chunkLen = min((size_t)IMAGE_CHUNK_SIZE, totalBytes - offset);

      if (webSocket.sendBIN(fb->buf + offset, chunkLen))
      {
        offset += chunkLen;
        sendFailCount = 0;
        esp_task_wdt_reset();
      }
      else
      {
        sendFailCount++;

        if (sendFailCount > 5)
        {
          break;
        }
        esp_task_wdt_reset();
        yield();
        webSocket.loop();
      }
    }

    // If we aborted due to network backpressure, cancel the rest of the process
    if (sendFailCount > 0)
    {
      goto cleanup;
    }
  }

  // ---- 3. image.complete -------------------------------------------------
  ackComplete = ackAlloc(ACK_IMAGE_COMPLETE, uploadId);
  {
    JsonDocument doc;
    doc["kind"] = "image.complete";
    doc["uuid"] = DEVICE_UUID;
    doc["uploadId"] = uploadId;
    doc["mode"] = modeStr(deviceMode);
    if (previewRoomId)
      doc["roomId"] = previewRoomId;
    if (liveStreamEnabled)
      doc["liveStream"] = true;
    if (isRequested)
      doc["isRequested"] = true;

    if (!sendPayload(doc))
    {
      if (ackComplete)
        ackComplete->active = false;
      goto cleanup;
    }
  }

  waitAck(ackComplete);

cleanup:
  cameraReleaseFrame(fb);
  imageInFlight = false;
  imageInFlightUploadId = "";
}

// ---------------------------------------------------------------------------
// WebSocket event callback
// ---------------------------------------------------------------------------
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
    wsConnected = true;
    handshakeDone = false;
    pendingSendHandshake = true;
    break;

  case WStype_DISCONNECTED:
    ESP.restart();
    break;

  // ---- text frame --------------------------------------------------------
  case WStype_TEXT:
  {

    JsonDocument data;
    DeserializationError derr = deserializeJson(data, payload, length);
    if (derr != DeserializationError::Ok)
    {
      break;
    }

    const char *msgType = data["type"] | "";

    if (strcmp(msgType, "ready") == 0)
    {
      break;
    }
    if (strcmp(msgType, "error") == 0)
    {
      break;
    }
    if (strcmp(msgType, "data") != 0)
      break;

    const char *channel = data["channel"] | "";
    if (strcmp(channel, "device.client") != 0)
      break;
    JsonObject msg;
    if (data["data"].is<JsonObject>())
    {
      msg = data["data"].as<JsonObject>();
    }
    else if (data["payload"].is<JsonObject>())
    {
      msg = data["payload"].as<JsonObject>();
    }
    else
    {
      break;
    }
    const char *kind = msg["kind"] | "";

    // ---- Resolve pending ACK (image.start / image.complete only) ------
    if (strcmp(kind, "image.start") == 0 || strcmp(kind, "image.complete") == 0)
    {
      const char *uploadId = msg["uploadId"] | "";
      bool accepted = msg["accepted"] | false;
      ackDispatch(kind, uploadId, accepted);
      break; // ACK messages need no further dispatch
    }

    if (!(msg["accepted"] | false) && strcmp(kind, "ability.request") != 0)
      break;

    // ---- Normal dispatch ------------------------------------------------
    if (strcmp(kind, "handshake") == 0)
    {
      tempIntervalSec = max(1, (int)(msg["tempFetchInterval"] | 60));
      countIntervalSec = max(1, (int)(msg["countFetchInterval"] | 60));
      deviceMode = parseMode(msg["mode"] | "", MODE_NORMAL);
      handshakeDone = true;

      pendingSendAbilityDecl = true;
    }
    else if (strcmp(kind, "config.update") == 0)
    {
      if (msg["roomId"].is<const char *>())
      {
        previewRoomId = msg["roomId"].as<const char *>();
      }
      else
      {
        int roomId = msg["roomId"] | -1;
        previewRoomId = roomId >= 0 ? String(roomId) : "";
      }
      liveStreamEnabled = msg["liveStream"] | false;
      deviceMode = parseMode(msg["mode"] | "", deviceMode);
      tempIntervalSec = max(1, (int)(msg["tempFetchInterval"] | tempIntervalSec));
      countIntervalSec = max(1, (int)(msg["countFetchInterval"] | countIntervalSec));
    }
    else if (strcmp(kind, "capture.request") == 0)
    {
      pendingSendCapture = true;
    }
    else if (strcmp(kind, "ability.request") == 0)
    {
      pendingAbilityMethod = msg["method"] | "";
      pendingAbilityName = msg["ability"] | "";
      if (pendingAbilityMethod.isEmpty())
        pendingAbilityMethod = msg["command"] | "";
      normalizeAbilityCommand(pendingAbilityMethod, pendingAbilityName);
      pendingAbilityRequestId = msg["requestId"] | "";

      // Parse --no-output flag
      pendingAbilityNoOutput = (pendingAbilityName.indexOf("--no-output") >= 0);
      if (pendingAbilityNoOutput)
      {
        pendingAbilityName.replace("--no-output", "");
        pendingAbilityName.trim();
      }

      // Parse class light state from the message
      const char *valueStr = msg["value"] | "";
      if (valueStr && valueStr[0] != '\0')
        pendingAbilityClassLightState = parseClassLightValue(String(valueStr));
      else
        pendingAbilityClassLightState = CLASS_LED_OFF;
    }
    break;
  }

  case WStype_BIN:
    break;

  default:
    break;
  }
}

// ---------------------------------------------------------------------------
// WiFi connect helper
// ---------------------------------------------------------------------------
static bool connectWifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    esp_task_wdt_reset(); // Feed watchdog during WiFi connection wait
    if (millis() - t0 >= (unsigned long)WIFI_CONNECT_TIMEOUT_MS)
    {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// WebSocket connect helper
// ---------------------------------------------------------------------------
static void startWebSocket()
{
#if WS_SSL_ENABLED
  webSocket.beginSSL(WS_HOST, atoi(ENV_WS_PORT), WS_PATH);
#else
  webSocket.begin(WS_HOST, atoi(ENV_WS_PORT), WS_PATH);
#endif
  webSocket.enableHeartbeat(15000, 5000, 7);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
  // Serial.end(); // Commented out - can interfere with debugging

  // Initialize hardware modules
  ledInit();
  temperatureInit();

  // ---- Configure Watchdog Timer (x second timeout) ----------------------
  esp_task_wdt_init(7, true); // x sec timeout, panic on timeout
  esp_task_wdt_add(NULL);     // Add current thread to WDT watch

  connectWifi();

  if (!cameraInit())
  {
    ESP.restart();
  }

  // ---- Configure links2004/WebSockets ------------------------------------
  startWebSocket();
  webSocket.onEvent(webSocketEvent);

  String hdrs = "X-Auth-Method: deviceUuid\r\nX-Device-UUID: ";
  hdrs += DEVICE_UUID;
  webSocket.setExtraHeaders(hdrs.c_str());

  esp_task_wdt_reset();
}

void loop()
{
  // Feed watchdog at start of every loop iteration
  esp_task_wdt_reset();

  if (WiFi.status() != WL_CONNECTED)
  {
    wsConnected = false;
    connectWifi();
  }

  webSocket.loop();

  // ---- Process deferred actions ----------------------------------------
  if (pendingSendHandshake)
  {
    pendingSendHandshake = false;
    sendHandshake();
  }
  if (pendingSendAbilityDecl)
  {
    pendingSendAbilityDecl = false;
    sendAbilityDeclaration();
  }
  if (pendingSendCapture)
  {
    pendingSendCapture = false;
    sendChunkedImage(true);
  }
  if (!pendingAbilityMethod.isEmpty())
  {
    String m = pendingAbilityMethod;
    String a = pendingAbilityName;
    String r = pendingAbilityRequestId;
    ClassLedState state = pendingAbilityClassLightState;
    pendingAbilityMethod = "";
    pendingAbilityName = "";
    pendingAbilityRequestId = "";
    pendingAbilityClassLightState = CLASS_LED_OFF;

    JsonDocument resp;
    if (handleAbilityRequest(m, a, r, state, resp))
    {
      if (!pendingAbilityNoOutput)
        sendPayload(resp);
    }
    pendingAbilityNoOutput = false;
  }
  if (!wsConnected || !handshakeDone || deviceMode == MODE_IDLE)
    return;

  unsigned long now = millis();

  // ---- Periodic temperature ----------------------------------------------
  if (now - lastTempMs >= (unsigned long)tempIntervalSec * 1000UL)
  {
    lastTempMs = now;
    sendTemperature();
  }

  // ---- Periodic / live image ---------------------------------------------
  if (liveStreamEnabled)
  {
    // Live stream -- honour LIVESTREAM_FPS cap (-1 = uncapped)
#if LIVESTREAM_FPS > 0
    static unsigned long lastLiveMs = 0;
    const unsigned long liveIntervalMs = 1000UL / LIVESTREAM_FPS;
    if (!ackImageActive(imageInFlightUploadId) && (now - lastLiveMs >= liveIntervalMs))
    {
      lastLiveMs = now;
      sendChunkedImage();
    }
#else
    if (!ackImageActive(imageInFlightUploadId))
    {
      sendChunkedImage();
    }
#endif
  }
  else if (now - lastImageMs >= (unsigned long)countIntervalSec * 1000UL)
  {
    lastImageMs = now;
    sendChunkedImage();
  }

  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(1));
}