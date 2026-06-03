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
bool classInUse = false;

unsigned long lastTempMs = 0;
unsigned long lastImageMs = 0;
bool imageInFlight = false;
String imageInFlightUploadId = "";
#define ACK_MAP_SIZE 8  // must be power-of-two; keep load < 0.75

struct PendingAck
{
  bool active   = false;
  bool resolved = false;
  bool accepted = false;
  String key    = "";  // "kind:uploadId"
};
static PendingAck ackMap[ACK_MAP_SIZE];

static uint32_t ackHash(const String &k)
{
  // FNV-1a 32-bit
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < k.length(); i++)
    h = (h ^ (uint8_t)k[i]) * 16777619UL;
  return h;
}

static String ackMakeKey(const String &kind, const String &uploadId)
{
  return kind + ":" + uploadId;
}

// Returns pointer to a new slot, or nullptr if key is already active (duplicate).
// Prints a warning and evicts the oldest colliding slot if the map is full.
static PendingAck *ackAlloc(const String &kind, const String &uploadId = "")
{
  String k = ackMakeKey(kind, uploadId);
  uint32_t idx = ackHash(k) & (ACK_MAP_SIZE - 1);

  // Linear probe
  for (int probe = 0; probe < ACK_MAP_SIZE; probe++)
  {
    PendingAck &slot = ackMap[(idx + probe) & (ACK_MAP_SIZE - 1)];

    // Duplicate guard: same key already waiting
    if (slot.active && !slot.resolved && slot.key == k)
      return nullptr;

    // Free or already-resolved slot -- claim it
    if (!slot.active || slot.resolved)
    {
      slot.active   = true;
      slot.resolved = false;
      slot.accepted = false;
      slot.key      = k;
      return &slot;
    }
  }

  // Map full -- evict the home slot and warn

  PendingAck &evict = ackMap[idx];
  evict.active   = true;
  evict.resolved = false;
  evict.accepted = false;
  evict.key      = k;
  return &evict;
}

// O(1) avg: hash directly to the home bucket, then probe for the key.
static void ackDispatch(const char *kind, const char *uploadId, bool accepted)
{
  String k = ackMakeKey(kind, uploadId);
  uint32_t idx = ackHash(k) & (ACK_MAP_SIZE - 1);

  for (int probe = 0; probe < ACK_MAP_SIZE; probe++)
  {
    PendingAck &slot = ackMap[(idx + probe) & (ACK_MAP_SIZE - 1)];
    if (!slot.active)
      break;  // empty slot -- key not present
    if (!slot.resolved && slot.key == k)
    {
      slot.accepted = accepted;
      slot.resolved = true;
      return;
    }
  }
}

// O(1) avg: checks only whether an image.start or image.complete ACK for the
// given uploadId is still active. Unrelated ACKs (e.g. temperature) do not block
// new livestream frames.
static bool ackImageActive(const String &uploadId)
{
  const char *imageKinds[] = { "image.start", "image.complete" };
  for (const char *kind : imageKinds)
  {
    String k = ackMakeKey(kind, uploadId);
    uint32_t idx = ackHash(k) & (ACK_MAP_SIZE - 1);
    for (int probe = 0; probe < ACK_MAP_SIZE; probe++)
    {
      PendingAck &slot = ackMap[(idx + probe) & (ACK_MAP_SIZE - 1)];
      if (!slot.active)
        break;
      if (!slot.resolved && slot.key == k)
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
    return false;  // duplicate was suppressed; treat as rejected
  unsigned long t0 = millis();
  while (!slot->resolved)
  {
    webSocket.loop();
    esp_task_wdt_reset();
    yield();
    if (millis() - t0 > ACK_TIMEOUT_MS)
    {

      slot->active = false;
      return false;
    }

    taskYIELD();
  }
  bool accepted = slot->accepted;
  slot->active = false;
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
  doc["kind"] = "ability";
  doc["uuid"] = DEVICE_UUID;
  JsonObject get = doc["abilities"]["get"].to<JsonObject>();
  get["temp"] = "Read latest temperature and humidity";
  get["picture"] = "Capture single JPEG frame and return metadata";
  get["picture_bytes"] = "Capture JPEG frame and return base64 bytes";
  JsonObject set = doc["abilities"]["set"].to<JsonObject>();
  set["class_light"] = "Set classroom usage RGB LED status";
  sendPayload(doc);

}

// ---------------------------------------------------------------------------
// Protocol -- temperature telemetry
// ---------------------------------------------------------------------------
static void sendTemperature()
{
  float t = readTemperature();
  float h = readHumidity();
  if (t < 0 || h < 0)
    return;
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

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {

    return;
  }

  imageInFlight = true;
  String uploadId = makeUploadId();
  imageInFlightUploadId = uploadId;
  size_t totalBytes = fb->len;



  PendingAck *ackStart = ackAlloc("image.start", uploadId);
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
    if (!sendPayload(doc)) {
      if (ackStart) ackStart->active = false;
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
    while (offset < totalBytes)
    {
      size_t chunkLen = min((size_t)IMAGE_CHUNK_SIZE, totalBytes - offset);
      
      if (webSocket.sendBIN(fb->buf + offset, chunkLen)) {
        offset += chunkLen;
      } else {
        delay(5);
      }

      webSocket.loop();
      esp_task_wdt_reset();
      yield();
    }
  }

  // ---- 3. image.complete -------------------------------------------------
  ackComplete = ackAlloc("image.complete", uploadId);
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
      
    if (!sendPayload(doc)) {
      if (ackComplete) ackComplete->active = false;
      goto cleanup;
    }
  }

  waitAck(ackComplete);

cleanup:
  esp_camera_fb_return(fb);
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
    wsConnected = false;
    handshakeDone = false;
    for (int i = 0; i < ACK_MAP_SIZE; i++)
    {
      if (ackMap[i].active)
      {
        ackMap[i].accepted = false;
        ackMap[i].resolved = true;
      }
    }

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
      break;  // ACK messages need no further dispatch
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
      
      // Parse class light state from the message
      const char *valueStr = msg["value"] | "";
      if (valueStr && valueStr[0] != '\0')
        pendingAbilityClassLightState = parseClassLightValue(String(valueStr));
      else
        pendingAbilityClassLightState = CLASS_LED_OFF;
      

    }
    else
    {

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
static void connectWifi()
{

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);

  }

}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
  Serial.end();
  
  // Initialize hardware modules
  ledInit();
  temperatureInit();

  if (!hasConfigValue(WIFI_SSID) || !hasConfigValue(WIFI_PASSWORD) ||
      !hasConfigValue(WS_HOST) || !hasConfigValue(DEVICE_UUID))
  {

    while (true)
      delay(1000);
  }

  connectWifi();

  if (!cameraInit())
  {

    while (true)
      delay(1000);
  }

  // ---- Configure links2004/WebSockets ------------------------------------
#if WS_SSL_ENABLED
  webSocket.beginSSL(WS_HOST, WS_PORT, WS_PATH);
#else
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
#endif
  webSocket.onEvent(webSocketEvent);

  String hdrs = "X-Auth-Method: deviceUuid\r\nX-Device-UUID: ";
  hdrs += DEVICE_UUID;
  webSocket.setExtraHeaders(hdrs.c_str());

  webSocket.setReconnectInterval(RECONNECT_DELAY_MS);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    wsConnected = false;
    connectWifi();

#if WS_SSL_ENABLED
    webSocket.beginSSL(WS_HOST, WS_PORT, WS_PATH);
#else
    webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
#endif
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
      sendPayload(resp);
    }
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
    if (!ackImageActive(imageInFlightUploadId)) { sendChunkedImage(); }
#endif
  }
  else if (now - lastImageMs >= (unsigned long)countIntervalSec * 1000UL)
  {
    lastImageMs = now;
    sendChunkedImage();
  }
}
