#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include "esp_camera.h"
#include "esp_task_wdt.h" 

#ifndef ENV_WIFI_SSID
#define ENV_WIFI_SSID ""
#endif

#ifndef ENV_WIFI_PASSWORD
#define ENV_WIFI_PASSWORD ""
#endif

#ifndef ENV_WS_HOST
#define ENV_WS_HOST ""
#endif

#ifndef ENV_DEVICE_UUID
#define ENV_DEVICE_UUID ""
#endif

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static const char *WIFI_SSID = ENV_WIFI_SSID;
static const char *WIFI_PASSWORD = ENV_WIFI_PASSWORD;

// ---------------------------------------------------------------------------
// Server & Device identity
// ---------------------------------------------------------------------------
static const char *WS_HOST = ENV_WS_HOST;
static const uint16_t WS_PORT = 3010;
static const char *WS_PATH = "/ws";
static const char *DEVICE_UUID = ENV_DEVICE_UUID;

// ---------------------------------------------------------------------------
// [TODO-2] DHT sensor
// ---------------------------------------------------------------------------
#define DHT_PIN 13     
#define DHT_TYPE DHT22 
DHT dht(DHT_PIN, DHT_TYPE);

// ---------------------------------------------------------------------------
// [TODO-3] Camera pin configuration -- AI-Thinker ESP32-CAM
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
// [TODO-4] Image / PSRAM settings
// ---------------------------------------------------------------------------
#define JPEG_QUALITY 12           
#define FRAME_SIZE FRAMESIZE_SVGA

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------
#define IMAGE_CHUNK_SIZE ( * 1024)
#define TEMP_INTERVAL_DEFAULT 60
#define COUNT_INTERVAL_DEFAULT 60
#define ACK_TIMEOUT_MS 30000
#define RECONNECT_DELAY_MS 5000
#define LIVE_STREAM_MIN_GAP_MS 200

// ---------------------------------------------------------------------------
// RGB status LEDs
// ---------------------------------------------------------------------------
#define TEMP_LED_R_PIN 4
#define TEMP_LED_G_PIN 12
#define TEMP_LED_B_PIN 14
#define CLASS_LED_R_PIN 15
#define CLASS_LED_G_PIN 1
#define CLASS_LED_B_PIN 3
#define RGB_LED_ACTIVE_HIGH 1

// ---------------------------------------------------------------------------
// Device mode
// ---------------------------------------------------------------------------
enum DeviceMode
{
  MODE_NORMAL,
  MODE_FOCUS,
  MODE_IDLE
};

static DeviceMode parseMode(const char *s, DeviceMode def = MODE_NORMAL)
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
static const char *modeStr(DeviceMode m)
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
bool classUsageKnown = false;
bool classInUse = false;

unsigned long lastTempMs = 0;
unsigned long lastImageMs = 0;
bool imageInFlight = false;

struct PendingAck
{
  bool active = false;
  String expectedKind = "";
  String expectedUploadId = "";
  bool resolved = false;
  bool accepted = false;
};
static PendingAck ack;

// ---------------------------------------------------------------------------
// Deferred action flags
// ---------------------------------------------------------------------------
static bool pendingSendHandshake = false;
static bool pendingSendAbilityDecl = false;
static bool pendingSendCapture = false;

static String pendingAbilityMethod;
static String pendingAbilityName;
static String pendingAbilityRequestId;
static bool pendingAbilityHasClassLightValue = false;
static bool pendingAbilityClassLightInUse = false;

static bool hasConfigValue(const char *value)
{
  return value && value[0] != '\0';
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
static String makeUploadId()
{
  String id = DEVICE_UUID;
  id.replace("-", "");
  id += '_';
  id += String(millis());
  return id;
}

// ---------------------------------------------------------------------------
// RGB LED helpers
// ---------------------------------------------------------------------------
static uint8_t ledLevel(bool on)
{
  return RGB_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
}

static void initRgbLed(uint8_t redPin, uint8_t greenPin, uint8_t bluePin)
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, ledLevel(false));
  digitalWrite(greenPin, ledLevel(false));
  digitalWrite(bluePin, ledLevel(false));
}

static void writeRgbLed(uint8_t redPin, uint8_t greenPin, uint8_t bluePin,
                        bool red, bool green, bool blue)
{
  digitalWrite(redPin, ledLevel(red));
  digitalWrite(greenPin, ledLevel(green));
  digitalWrite(bluePin, ledLevel(blue));
}

static void setTemperatureLed(float temperature)
{
  if (temperature < 20.0f)
  {
    writeRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN, false, false, true);
    return;
  }
  if (temperature <= 25.0f)
  {
    writeRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN, true, true, false);
    return;
  }
  writeRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN, true, false, false);
}

static void setClassUsageLed(bool inUse, bool known = true)
{
  classUsageKnown = known;
  classInUse = inUse;

  if (!known)
  {
    writeRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN, false, false, false);
    return;
  }

  if (inUse)
  {
    writeRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN, false, true, false);
    return;
  }
  writeRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN, true, true, true);
}

static bool parseClassLightText(const String &rawText, bool &out)
{
  String text = rawText;
  text.trim();
  text.toLowerCase();
  if (text == "true" || text == "1" || text == "used" ||
      text == "in-use" || text == "in_use" || text == "occupied" ||
      text == "busy" || text == "yes" || text == "on")
  {
    out = true;
    return true;
  }
  if (text == "false" || text == "0" || text == "not-used" ||
      text == "not_used" || text == "not used" || text == "unused" ||
      text == "available" || text == "idle" || text == "free" ||
      text == "vacant" || text == "empty" || text == "no" || text == "off")
  {
    out = false;
    return true;
  }

  return false;
}

static bool parseClassLightBool(JsonVariantConst value, bool &out)
{
  if (value.isNull())
    return false;

  if (value.is<bool>())
  {
    out = value.as<bool>();
    return true;
  }

  if (value.is<int>())
  {
    int intValue = value.as<int>();
    if (intValue == 0 || intValue == 1)
    {
      out = intValue == 1;
      return true;
    }
  }

  const char *textValue = value.as<const char *>();
  if (!textValue)
    return false;

  return parseClassLightText(String(textValue), out);
}

static bool readClassLightField(JsonObjectConst obj, const char *key, bool &out)
{
  return parseClassLightBool(obj[key], out);
}

static bool readClassLightNested(JsonObjectConst obj, const char *key, bool &out)
{
  if (!obj[key].is<JsonObjectConst>())
    return false;

  JsonObjectConst nested = obj[key].as<JsonObjectConst>();
  return readClassLightField(nested, "classInUse", out) ||
         readClassLightField(nested, "classUsed", out) ||
         readClassLightField(nested, "roomInUse", out) ||
         readClassLightField(nested, "roomUsed", out) ||
         readClassLightField(nested, "isClassUsed", out) ||
         readClassLightField(nested, "isRoomUsed", out) ||
         readClassLightField(nested, "used", out) ||
         readClassLightField(nested, "occupied", out) ||
         readClassLightField(nested, "inUse", out);
}

static bool extractClassLightValue(JsonObjectConst obj, bool &out)
{
  return readClassLightField(obj, "classInUse", out) ||
         readClassLightField(obj, "classUsed", out) ||
         readClassLightField(obj, "roomInUse", out) ||
         readClassLightField(obj, "roomUsed", out) ||
         readClassLightField(obj, "isClassUsed", out) ||
         readClassLightField(obj, "isRoomUsed", out) ||
         readClassLightField(obj, "used", out) ||
         readClassLightField(obj, "occupied", out) ||
         readClassLightField(obj, "inUse", out) ||
         readClassLightField(obj, "data", out) ||
         readClassLightField(obj, "value", out) ||
         readClassLightField(obj, "state", out) ||
         readClassLightNested(obj, "data", out) ||
         readClassLightNested(obj, "classroom", out) ||
         readClassLightNested(obj, "classStatus", out) ||
         readClassLightNested(obj, "roomStatus", out) ||
         readClassLightNested(obj, "status", out);
}

static bool parseClassLightAbility(const String &rawAbility, String &normalizedAbility,
                                   bool &hasValue, bool &out)
{
  const String expectedAbility = "class_light";
  String abilityText = rawAbility;
  abilityText.trim();
  hasValue = false;
  normalizedAbility = abilityText;

  if (abilityText == expectedAbility)
  {
    normalizedAbility = expectedAbility;
    return true;
  }

  if (!abilityText.startsWith(expectedAbility))
    return false;

  int prefixLen = expectedAbility.length();
  char separator = abilityText.charAt(prefixLen);
  if (separator != ' ' && separator != ':' && separator != '=' &&
      separator != '/' && separator != ',')
    return false;

  String valueText = abilityText.substring(prefixLen + 1);
  valueText.trim();
  normalizedAbility = expectedAbility;
  hasValue = parseClassLightText(valueText, out);
  return true;
}

static bool isClassLightAbility(const String &ability)
{
  String normalizedAbility;
  bool hasValue = false;
  bool value = false;
  return parseClassLightAbility(ability, normalizedAbility, hasValue, value);
}

static void normalizeAbilityCommand(String &method, String &ability)
{
  method.trim();
  ability.trim();

  if (!ability.isEmpty())
    return;

  int firstSpace = method.indexOf(' ');
  if (firstSpace <= 0)
    return;

  ability = method.substring(firstSpace + 1);
  method = method.substring(0, firstSpace);
  method.trim();
  ability.trim();
}

// ---------------------------------------------------------------------------
// DHT reads
// ---------------------------------------------------------------------------
static float readTemperature()
{
  float v = dht.readTemperature();
  if (isnan(v))
  {
    Serial.println("[WARN] DHT temp read failed");
    return -1.f;
  }
  return v;
}
static float readHumidity()
{
  float v = dht.readHumidity();
  if (isnan(v))
  {
    Serial.println("[WARN] DHT humidity read failed");
    return -1.f;
  }
  return v;
}

// ---------------------------------------------------------------------------
// Camera init
// ---------------------------------------------------------------------------
static bool cameraInit()
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
  {
    Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
    return false;
  }
  Serial.println("[INFO] Camera initialised.");
  return true;
}

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

static bool waitAck()
{
  unsigned long t0 = millis();
  while (!ack.resolved)
  {
    webSocket.loop();     
    esp_task_wdt_reset();
    if (millis() - t0 > ACK_TIMEOUT_MS)
    {
      Serial.printf("[WARN] ACK timeout (kind=%s)\n", ack.expectedKind.c_str());
      ack.active = false;
      return false;
    }

    taskYIELD();
  }
  ack.active = false;
  return ack.accepted;
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
  Serial.println("[INFO] Handshake sent.");
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
  Serial.println("[INFO] Ability declaration sent.");
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

  if (sendPayload(doc))
    Serial.printf("[INFO] Temp sent: %.2f C  %.2f %%RH\n", t, h);
}

// ---------------------------------------------------------------------------
// Protocol -- chunked image upload
// ---------------------------------------------------------------------------
static void sendChunkedImage(bool isRequested = false)
{
  if (imageInFlight && !isRequested)
  {
    Serial.println("[INFO] Image upload already in flight -- skipping.");
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("[WARN] Camera capture failed.");
    return;
  }

  imageInFlight = true;
  String uploadId = makeUploadId();
  size_t totalBytes = fb->len;

  Serial.printf("[INFO] Image upload start: %u bytes  id=%s\n",
                totalBytes, uploadId.c_str());

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
      goto cleanup;
  }

  ack.active = true;
  ack.expectedKind = "image.start";
  ack.expectedUploadId = uploadId;
  ack.resolved = false;
  ack.accepted = false;
  if (!waitAck())
  {
    Serial.println("[WARN] image.start rejected or timed out.");
    goto cleanup;
  }

  // ---- 2. Binary chunks --------------------------------------------------
  {
    size_t offset = 0;
    while (offset < totalBytes)
    {
      size_t chunkLen = min((size_t)IMAGE_CHUNK_SIZE, totalBytes - offset);

      webSocket.sendBIN(fb->buf + offset, chunkLen);

      ack.active = true;
      ack.expectedKind = "image.chunk.bytes";
      ack.expectedUploadId = uploadId;
      ack.resolved = false;
      ack.accepted = false;
      if (!waitAck())
      {
        Serial.println("[WARN] Chunk ACK failed.");
        goto cleanup;
      }

      offset += chunkLen;
    }
  }

  // ---- 3. image.complete -------------------------------------------------
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
      goto cleanup;
  }

  ack.active = true;
  ack.expectedKind = "image.complete";
  ack.expectedUploadId = uploadId;
  ack.resolved = false;
  ack.accepted = false;
  if (waitAck())
    Serial.printf("[INFO] Image upload complete (%u bytes).\n", totalBytes);
  else
    Serial.println("[WARN] image.complete rejected.");

cleanup:
  esp_camera_fb_return(fb);
  imageInFlight = false;
}

// ---------------------------------------------------------------------------
// Protocol -- ability request handler
// ---------------------------------------------------------------------------
static void handleAbilityRequest(const String &method, const String &ability, const String &requestId,
                                 bool hasClassLightValue, bool classLightInUseValue)
{
  JsonDocument resp;
  resp["kind"] = "ability.response";
  resp["uuid"] = DEVICE_UUID;
  resp["method"] = method;
  resp["ability"] = ability;
  resp["requestId"] = requestId;

  auto reject = [&](const char *error)
  {
    resp["accepted"] = false;
    resp["error"] = error;
    sendPayload(resp);
    Serial.printf("[INFO] Ability rejected %s/%s  error=%s\n",
                  method.c_str(), ability.c_str(), error);
  };

  if (method.isEmpty() || ability.isEmpty() || requestId.isEmpty())
  {
    reject("payload-invalid");
    return;
  }

  String normalizedAbility = ability;
  bool abilityHasClassLightValue = false;
  bool abilityClassLightInUse = false;
  bool classLightAbility = parseClassLightAbility(ability, normalizedAbility,
                                                  abilityHasClassLightValue,
                                                  abilityClassLightInUse);
  resp["ability"] = normalizedAbility;

  if (method == "set")
  {
    if (!classLightAbility)
    {
      reject("unsupported-ability");
      return;
    }

    if (abilityHasClassLightValue)
    {
      hasClassLightValue = true;
      classLightInUseValue = abilityClassLightInUse;
    }

    if (!hasClassLightValue)
    {
      setClassUsageLed(false, false);
      reject("payload-invalid");
      return;
    }

    setClassUsageLed(classLightInUseValue);
    resp["accepted"] = true;
    resp["data"]["classInUse"] = classInUse;
    resp["data"]["known"] = classUsageKnown;
    sendPayload(resp);
    Serial.printf("[INFO] Ability set/%s -> %s\n",
                  normalizedAbility.c_str(),
                  classInUse ? "used" : "not-used");
    return;
  }

  if (method != "get")
  {
    reject("unsupported-ability");
    return;
  }

  // ---- get/temp ----------------------------------------------------------
  if (ability == "temp")
  {
    float t = readTemperature(), h = readHumidity();
    if (t < 0)
    {
      reject("sensor-error");
      return;
    }
    resp["accepted"] = true;
    resp["data"]["temperature"] = t;
    resp["data"]["humidity"] = h;
    resp["data"]["mode"] = modeStr(deviceMode);
    sendPayload(resp);
    Serial.printf("[INFO] Ability get/temp -> %.2f C  %.2f %%RH\n", t, h);
    return;
  }

  // ---- get/picture -------------------------------------------------------
  if (ability == "picture")
  {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
      reject("capture-failed");
      return;
    }
    int byteLen = (int)fb->len;
    long long capAt = (long long)millis();
    esp_camera_fb_return(fb); // done with frame buffer before any alloc

    resp["accepted"] = true;
    resp["data"]["mimeType"] = "image/jpeg";
    resp["data"]["bytes"] = byteLen;
    resp["data"]["capturedAt"] = capAt;
    sendPayload(resp);
    Serial.printf("[INFO] Ability get/picture -> %d bytes metadata sent.\n", byteLen);
    return;
  }

  // ---- get/picture_bytes -------------------------------------------------
  if (ability == "picture_bytes")
  {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
      reject("capture-failed");
      return;
    }

    // 1. Base64-encode the raw JPEG bytes
    static const char b64chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t inLen = fb->len;
    size_t b64Len = ((inLen + 2) / 3) * 4 + 1;
    char *b64Buf = (char *)malloc(b64Len);
    if (!b64Buf)
    {
      esp_camera_fb_return(fb);
      reject("out-of-memory");
      return;
    }
    {
      uint8_t *in = fb->buf;
      char *out = b64Buf;
      for (size_t i = 0; i < inLen; i += 3)
      {
        uint32_t n = ((uint32_t)in[i] << 16) | (i + 1 < inLen ? (uint32_t)in[i + 1] << 8 : 0) | (i + 2 < inLen ? (uint32_t)in[i + 2] : 0);
        *out++ = b64chars[(n >> 18) & 63];
        *out++ = b64chars[(n >> 12) & 63];
        *out++ = (i + 1 < inLen) ? b64chars[(n >> 6) & 63] : '=';
        *out++ = (i + 2 < inLen) ? b64chars[n & 63] : '=';
      }
      *out = '\0';
    }
    esp_camera_fb_return(fb); // release camera buffer NOW before building tx string
    fb = nullptr;

    // 2. Build the full JSON
    String txStr;
    txStr.reserve(b64Len + 300); // pre-allocate to avoid realloc during build
    txStr = "{\"type\":\"push\",\"channel\":\"device.client\",\"payload\":{"
            "\"kind\":\"ability.response\","
            "\"uuid\":\"";
    txStr += DEVICE_UUID;
    txStr += "\",\"method\":\"";
    txStr += method;
    txStr += "\",\"ability\":\"";
    txStr += ability;
    txStr += "\",\"requestId\":\"";
    txStr += requestId;
    txStr += "\",\"accepted\":true,"
             "\"data\":{"
             "\"mimeType\":\"image/jpeg\","
             "\"encoding\":\"base64\","
             "\"base64\":\"";
    txStr += b64Buf;
    txStr += "\"}}}";
    free(b64Buf);

    if (!wsConnected)
    {
      Serial.println("[WARN] picture_bytes: WS disconnected before send.");
      return;
    }
    webSocket.sendTXT(txStr);
    Serial.printf("[INFO] Ability get/picture_bytes -> %u B b64 sent.\n", (unsigned)b64Len);
    return;
  }

  reject("unsupported-ability");
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
    Serial.println("[INFO] WebSocket connected -- handshake deferred to loop().");
    break;

  case WStype_DISCONNECTED:
    wsConnected = false;
    handshakeDone = false;
    if (ack.active)
    {
      ack.accepted = false;
      ack.resolved = true;
    }
    Serial.println("[INFO] WebSocket disconnected -- library will reconnect.");
    break;

  // ---- text frame --------------------------------------------------------
  case WStype_TEXT:
  {
    Serial.printf("[RAW] %.*s\n", (int)min(length, (size_t)300), (char *)payload);

    JsonDocument data;
    DeserializationError derr = deserializeJson(data, payload, length);
    if (derr != DeserializationError::Ok)
    {
      Serial.printf("[WARN] JSON parse error: %s\n", derr.c_str());
      break;
    }

    const char *msgType = data["type"] | "";

    if (strcmp(msgType, "ready") == 0)
    {
      Serial.println("[INFO] Server ready.");
      break;
    }
    if (strcmp(msgType, "error") == 0)
    {
      Serial.printf("[ERROR] Server: %s\n",
                    (const char *)(data["message"] | "unknown"));
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
      Serial.println("[WARN] No data/payload object in message.");
      break;
    }
    const char *kind = msg["kind"] | "";
    Serial.printf("[DBG] kind=%s roomId=%s\n", kind, (const char *)(msg["roomId"] | "(null)"));

    // ---- Resolve pending ACK -------------------------------------------
    if (ack.active)
    {
      bool kMatch = (ack.expectedKind == kind);
      bool iMatch = ack.expectedUploadId.isEmpty() || ack.expectedUploadId == (const char *)(msg["uploadId"] | "");
      if (kMatch && iMatch)
      {
        ack.accepted = msg["accepted"] | false;
        ack.resolved = true;
        break;
      }
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
      Serial.printf("[INFO] Handshake OK | Room: %s | TempInterval: %ds\n",
                    (const char *)(msg["roomId"] | "none"), tempIntervalSec);
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
      Serial.printf("[INFO] Config updated | Room: %s | Mode: %s | Live: %s\n",
                    previewRoomId.c_str(), modeStr(deviceMode),
                    liveStreamEnabled ? "yes" : "no");
    }
    else if (strcmp(kind, "capture.request") == 0)
    {
      pendingSendCapture = true;
      Serial.println("[INFO] Capture request received -- deferred to loop().");
    }
    else if (strcmp(kind, "ability.request") == 0)
    {
      pendingAbilityMethod = msg["method"] | "";
      pendingAbilityName = msg["ability"] | "";
      if (pendingAbilityMethod.isEmpty())
        pendingAbilityMethod = msg["command"] | "";
      normalizeAbilityCommand(pendingAbilityMethod, pendingAbilityName);
      pendingAbilityRequestId = msg["requestId"] | "";
      pendingAbilityClassLightInUse = false;
      pendingAbilityHasClassLightValue = extractClassLightValue(msg, pendingAbilityClassLightInUse);
      Serial.printf("[INFO] Ability request queued: %s/%s\n",
                    pendingAbilityMethod.c_str(), pendingAbilityName.c_str());
    }
    else
    {
      Serial.printf("[DEBUG] Unhandled kind: %s\n", kind);
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
  Serial.printf("[INFO] Connecting to WiFi: %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print('.');
  }
  Serial.printf("\n[INFO] WiFi connected -- IP: %s\n",
                WiFi.localIP().toString().c_str());
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println("\n\n[INFO] ESP32 Device Client booting...");

  if (!hasConfigValue(WIFI_SSID) || !hasConfigValue(WIFI_PASSWORD) ||
      !hasConfigValue(WS_HOST) || !hasConfigValue(DEVICE_UUID))
  {
    Serial.println("[ERROR] Missing required build config. Fill .env then rebuild.");
    while (true)
      delay(1000);
  }

  dht.begin();
  initRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN);
  initRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN);
  setClassUsageLed(false, false);

  connectWifi();

  if (!cameraInit())
  {
    Serial.println("[ERROR] Camera init failed -- halting.");
    while (true)
      delay(1000);
  }

  // ---- Configure links2004/WebSockets ------------------------------------
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
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

    webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
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
    bool hasClassLightValue = pendingAbilityHasClassLightValue;
    bool classLightInUseValue = pendingAbilityClassLightInUse;
    pendingAbilityMethod = "";
    pendingAbilityName = "";
    pendingAbilityRequestId = "";
    pendingAbilityHasClassLightValue = false;
    pendingAbilityClassLightInUse = false;
    handleAbilityRequest(m, a, r, hasClassLightValue, classLightInUseValue);
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
    // Live stream
    static unsigned long lastLiveMs = 0;
    if (now - lastLiveMs >= LIVE_STREAM_MIN_GAP_MS)
    {
      lastLiveMs = now;
      sendChunkedImage();
    }
  }
  else if (now - lastImageMs >= (unsigned long)countIntervalSec * 1000UL)
  {
    lastImageMs = now;
    sendChunkedImage();
  }
}
