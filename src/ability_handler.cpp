#include "ability_handler.h"
#include "temperature.h"
#include "led_control.h"
#include "camera.h"

// ---------------------------------------------------------------------------
// Public Functions
// ---------------------------------------------------------------------------

void normalizeAbilityCommand(String &method, String &ability)
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

void generateAbilityDeclaration(JsonDocument &doc)
{
  doc["kind"] = "ability";
  doc["uuid"] = DEVICE_UUID;
  JsonObject get = doc["abilities"]["get"].to<JsonObject>();
  get["temp"] = "Read latest temperature and humidity";
  get["picture"] = "Capture single JPEG frame and return metadata";
  get["picture_bytes"] = "Capture JPEG frame and return base64 bytes";
  JsonObject set = doc["abilities"]["set"].to<JsonObject>();
  set["class_light"] = "Set classroom usage RGB LED status";
}

bool handleAbilityRequest(const String &method, const String &ability, 
                         const String &requestId, ClassLedState classLightState,
                         JsonDocument &response)
{
  response["kind"] = "ability.response";
  response["uuid"] = DEVICE_UUID;
  response["method"] = method;
  response["ability"] = ability;  // correct for GET; overridden later for SET
  response["requestId"] = requestId;

  auto reject = [&](const char *error)
  {
    response["accepted"] = false;
    response["error"] = error;
  };

  if (method.isEmpty() || ability.isEmpty() || requestId.isEmpty())
  {
    reject("payload-invalid");
    return false;
  }

  String normalizedAbility;
  ClassLedState abilityState = parseClassLightAbility(ability, normalizedAbility);
  
  // Merge ability value with external value (external takes priority if not OFF)
  if (classLightState != CLASS_LED_OFF)
    abilityState = classLightState;

  // ---- SET abilities -----------------------------------------------------
  if (method == "set")
  {
    if (normalizedAbility != "class_light")
    {
      reject("unsupported-ability");
      return false;
    }

    setClassUsageLed(abilityState);
    response["ability"] = "class_light"; // normalize ability name for SET
    response["accepted"] = true;
    
    // Return the LED status as string
    const char* ledStatus;
    switch (abilityState)
    {
      case CLASS_LED_USED:
        ledStatus = "used";
        break;
      case CLASS_LED_UNUSED:
        ledStatus = "unused";
        break;
      case CLASS_LED_OFF:
      default:
        ledStatus = "off";
        break;
    }
    response["data"]["status"] = ledStatus;
    return true;
  }

  // ---- GET abilities -----------------------------------------------------
  if (method != "get")
  {
    reject("unsupported-ability");
    return false;
  }

  // ---- get/temp ----------------------------------------------------------
  if (ability == "temp")
  {
    float t = readTemperature(), h = readHumidity();
    if (t < 0)
    {
      reject("sensor-error");
      return false;
    }
    response["accepted"] = true;
    response["data"]["temperature"] = t;
    response["data"]["humidity"] = h;
    response["data"]["mode"] = modeStr(deviceMode);
    return true;
  }

  // ---- get/picture -------------------------------------------------------
  if (ability == "picture")
  {
    camera_fb_t *fb = cameraCaptureFrame();
    if (!fb)
    {
      reject("capture-failed");
      return false;
    }
    int byteLen = (int)fb->len;
    long long capAt = (long long)millis();
    cameraReleaseFrame(fb);

    response["accepted"] = true;
    response["data"]["mimeType"] = "image/jpeg";
    response["data"]["bytes"] = byteLen;
    response["data"]["capturedAt"] = capAt;
    return true;
  }

  // ---- get/picture_bytes -------------------------------------------------
  if (ability == "picture_bytes")
  {
    camera_fb_t *fb = cameraCaptureFrame();
    if (!fb)
    {
      reject("capture-failed");
      return false;
    }

    size_t b64Len;
    char *b64Buf = cameraEncodeBase64(fb, &b64Len);
    cameraReleaseFrame(fb);
    
    if (!b64Buf)
    {
      reject("out-of-memory");
      return false;
    }

    response["accepted"] = true;
    response["data"]["mimeType"] = "image/jpeg";
    response["data"]["encoding"] = "base64";
    response["data"]["base64"] = b64Buf;
    
    free(b64Buf);
    return true;
  }

  reject("unsupported-ability");
  return false;
}
