#include "led_control.h"

// ---------------------------------------------------------------------------
// RGB LED Pin Definitions
// ---------------------------------------------------------------------------
#define TEMP_LED_R_PIN 4
#define TEMP_LED_G_PIN 12
#define TEMP_LED_B_PIN 14
#define CLASS_LED_R_PIN 15
#define CLASS_LED_G_PIN 1
#define CLASS_LED_B_PIN 3
#define RGB_LED_ACTIVE_HIGH 1

// ---------------------------------------------------------------------------
// Internal Helper Functions
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

// ---------------------------------------------------------------------------
// Public Functions
// ---------------------------------------------------------------------------

void ledInit()
{
  initRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN);
  initRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN);
}

void setTemperatureLed(float temperature)
{
  if (temperature < 20.0f)
  {
    // Blue for cold
    writeRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN, false, false, true);
    return;
  }
  if (temperature <= 25.0f)
  {
    // Yellow for comfortable
    writeRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN, true, true, false);
    return;
  }
  // Red for hot
  writeRgbLed(TEMP_LED_R_PIN, TEMP_LED_G_PIN, TEMP_LED_B_PIN, true, false, false);
}

void setClassUsageLed(ClassLedState state)
{
  switch (state)
  {
    case CLASS_LED_USED:
      // Green when class is in use
      writeRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN, false, true, false);
      break;
    
    case CLASS_LED_UNUSED:
      // White when class is not in use
      writeRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN, true, true, true);
      break;
    
    case CLASS_LED_OFF:
    default:
      // Off - all LEDs off
      writeRgbLed(CLASS_LED_R_PIN, CLASS_LED_G_PIN, CLASS_LED_B_PIN, false, false, false);
      break;
  }
}

ClassLedState parseClassLightValue(const String &rawText)
{
  String text = rawText;
  text.trim();
  text.toLowerCase();
  
  if (text == "used")
    return CLASS_LED_USED;
  if (text == "unused")
    return CLASS_LED_UNUSED;
  if (text == "off")
    return CLASS_LED_OFF;
  
  return CLASS_LED_OFF; // default to off for invalid values
}

ClassLedState parseClassLightAbility(const String &rawAbility, String &normalizedAbility)
{
  const String expectedAbility = "class_light";
  String abilityText = rawAbility;
  abilityText.trim();
  normalizedAbility = "class_light";

  // Just "class_light" without value -> default to off
  if (abilityText == expectedAbility)
    return CLASS_LED_OFF;

  if (!abilityText.startsWith(expectedAbility))
    return CLASS_LED_OFF;

  int prefixLen = expectedAbility.length();
  char separator = abilityText.charAt(prefixLen);
  if (separator != ' ' && separator != ':' && separator != '=' &&
      separator != '/' && separator != ',')
    return CLASS_LED_OFF;

  String valueText = abilityText.substring(prefixLen + 1);
  return parseClassLightValue(valueText);
}
