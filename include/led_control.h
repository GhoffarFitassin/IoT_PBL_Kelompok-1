#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------------
// LED Control Module - RGB Status LEDs
// ---------------------------------------------------------------------------

// Initialize both temperature and class LEDs
void ledInit();

// Set temperature LED based on temperature value
// Blue: < 20°C, Yellow: 20-25°C, Red: > 25°C
void setTemperatureLed(float temperature);

// Set class usage LED state
void setClassUsageLed(ClassLedState state);

// Parse class light value from string ("used", "unused", "off")
ClassLedState parseClassLightValue(const String &rawText);

// Parse class light ability command with optional value
// Returns the state and normalizes the ability name
ClassLedState parseClassLightAbility(const String &rawAbility, String &normalizedAbility);

#endif // LED_CONTROL_H
