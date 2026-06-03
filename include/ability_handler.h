#ifndef ABILITY_HANDLER_H
#define ABILITY_HANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Ability Handler Module - Device Capability Management
// ---------------------------------------------------------------------------

// Normalize ability command by splitting method and ability parameters
void normalizeAbilityCommand(String &method, String &ability);

// Generate ability declaration document
void generateAbilityDeclaration(JsonDocument &doc);

// Handle ability request and generate response
// Returns true if request was handled, false if rejected
bool handleAbilityRequest(const String &method, const String &ability, 
                         const String &requestId, ClassLedState classLightState,
                         JsonDocument &response);

#endif // ABILITY_HANDLER_H
