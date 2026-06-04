#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Temperature Module - DHT22 Sensor
// ---------------------------------------------------------------------------

// Initialize the temperature sensor
void temperatureInit();

// (Legacy) Read current temperature in Celsius (returns -1 on error)
float readTemperature();

// (Legacy) Read current humidity in percentage (returns -1 on error)
float readHumidity();

// Combined read: performs ONE DHT sensor read, fills temp & hum.
// Returns true on success, false on sensor failure.
// Does NOT block indefinitely — timeout + WDT reset built in.
bool readTemperatureAndHumidity(float &temp, float &hum);

#endif // TEMPERATURE_H