#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Temperature Module - DHT22 Sensor
// ---------------------------------------------------------------------------

// Initialize the temperature sensor
void temperatureInit();

// Read current temperature in Celsius (returns -1 on error)
float readTemperature();

// Read current humidity in percentage (returns -1 on error)
float readHumidity();

#endif // TEMPERATURE_H
