#include "temperature.h"
#include <DHTesp.h>

// ---------------------------------------------------------------------------
// DHT Sensor Configuration
// ---------------------------------------------------------------------------
#define DHT_PIN 13

static DHTesp dht;

// ---------------------------------------------------------------------------
// Public Functions
// ---------------------------------------------------------------------------

void temperatureInit()
{
  dht.setup(DHT_PIN, DHTesp::DHT22);
}

float readTemperature()
{
  float v = dht.getTemperature();
  if (isnan(v))
    return -1.f;
  return v;
}

float readHumidity()
{
  float v = dht.getHumidity();
  if (isnan(v))
    return -1.f;
  return v;
}