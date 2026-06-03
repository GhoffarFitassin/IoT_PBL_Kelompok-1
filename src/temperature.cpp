#include "temperature.h"
#include <DHT.h>

// ---------------------------------------------------------------------------
// DHT Sensor Configuration
// ---------------------------------------------------------------------------
#define DHT_PIN 13
#define DHT_TYPE DHT22

static DHT dht(DHT_PIN, DHT_TYPE);

// ---------------------------------------------------------------------------
// Public Functions
// ---------------------------------------------------------------------------

void temperatureInit()
{
  dht.begin();
}

float readTemperature()
{
  float v = dht.readTemperature();
  if (isnan(v))
    return -1.f;
  return v;
}

float readHumidity()
{
  float v = dht.readHumidity();
  if (isnan(v))
    return -1.f;
  return v;
}
