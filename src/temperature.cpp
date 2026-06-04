#include "temperature.h"
#include "esp_task_wdt.h"
#include <DHTesp.h>

// ---------------------------------------------------------------------------
// DHT Sensor Configuration
// ---------------------------------------------------------------------------
#define DHT_PIN 13

// Maximum time (ms) we wait for a DHT read before giving up
#define TEMP_READ_TIMEOUT_MS 250

// Stack memory for the DHT read helper task (bytes)
#define TEMP_TASK_STACK (2 * 1024)

static DHTesp dht;

// Failure tracking
static unsigned long lastFailureMs = 0;

// ---------------------------------------------------------------------------
// FreeRTOS task data — passed by pointer to the pinned task
// ---------------------------------------------------------------------------
typedef struct {
  float temperature;  // output
  float humidity;     // output
  bool  done;         // set true by task when finished (success or fail)
  bool  ok;           // set true by task on valid reading
} TempTaskData;

// ---------------------------------------------------------------------------
// FreeRTOS task: poll DHT sensor and store result, then self-delete.
// Pinned to core 0 (protocol/pro_cpu) so it never competes with the Arduino
// loop() which runs on core 1 (app_cpu).
// ---------------------------------------------------------------------------
static void tempTaskFunc(void *pvParams)
{
  TempTaskData *d = (TempTaskData *)pvParams;

  unsigned long t0 = millis();
  while (millis() - t0 < TEMP_READ_TIMEOUT_MS)
  {
    // getTempAndHumidity() returns immediately if a fresh reading is cached,
    // otherwise blocks for one sensor cycle (~2 ms for DHT22).
    TempAndHumidity th = dht.getTempAndHumidity();

    if (!isnan(th.temperature) && !isnan(th.humidity))
    {
      d->temperature = th.temperature;
      d->humidity    = th.humidity;
      d->ok   = true;
      d->done = true;
      vTaskDelete(NULL);
      return;
    }

    // Reading not ready yet — yield briefly and retry
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  // Timed out without a valid reading
  d->ok   = false;
  d->done = true;
  vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Public Functions
// ---------------------------------------------------------------------------

void temperatureInit()
{
  dht.setup(DHT_PIN, DHTesp::DHT22);
}

float readTemperature()
{
  // Use the combined function under the hood for consistency
  float t, h;
  if (readTemperatureAndHumidity(t, h))
    return t;
  return -1.f;
}

float readHumidity()
{
  float t, h;
  if (readTemperatureAndHumidity(t, h))
    return h;
  return -1.f;
}

bool readTemperatureAndHumidity(float &temp, float &hum)
{
  temp = -1.f;
  hum  = -1.f;

  // Feed WDT before spawning the task
  esp_task_wdt_reset();

  TempTaskData data = {};

  TaskHandle_t taskHandle = NULL;
  xTaskCreatePinnedToCore(
    tempTaskFunc,       // task function
    "dht_read",         // debug name
    TEMP_TASK_STACK,    // stack in bytes
    &data,              // parameter
    4,                  // priority — low enough not to starve idle/wifi tasks
    &taskHandle,
    0                   // core 0 (pro_cpu / protocol core)
  );

  // Wait for the task to finish, feeding WDT while we wait.
  // Add a small guard margin on top of TEMP_READ_TIMEOUT_MS so we never
  // kill a task that is about to succeed.
  const unsigned long guardMs = TEMP_READ_TIMEOUT_MS + 50UL;
  unsigned long t0 = millis();
  while (!data.done)
  {
    if (millis() - t0 >= guardMs)
    {
      vTaskDelete(taskHandle);
      lastFailureMs = millis();
      return false;
    }
    esp_task_wdt_reset();
    taskYIELD();
    yield();
  }

  if (!data.ok)
  {
    lastFailureMs = millis();
    return false;
  }

  temp = data.temperature;
  hum  = data.humidity;
  return true;
}