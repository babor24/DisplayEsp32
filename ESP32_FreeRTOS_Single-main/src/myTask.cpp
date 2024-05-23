#include <myTask.h>
#include <HandleBLE.h>
#include <HandleSpeed.h>
#include <SensorInput.h>

void HandleRNBLE(void *pvParameters)
{
  for (;;)
  {

    TickType_t uptime = xTaskGetTickCount();

    // Convert ticks to milliseconds
    uint32_t uptimeMs = uptime * portTICK_PERIOD_MS;

    SendRunningTime(uptimeMs);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void SpeedBLE(void *pvParameters)
{
  for (;;)
  {

    SendSpeed(speeds, selectedLimiter);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void HandleTemperature(void *pvParameters)
{
  for (;;)
  {
    int Vo = analogRead(TEMP_SENSOR);
    float R2 = 2000 * (4095.0 / (float)Vo - 1.0);
    float logR2 = log(R2);
    float T = (1.0 / (1.933975222e-03 + 1.207278824e-04 * logR2 + 10.76852783e-07 * logR2 * logR2 * logR2));
    temperatureValue = (T - 273.15);

    // Serial.println(String());

    SendTemperature();

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

