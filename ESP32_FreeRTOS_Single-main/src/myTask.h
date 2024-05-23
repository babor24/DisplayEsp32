#ifndef MYTASKS_H
#define MYTASKS_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


extern TaskHandle_t HandleRNBLETask;
extern TaskHandle_t SpeedBLETask;
extern TaskHandle_t HandleTemperatureTask;
extern TaskHandle_t HandleRPMTask;

extern SemaphoreHandle_t rpmSemaphore;


void HandleRNBLE(void *pvParameters);
void SpeedBLE(void *pvParameters);
void LimiterHandle(void *pvParameters);
void HandleTemperature(void *pvParameters);
void SelectLimiter(void *pvParameters);
void CalculateRPM(void *pvParameters);
void IRAM_ATTR EXT_RPM ();






#endif
