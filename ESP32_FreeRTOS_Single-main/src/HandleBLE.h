#ifndef HANDLEBLE_H
#define HANDLEBLE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <nRF24L01.h>
#include <RF24.h>
// #include <HandleBLE.h>

// DEFINE CHARACTERISTIC OF PARAMETER
#define SERVICE_UUID "c23b7ab5-0301-441a-ac60-1757084297d4"
#define RUNNING_TIME_CHARACTERISTIC_UUID "94bedc82-1bc3-44a8-88bd-17318eb59a44"
#define SPEED_CHARACTERISTIC_UUID "e7ca3a76-9026-4f56-9b35-09da4c3c5eea"
#define LIMITER_SATU_CHARACTERISTIC_UUID "8c6fe5b0-0931-41f7-bab5-6b08cb20f524"
#define RPM_CHARACTERISTIC_UUID "33062e3a-920d-439a-9d05-9e1a37ebb91d"
#define VEHICLEINFO_CHARACTERISTIC_UUID "35948d8e-2baf-44f7-a239-2b1d974a263b"
//"d0d62c90-894e-451c-8e67-c172ee741259"

extern String limiterValue;
extern uint8_t deviceConnected;
extern uint8_t oldDeviceConnected;
extern uint8_t limiterSatu;
extern uint8_t limiterDua;
extern uint8_t limiterTiga;
extern float temperatureValue;


void BLESetup();
void HandleDevices();
void SendRunningTime(uint32_t uptimeMs);
void SendSpeed(float speeds, uint8_t selectedLimiter);
void SendTemperature();
void SendRPM(uint16_t rpm);

#endif
