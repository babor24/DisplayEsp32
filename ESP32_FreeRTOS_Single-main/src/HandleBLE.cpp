#include <HandleBLE.h>

BLEServer *pServer = NULL;
BLECharacteristic *runningtimeCharacteristic = NULL;
BLECharacteristic *speedCharacteristic = NULL;
BLECharacteristic *rpmCharacteristic = NULL;
BLECharacteristic *limitersatuCharacteristic = NULL;
BLECharacteristic *vehicleinfoCharacteristic = NULL;

String SpeedData = "0#0#0";
String infoData = "0#0#0";

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected++;
        pServer->startAdvertising();
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = deviceConnected - 1;
        Serial.println("Device: " + deviceConnected);
    }
};

class LimiterSatuCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();

        uint8_t Index1, Index2, Index3, Index4;

        if (rxValue.length() > 0)
        {
            String limiterValue = &rxValue[0];

            Index1 = limiterValue.indexOf('#');
            Index2 = limiterValue.indexOf('#', Index1 + 1);
            Index3 = limiterValue.indexOf('#', Index2 + 1);
            Index4 = limiterValue.indexOf('#', Index3 + 1);

            limiterSatu = (limiterValue.substring(Index1 + 1, Index2)).toInt();
            limiterDua = (limiterValue.substring(Index2 + 1, Index3)).toInt();
            limiterTiga = (limiterValue.substring(Index3 + 1, Index4)).toInt();

            Serial.println("=====================");
            Serial.println("Recieved Value: " + limiterValue);
            Serial.println("=====================");
            limiterValue = "";
        }
    }
};

void initBLE()
{

    BLEDevice::init("SapuanginESP32");

    // Set ESP32 as a BLE server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create a service for the BLE server with the univerally unique identifier
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // characteristics that will hold the sensor reading values
    runningtimeCharacteristic = pService->createCharacteristic(
        RUNNING_TIME_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    rpmCharacteristic = pService->createCharacteristic(
        RPM_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    speedCharacteristic = pService->createCharacteristic(
        SPEED_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    vehicleinfoCharacteristic = pService->createCharacteristic(
        VEHICLEINFO_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    limitersatuCharacteristic = pService->createCharacteristic(
        LIMITER_SATU_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE);

    runningtimeCharacteristic->addDescriptor(new BLE2902());
    speedCharacteristic->addDescriptor(new BLE2902());
    rpmCharacteristic->addDescriptor(new BLE2902());
    vehicleinfoCharacteristic->addDescriptor(new BLE2902());

    limitersatuCharacteristic->setCallbacks(new LimiterSatuCallbacks());

    pService->start();
}

void startAdvertising()
{
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);

    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void BLESetup()
{
    initBLE();

    startAdvertising();

    limitersatuCharacteristic->setCallbacks(new LimiterSatuCallbacks());
}

void SendSpeed(float speeds, uint8_t selectedLimiter)
{
    Serial2.println("hello");
    if (deviceConnected > 0)
    {
        SpeedData = String(speeds) + "#" + String(20) + "#" + String(selectedLimiter);

        speedCharacteristic->setValue(SpeedData.c_str());
        speedCharacteristic->notify();
    }
}

void SendRunningTime(uint32_t uptimeMs)
{
    if (deviceConnected > 0)
    {
        String rnTm = String(uptimeMs);
        runningtimeCharacteristic->setValue(rnTm.c_str());
        runningtimeCharacteristic->notify();
    }
}

void HandleDevices()
{
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);
        pServer->startAdvertising();
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected)
    {
        oldDeviceConnected = deviceConnected;
    }
}

void SendTemperature()
{

    if (deviceConnected > 0)
    {
        infoData = "0#0#" + String(temperatureValue);

        vehicleinfoCharacteristic->setValue(infoData.c_str());
        vehicleinfoCharacteristic->notify();
    }
}

void SendRPM(uint16_t rpm)
{

    if (deviceConnected > 0)
    {

        rpmCharacteristic->setValue(String(rpm).c_str());
        rpmCharacteristic->notify();
    }
}