#include <Arduino.h>
#include <HandleBLE.h>
#include <myTask.h>
#include <SensorInput.h>
#include <HandleSpeed.h>
#include <nRF24L01.h>
#include <RF24.h>

// Define the UART pins
#define TXD_PIN 17
#define RXD_PIN 16

// Define the UART object
// HardwareSerial Serial2(1);

TaskHandle_t HandleRNBLETask;
TaskHandle_t SpeedBLETask;
TaskHandle_t SpeedInterrupt;
TaskHandle_t LimiterTask;
TaskHandle_t ToggleTask;
TaskHandle_t HandleTemperatureTask;
TaskHandle_t HandleRPMTask;

SemaphoreHandle_t rpmSemaphore;

// BluetoothDevice Variable
uint8_t deviceConnected = 0;
uint8_t oldDeviceConnected = 0;

uint8_t limiterSatu = 0;
uint8_t limiterDua = 0;
uint8_t limiterTiga = 0;
uint8_t selectedLimiter = 0;
uint8_t temp_selectedLim = 0;

// data array for 4 variables
//  uint8_t dataToArray [4] = {speeds,limiterSatu,limiterDua,limiterTiga};

// Define the buffer size for sending uart data
// static const int TX_BUF_SIZE = 2;

// Variable Untuk Speed
unsigned long waktu = 0, prev_waktu = 0;
unsigned long falling_time = 0;
unsigned long engoff_time = 0, prev_engoff_time = 0;
double maxspeed = 0;
bool autospeed = false;
float speeds = 0;

uint16_t rev = 0;
unsigned long oldtimes;
uint16_t OldRPM;

float temperatureValue = 0;

RF24 radio(7, 8); // CE, CSN

void SpeedFalingISR();
void SpeedRisingISR();
void handleMaxSpeed();
void handleMinSpeed();
void maxspeedHand();
void handleLimiter();
void sendUsartArray(uint8_t dataSend, uint8_t kecepatan, uint8_t limiterpertama, uint8_t limiterkedua, uint8_t limiterketiga);

void setup()
{

  Serial.begin(115200);

  // intialize serial port
  Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);

  pinMode(RelayPin, OUTPUT);
  pinMode(LimiterToogle, INPUT_PULLUP);

  rpmSemaphore = xSemaphoreCreateBinary();

  pinMode(RPMsensor, INPUT);
  pinMode(SpeedSensor, INPUT);
  pinMode(TEMP_SENSOR, INPUT);
  pinMode(lim1, INPUT); // 1
  pinMode(lim2, INPUT); // 2

  attachInterrupt(digitalPinToInterrupt(RPMsensor), EXT_RPM, FALLING);
  attachInterrupt(digitalPinToInterrupt(SpeedSensor), SpeedRisingISR, FALLING);
  BLESetup();

  // program transmitter
  //===========> program transmiter nrf24 <=============
  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0E1LL); // Alamat tujuan
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();
  //====================> akhiran <

  // Serial.println("KONTOL");
  // santai woii

  xTaskCreatePinnedToCore(HandleRNBLE, "RNBLE", 4096, NULL, 1, &HandleRNBLETask, 0);
  xTaskCreatePinnedToCore(SpeedBLE, "HandleSpeedBLE", 4096, NULL, 1, &SpeedBLETask, 0);
  xTaskCreatePinnedToCore(SelectLimiter, "ToggleLimiter", 4096, NULL, 1, &ToggleTask, 1);
  xTaskCreatePinnedToCore(HandleTemperature, "ToggleLimiter", 4096, NULL, 1, &HandleRNBLETask, 1);
  xTaskCreatePinnedToCore(LimiterHandle, "HandleLimiter", 4096, NULL, 8, &LimiterTask, 1);
  xTaskCreatePinnedToCore(CalculateRPM, "RPMFunc", 4096, NULL, 2, &HandleRPMTask, 0);
  // vTaskStartScheduler();
  // xSemaphoreGive(rpmSemaphore);

  // usartArray
  sendUsartArray(uint8_t dataArray, static_cast<uint8_t>(speeds), limiterSatu, limiterDua, limiterTiga);
}

void loop()
{
  HandleDevices();
  vTaskDelay(pdMS_TO_TICKS(10));
  sendUsartArray(uint8_t dataArray, static_cast<uint8_t>(speeds), limiterSatu, limiterDua, limiterTiga){
    
  }

  // Serial2.print(speeds,HEX);
  // Serial2.print(dataToArray[],HEX);
  // usart komunikasi
  // sendSpeedSerial(80);
}

void LimiterHandle(void *pvParameters)
{
  for (;;)
  {
    handleMaxSpeed();
    handleMinSpeed();
    maxspeedHand();
    handleLimiter();
    xSemaphoreGive(rpmSemaphore);
    vTaskDelay(pdMS_TO_TICKS(3));

    // program transmitter
    //===========> program transmiter nrf24 <=============
    radio.begin();
    radio.openWritingPipe(0xF0F0F0F0E1LL); // Alamat tujuan
    radio.setPALevel(RF24_PA_HIGH);
    radio.stopListening();
    //====================> akhiran <====================
  }
}

void SelectLimiter(void *pvParameters)
{

  for (;;)
  {
    if (digitalRead(lim1) == HIGH && digitalRead(lim2) == LOW)
    {
      selectedLimiter = 1;
      temp_selectedLim = selectedLimiter;
    }
    else if (digitalRead(lim2) == HIGH && digitalRead(lim1) == LOW)
    {
      selectedLimiter = 3;
      temp_selectedLim = selectedLimiter;
    }

    else if (digitalRead(lim2) == LOW && digitalRead(lim1) == LOW)
    {
      selectedLimiter = 2;
      temp_selectedLim = selectedLimiter;
    }

    else
    {
      selectedLimiter = temp_selectedLim;
    }

    // Serial.println(String(maxspeed));

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void SpeedFalingISR()
{
  attachInterrupt(digitalPinToInterrupt(SpeedSensor), SpeedRisingISR, FALLING);
  falling_time = millis();
}

void SpeedRisingISR()
{
  // xSemaphoreGiveFromISR(interruptSemaphore, NULL);
  prev_waktu = waktu;
  attachInterrupt(digitalPinToInterrupt(SpeedSensor), SpeedFalingISR, RISING);

  waktu = millis() - falling_time;

  // batasan
  if (waktu <= 70)
  {
    waktu = prev_waktu;
  }

  speeds = (1.75 / waktu) * 3600; // m/detik
}

void handleMaxSpeed()
{
  if (autospeed)
  {
    if (speeds >= maxspeed)
    {
      digitalWrite(RelayPin, HIGH);
      engoff_time = millis();
    }

    else
    {
      if (millis() - engoff_time >= 5000)
      {
        digitalWrite(RelayPin, LOW);
      }
    }
  }

  else
  {
    digitalWrite(RelayPin, LOW);
  }
}

void handleMinSpeed()
{
  if (millis() - falling_time > 5000)
  {
    speeds = 0;
  }
}

void maxspeedHand()
{
  if (maxspeed > 0)
  {
    autospeed = true;
  }
  else
  {
    autospeed = false;
  }
}

void handleLimiter()
{
  if (selectedLimiter == 1)
  {
    maxspeed = limiterSatu;
  }
  else if (selectedLimiter == 2)
  {
    maxspeed = limiterDua;
  }
  else if (selectedLimiter == 3)
  {
    maxspeed = limiterTiga;
  }
}

void IRAM_ATTR EXT_RPM()
{
  // Serial.println(rev);
  rev++;
  // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // vTaskNotifyGiveFromISR(rpmSemaphore, &xHigherPriorityTaskWoken);
  // if (xHigherPriorityTaskWoken)
  // {
  //   portYIELD_FROM_ISR();
  // }
}

void CalculateRPM(void *pvParameters)
{

  for (;;)
  {
    unsigned long times = millis() - oldtimes;

    // Serial.println("KATANYA INTERRUPT");
    if (xSemaphoreTake(rpmSemaphore, portMAX_DELAY) == pdTRUE)
    {
      if (times >= 300)
      {
        detachInterrupt(digitalPinToInterrupt(RPMsensor));
        uint16_t newRPM = (60 * 1000) / (millis() - oldtimes) * rev * 2;

        uint16_t rpm = (newRPM + OldRPM) / 2;

        OldRPM = rpm;

        // rpm = (60 * 1000) / 100 * rev * 2;
        // rpm = (60 * 1000 * rev * 2) / 100;
        // rpm = rev*60*2;
        // Serial.println(rev);
        SendRPM(rpm);
        // Serial.println(rpm);
        rev = 0;
        oldtimes = millis();
        attachInterrupt(digitalPinToInterrupt(RPMsensor), EXT_RPM, FALLING);
      }
      xSemaphoreGive(rpmSemaphore);
    }
    // Serial.println("G");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sendSpeedSerial(uint8_t speed)
{
  Serial2.print(80, HEX);
  // __uint32_t speedHex = (__uint32_t)speed, HEX(speed);
  // char data_to_send[TX_BUF_SIZE];
  // snprintf(data_to_send, sizeof(data_to_send), "%04X", speedHex);
  // SerialUART.write((uint8_t*)&speedHex,sizeof(speedHex));
  // static_cast<char>(dataSendArray);
  // char dataSendArray = static_cast<char>(dataSendArrayUtama);
  // Serial2.print(kecepatan,HEX);
}

void sendUsartArray(uint8_t dataSend, uint8_t kecepatan, uint8_t limiterpertama, uint8_t limiterkedua, uint8_t limiterketiga)
{
  dataSend[4] = {kecepatan, limiterpertama, limiterkedua, limiterketiga};
  char dataSendArray[12];
  snprintf(dataSendArray, sizeof(dataSendArray), "%02x %02x %02x %02x", dataSend[0], dataSend[1], dataSend[2], dataSend[3]);
  Serial2.write((uint8_t *)&dataSendArray, sizeof(dataSendArray));
}
