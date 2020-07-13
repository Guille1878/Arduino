#include <Wire.h>
#include <SPI.h>
#include "MPU6050WE.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12 // SDO
#define BMP_MOSI 11 // SDI
#define BMP_CS 10 //CSB

MPU6050 mpu6050(Wire);
Adafruit_BMP280 bme(BMP_CS); // hardware SPI

//long timer = 0;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Serial.println("");

  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

int8_t bestOf = 5, counterIndex = 0;

  float CurrentRoll = 0, 
        CurrentPitch = 0,
        CurrentSide = 0,
        CurrentYaw = 0,
        CurrentVertical = 0;

void loop() {
    
  mpu6050.update();

  CurrentVertical = bme.readAltitude(1022);
  CurrentRoll = mpu6050.getAccY();
  CurrentPitch = mpu6050.getAccX();
  CurrentSide = mpu6050.getAccZ();
  CurrentVertical = mpu6050.getAngleZ();

    String returning = "R:" + String(CurrentRoll) + 
                       "/P:" + String(CurrentPitch) +
                       "/S:" + String(CurrentSide) +
                       "/Y:" + String(CurrentYaw) +
                       "/A:" + String(CurrentVertical);  

    Serial.println(returning);

  delay(10);
}

float AvgBestOf(float *arrayNumbers, int8_t bestOf)
{
  float maxValue, minValue;
  float accResult = 0;
  for (int index = 0; index < bestOf; index++)
  {
    float loopValue = arrayNumbers[index];
    if (index == 0)
      maxValue = minValue = loopValue;
    else if (loopValue > maxValue)
      maxValue = loopValue;
    else if (loopValue < minValue)
      minValue = loopValue;

    accResult += loopValue;
  }
  return (accResult - maxValue - minValue) / (bestOf - 2);
}
