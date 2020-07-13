// MPU6050WE.h

#ifndef _MPU6050WE_h
#define _MPU6050WE_h

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

class MPU6050{
public:
  MPU6050(TwoWire &w);
  MPU6050(TwoWire &w, float aC, float gC);
  void begin();

  float getAccX() { return accX; };
  float getAccY() { return accY; };
  float getAccZ() { return accZ; };
  
  void update();
  float getAngleZ() { return angleZ; };
  void setGyroOffsets(float x, float y, float z);
  void writeMPU6050(byte reg, byte data);
  byte readMPU6050(byte reg);
  void calcGyroOffsets(bool console = false);

private:
  

  TwoWire *wire;
  
  int16_t rawAccX, rawAccY, rawAccZ, rawGyroZ;
  
  float gyroXoffset, gyroYoffset, gyroZoffset;

  float accX, accY, accZ, gyroZ;

  float angleGyroZ, angleZ,angleAccZ ;

  long interval, preInterval;
  
  float accCoef, gyroCoef;
};

#endif

