#include "MPU6050WE.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  
}

void MPU6050::writeMPU6050(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}


void MPU6050::calcGyroOffsets(bool console){
  float x = 0, y = 0, z = 0;
  int16_t rx, ry, rz;

  if(console){
    Serial.println("calculate gyro offsets");
    Serial.print("DO NOT MOVE A MPU6050");
  }
  for(int i = 0; i < 1000; i++){
    if(console && i % 1000 == 0){
      Serial.print(".");
    }
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x3B);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);

    wire->read() << 8 | wire->read();
    wire->read() << 8 | wire->read();
    wire->read() << 8 | wire->read();
    wire->read() << 8 | wire->read();
    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }
  gyroXoffset = x / 1000;
  gyroYoffset = y / 1000;
  gyroZoffset = z / 1000;
  
  if(console){
    Serial.println("Done!!!");
    Serial.print("X : ");Serial.println(gyroXoffset);
    Serial.print("Y : ");Serial.println(gyroYoffset);
    Serial.print("Z : ");Serial.println(gyroYoffset);
    Serial.print("Program will start after 3 seconds");
    delay(3000);
  }
}

void MPU6050::update(){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);
    
  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  wire->read() << 8 | wire->read();
  wire->read() << 8 | wire->read();
  wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

  accX = ((float)rawAccX) / 16384.0;
  accY = ((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;
  
  gyroZ = ((float)rawGyroZ) / 65.5;
  
  interval = millis() - preInterval;
  
  gyroZ -= gyroZoffset;

  angleGyroZ += gyroZ * (interval * 0.001);
  
  preInterval = millis();
  angleZ = angleGyroZ;
  
}



