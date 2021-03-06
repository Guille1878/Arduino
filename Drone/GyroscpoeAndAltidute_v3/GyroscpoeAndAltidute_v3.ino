#include <Wire.h>
#include <SPI.h>
#include <MPU6050_tockn.h>
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

  Serial.print("Air presure ON");
  Serial.println("");

  ride(true);

  Serial.println("Offset done!");

  Serial.println("Starting");
}

typedef struct AxisCommandos
{
    float Offset = 0;
    float WishedValue = 0; 
};

AxisCommandos RollCommandos,
              PitchCommandos,
              YawCommandos,
              VerticalCommandos,
              SideCommandos;

typedef struct AvgMessures
{ 
  float maxValue = 0;
  float minValue = 0;
  float accResult = 0;
  float loopValue = 0;
  float currentValue = 0;
};

void loop() 
{

  ride(false);
  
  delay(50);
}

void showMesearues()
{

  mpu6050.update();
  
  int8_t bestOf = 5;

  AvgMessures RollMessures, 
        PitchMessures,
        SideMessures,
        YawMessures,
        VerticalMessures;

  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {
    
    VerticalMessures.loopValue = bme.readAltitude(1022);
    RollMessures.loopValue = mpu6050.getAccY();
    PitchMessures.loopValue = mpu6050.getAccX();
    SideMessures.loopValue = mpu6050.getAccZ();
    YawMessures.loopValue = mpu6050.getAngleZ();

    if (counterIndex == 0)
      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
      VerticalMessures.maxValue = VerticalMessures.loopValue;
    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
      VerticalMessures.minValue = VerticalMessures.loopValue;

     if (counterIndex == 0)
      RollMessures.maxValue = RollMessures.minValue = RollMessures.loopValue;
    else if (RollMessures.loopValue > RollMessures.maxValue)
      RollMessures.maxValue = RollMessures.loopValue;
    else if (RollMessures.loopValue < RollMessures.minValue)
      RollMessures.minValue = RollMessures.loopValue;

     if (counterIndex == 0)
      PitchMessures.maxValue = PitchMessures.minValue = PitchMessures.loopValue;
    else if (PitchMessures.loopValue > PitchMessures.maxValue)
      PitchMessures.maxValue = PitchMessures.loopValue;
    else if (PitchMessures.loopValue < PitchMessures.minValue)
      PitchMessures.minValue = PitchMessures.loopValue;

     if (counterIndex == 0)
      SideMessures.maxValue = SideMessures.minValue = SideMessures.loopValue;
    else if (SideMessures.loopValue > SideMessures.maxValue)
      SideMessures.maxValue = SideMessures.loopValue;
    else if (SideMessures.loopValue < SideMessures.minValue)
      SideMessures.minValue = SideMessures.loopValue;

     if (counterIndex == 0)
      YawMessures.maxValue = YawMessures.minValue = YawMessures.loopValue;
    else if (YawMessures.loopValue > YawMessures.maxValue)
      YawMessures.maxValue = YawMessures.loopValue;
    else if (YawMessures.loopValue < YawMessures.minValue)
      YawMessures.minValue = YawMessures.loopValue;
        
  }

    RollMessures.currentValue = ((int)(((RollMessures.accResult - RollMessures.maxValue - RollMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
    PitchMessures.currentValue = ((int)(((PitchMessures.accResult - PitchMessures.maxValue - PitchMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
    SideMessures.currentValue = ((int)(((SideMessures.accResult - SideMessures.maxValue - SideMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
    YawMessures.currentValue = ((int)(((YawMessures.accResult - YawMessures.maxValue - YawMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
    VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100))/100.0; 

    String returning = "R:" + String(RollMessures.currentValue) + 
                       "P:" + String(PitchMessures.currentValue) +
                       "S:" + String(SideMessures.currentValue) +
                       "Y:" + String(YawMessures.currentValue) +
                       "A:" + String(VerticalMessures.currentValue);  

    Serial.println(returning);

  
}
       
  int8_t UpOrDownSpeed = 0;

void ride(bool onlyOffset)
{
  float VerticalAcceptanceMarginal = 0.2;
  float PlanningAxisAcceptanceMarginal = 0.05;
  int8_t MotorDifferenceMarginal = 10;
  float MaxAngleOnAxis = 0.45;
    
  int8_t bestOf = 5;

  AvgMessures RollMessures, 
        PitchMessures,
        SideMessures,
        YawMessures,
        VerticalMessures;
  
   mpu6050.update();
  
  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {
    
    VerticalMessures.loopValue = bme.readAltitude(1022);
    RollMessures.loopValue = mpu6050.getAccY();
    PitchMessures.loopValue = mpu6050.getAccX();
    SideMessures.loopValue = mpu6050.getAccZ();
    YawMessures.loopValue = mpu6050.getAngleZ();

    if (counterIndex == 0)
      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
      VerticalMessures.maxValue = VerticalMessures.loopValue;
    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
      VerticalMessures.minValue = VerticalMessures.loopValue;

     if (counterIndex == 0)
      RollMessures.maxValue = RollMessures.minValue = RollMessures.loopValue;
    else if (RollMessures.loopValue > RollMessures.maxValue)
      RollMessures.maxValue = RollMessures.loopValue;
    else if (RollMessures.loopValue < RollMessures.minValue)
      RollMessures.minValue = RollMessures.loopValue;

     if (counterIndex == 0)
      PitchMessures.maxValue = PitchMessures.minValue = PitchMessures.loopValue;
    else if (PitchMessures.loopValue > PitchMessures.maxValue)
      PitchMessures.maxValue = PitchMessures.loopValue;
    else if (PitchMessures.loopValue < PitchMessures.minValue)
      PitchMessures.minValue = PitchMessures.loopValue;

     if (counterIndex == 0)
      SideMessures.maxValue = SideMessures.minValue = SideMessures.loopValue;
    else if (SideMessures.loopValue > SideMessures.maxValue)
      SideMessures.maxValue = SideMessures.loopValue;
    else if (SideMessures.loopValue < SideMessures.minValue)
      SideMessures.minValue = SideMessures.loopValue;

     if (counterIndex == 0)
      YawMessures.maxValue = YawMessures.minValue = YawMessures.loopValue;
    else if (YawMessures.loopValue > YawMessures.maxValue)
      YawMessures.maxValue = YawMessures.loopValue;
    else if (YawMessures.loopValue < YawMessures.minValue)
      YawMessures.minValue = YawMessures.loopValue;
        
  }

  RollMessures.currentValue = ((int)(((RollMessures.accResult - RollMessures.maxValue - RollMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
  PitchMessures.currentValue = ((int)(((PitchMessures.accResult - PitchMessures.maxValue - PitchMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
  SideMessures.currentValue = ((int)(((SideMessures.accResult - SideMessures.maxValue - SideMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
  YawMessures.currentValue = ((int)(((YawMessures.accResult - YawMessures.maxValue - YawMessures.minValue) / (bestOf - 2)) * 100))/100.0; 
  VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100))/100.0; 

  if (onlyOffset)
  {
      PitchCommandos.Offset = PitchMessures.currentValue * -1;
      RollCommandos.Offset = RollMessures.currentValue * -1;
      YawCommandos.Offset = YawMessures.currentValue * -1;
      VerticalCommandos.Offset = VerticalMessures.currentValue * -1;
      SideCommandos.Offset = SideMessures.currentValue * -1;
  }
  else
  {
      PitchMessures.currentValue += PitchCommandos.Offset;
      RollMessures.currentValue += RollCommandos.Offset;
      YawMessures.currentValue += YawCommandos.Offset;
      VerticalMessures.currentValue += VerticalCommandos.Offset;
      SideMessures.currentValue += SideCommandos.Offset;
    
    if (PitchMessures.currentValue > (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
      Motors_MoveToFront(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis);
    else if (PitchMessures.currentValue < (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
      Motors_MoveToBack(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis);
  
    if (RollMessures.currentValue > (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
      Motors_MoveToLeft(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis);
    else if (RollMessures.currentValue < (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal: PlanningAxisAcceptanceMarginal * -1)) 
      Motors_MoveToRight(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis);
  }  
}


void Motors_MoveToLeft(int8_t addingSpeed)
{
  Serial.print("MoveToLeft: ");
  Serial.println(addingSpeed);
  /*
  addingSpeed /= 2;
  SpeedUpOrDown(1221, addingSpeed);
  SpeedUpOrDown(2112, -addingSpeed);
  */
}

void Motors_MoveToRight(int8_t addingSpeed)
{
  Serial.print("MoveToRight: ");
  Serial.println(addingSpeed);

  /*
  addingSpeed /= 2;
  SpeedUpOrDown(1221, -addingSpeed);
  SpeedUpOrDown(2112, addingSpeed);
  */
}

void Motors_MoveToFront(int8_t addingSpeed)
{
    Serial.print("MoveToFront: ");
  Serial.println(addingSpeed);
  /*
  addingSpeed /= 2;
  SpeedUpOrDown(1122, addingSpeed);
  SpeedUpOrDown(2211, -addingSpeed);
  */
}

void Motors_MoveToBack(int8_t addingSpeed)
{
    Serial.print("MoveToBack: ");
    Serial.println(addingSpeed);
  /*
  addingSpeed /= 2;
  SpeedUpOrDown(1122, -addingSpeed);
  SpeedUpOrDown(2211, addingSpeed);
  */
}



