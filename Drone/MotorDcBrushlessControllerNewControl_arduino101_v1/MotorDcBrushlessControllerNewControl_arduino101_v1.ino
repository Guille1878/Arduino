/*
  Wille Esteche

  // In 1080 and 1090 always staqrt again after it got hang.

*/
#include <Wire.h>
#include <Servo.h>

#define WithGYROSCOPE
//#define WithAIRPRESSURE

// GyroScope
#ifdef WithGYROSCOPE
  #include "MPU6050WE.h"
  MPU6050 mpu6050(Wire);

  typedef struct AvgMessures
  {
    float maxValue = 0;
    float minValue = 0;
    float accResult = 0;
    float loopValue = 0;
    float currentValue = 0;
  };
  
  typedef struct AxisCommandos
  {
    float Offset = 0;
    float WishedValue = 0;
  };
  
  bool withoutOffsets = false;
  
  AxisCommandos RollCommandos,
                PitchCommandos;
  //              YawCommandos,
  //              VerticalCommandos,
  //              SideCommandos;
  
#endif

// Altitude
#ifdef WithAIRPRESSURE

  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP280.h>

  #define BMP_SCK 13
  #define BMP_MISO 12 // SDO
  #define BMP_MOSI 11 // SDI
  #define BMP_CS 10 //CSB
  Adafruit_BMP280 bme(BMP_CS); // hardware SPI

#endif


//Relay
#define PIN_RELAY 3

// Motors
#define ESC_HIGH_DEFAULT 1000
#define ESC_LOW_DEFAULT 20

//Servo escs[4];  // create servo object to control a servo  
//bool isInSettingMode = false;

bool rinding = false;
bool isRideProcessing = false;

typedef struct 
{
  float currentSpeed = ESC_LOW_DEFAULT;
  float balanceRoll = 0;
  float balancePitch = 0;
  Servo esc;
} Motor;

Motor motors[4];


void setup() {

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  motors[0].esc.attach(4);  // attaches the servo on pin 4 to the motor FrontLeft
  motors[1].esc.attach(5);  // attaches the servo on pin 5 to the motor FrontRight
  motors[2].esc.attach(6);  // attaches the servo on pin 6 to the motor BackRight
  motors[3].esc.attach(7);  // attaches the servo on pin 7 to the motor BackLeft

  delay(3000);

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].esc.write(motors[motorIndex].currentSpeed);
  }

  delay(1000);

  Serial.begin(9600);

  Serial.println("Motors initiated");
  
  Wire.begin();

#ifdef WithGYROSCOPE 
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(1000);
  Serial.println("Gyroscope ON");
#endif

#ifdef WithAIRPRESSURE
  if (!bme.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  else
    Serial.println("Air Pressure ON");

  ride(true,Wifi);
  Serial.println("Offset done!");

#endif

  Serial.println("Ready to start.");

}


int8_t looptimes = 0;
void loop() {

#ifdef WithGYROSCOPE
  if (looptimes == 50)
  {
    if (rinding && !isRideProcessing)
      ride(false);

    looptimes = 0;
  }
  looptimes++;
#endif

}

void SetMortosToZero()
{
  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].currentSpeed = ESC_LOW_DEFAULT;
    motors[motorIndex].esc.write(motors[motorIndex].currentSpeed);
  }
}


#ifdef WithGYROSCOPE
int8_t MotorDifferenceMarginal = 6;
void ride(bool onlyOffset)
{
  isRideProcessing = true;
  
  float VerticalAcceptanceMarginal = 0.2;
  float PlanningAxisAcceptanceMarginal = 0.1;
  float MaxAngleOnAxis = 0.45;

  int8_t UpOrDownSpeed = 0;
  int8_t bestOf = 5;

  AvgMessures RollMessures,
              PitchMessures;
//              SideMessures,
//              YawMessures,
//              VerticalMessures;

  mpu6050.update();

  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {

    RollMessures.loopValue = mpu6050.getAccY();
    PitchMessures.loopValue = mpu6050.getAccX();
//    SideMessures.loopValue = mpu6050.getAccZ();
//    YawMessures.loopValue = mpu6050.getAngleZ();
//    VerticalMessures.loopValue = bme.readAltitude(1022);

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

//    if (counterIndex == 0)
//      SideMessures.maxValue = SideMessures.minValue = SideMessures.loopValue;
//    else if (SideMessures.loopValue > SideMessures.maxValue)
//      SideMessures.maxValue = SideMessures.loopValue;
//    else if (SideMessures.loopValue < SideMessures.minValue)
//      SideMessures.minValue = SideMessures.loopValue;
//
//    if (counterIndex == 0)
//      YawMessures.maxValue = YawMessures.minValue = YawMessures.loopValue;
//    else if (YawMessures.loopValue > YawMessures.maxValue)
//      YawMessures.maxValue = YawMessures.loopValue;
//    else if (YawMessures.loopValue < YawMessures.minValue)
//      YawMessures.minValue = YawMessures.loopValue;
//
//    if (counterIndex == 0)
//      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
//      VerticalMessures.maxValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
//      VerticalMessures.minValue = VerticalMessures.loopValue;

  }

  RollMessures.currentValue = ((int)(((RollMessures.accResult - RollMessures.maxValue - RollMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
  PitchMessures.currentValue = ((int)(((PitchMessures.accResult - PitchMessures.maxValue - PitchMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  SideMessures.currentValue = ((int)(((SideMessures.accResult - SideMessures.maxValue - SideMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  YawMessures.currentValue = ((int)(((YawMessures.accResult - YawMessures.maxValue - YawMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;

  if (onlyOffset)
  {
    PitchCommandos.Offset = PitchMessures.currentValue * -1;
    RollCommandos.Offset = RollMessures.currentValue * -1;
//    YawCommandos.Offset = YawMessures.currentValue * -1;
//    VerticalCommandos.Offset = VerticalMessures.currentValue * -1;
//    SideCommandos.Offset = SideMessures.currentValue * -1;
  }
  else
  {
    if (!withoutOffsets)
    {    
      PitchMessures.currentValue += PitchCommandos.Offset;
      RollMessures.currentValue += RollCommandos.Offset;
      //    YawMessures.currentValue += YawCommandos.Offset;
      //    VerticalMessures.currentValue += VerticalCommandos.Offset;
      //    SideMessures.currentValue += SideCommandos.Offset;
    }

    if (PitchMessures.currentValue > (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
      Serial.print("PitchValue: "); Serial.println(PitchMessures.currentValue);
      Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true);
    }
    else if (PitchMessures.currentValue < (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
      Serial.print("PitchValue: "); Serial.println(PitchMessures.currentValue);
      Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false);
    }
    
    if (RollMessures.currentValue > (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
      Serial.print("RollValue: "); Serial.println(RollMessures.currentValue);
      Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true);
    }
    else if (RollMessures.currentValue < (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
      Serial.print("RollValue: "); Serial.println(RollMessures.currentValue);
      Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false);
    }
  }
  isRideProcessing = false;
}
#endif

void setMotorDifferenceMarginal(int motorDifferenceMarginal)
{
#ifdef WithGYROSCOPE
    MotorDifferenceMarginal = motorDifferenceMarginal;
    Serial.print("MotorDifferenceMarginal upset to: ");
    Serial.println(MotorDifferenceMarginal);

#endif   
}

void setGyroOffsets(int OnOff)
{
#ifdef WithGYROSCOPE

    withoutOffsets = (OnOff == 1);

    Serial.print("Offsets ");
    Serial.println(withoutOffsets ? "ON" : "OFF");

#endif   
}


void setSpeed(Motor motor, int newSpeed)
{
  if (newSpeed > 0)
  {
    int speed = newSpeed + motor.balancePitch + motor.balanceRoll;
    int angle = map(speed, ESC_LOW_DEFAULT, ESC_HIGH_DEFAULT, 0, 180);
    motor.esc.write(angle);
    //motor.esc.write(newSpeed + motor.balancePitch + motor.balanceRoll);
    delay(2);
  }
}

void SpeedUpOrDown(int commandoValue, int newSpeed)
{

  String motrosStringPins = String(commandoValue);
  /*
    Serial.print(F("PinsString: "));
    Serial.print(motrosStringPins);
    Serial.print(F(". Value to add: "));
    Serial.println(newSpeed);
  */
  for (int8_t index = 0; index < 4; index++)
  {
    /*
        Serial.print(F("Loop string part: "));
        Serial.println(motrosStringPins.substring(index, index + 1));
    */

    if (motrosStringPins.substring(index, index + 1) == "1")
    {
      if (motors[index].currentSpeed + newSpeed < ESC_LOW_DEFAULT)
        motors[index].currentSpeed = ESC_LOW_DEFAULT;
      else if (motors[index].currentSpeed + newSpeed > ESC_HIGH_DEFAULT)
        motors[index].currentSpeed = ESC_HIGH_DEFAULT;
      else
        motors[index].currentSpeed += newSpeed;

      setSpeed(motors[index], motors[index].currentSpeed);

        Serial.print(F("Motor: "));
        Serial.print(index);
        Serial.print(F(" upset to "));
        Serial.println(motors[index].currentSpeed);
      
    }
  }
}

void Motors_BalanceRoll(int8_t addingSpeed, bool toLeft)
{
  
  addingSpeed /= 2;

  float multiplicateToSides = toLeft ? 1 : -1;

  if ((motors[0].balanceRoll == addingSpeed * multiplicateToSides))
    return;

  motors[0].balanceRoll = addingSpeed * multiplicateToSides;
  motors[1].balanceRoll = -addingSpeed * multiplicateToSides;
  motors[2].balanceRoll = -addingSpeed * multiplicateToSides;
  motors[3].balanceRoll = addingSpeed * multiplicateToSides;

  ReloadSpeedOnMotors();

  Serial.print(F("BalanceRoll: "));
  Serial.print(addingSpeed);
  Serial.print(F(", isToLeft: "));
  Serial.println(toLeft);
 
}

void Motors_BalancePitch(int8_t addingSpeed, bool toFront)
{
  addingSpeed /= 2;

  float multiplicateToSides = toFront ? 1 : -1;

  if ((motors[0].balancePitch == addingSpeed * multiplicateToSides))
    return;

  motors[0].balancePitch = addingSpeed * multiplicateToSides;
  motors[1].balancePitch = addingSpeed * multiplicateToSides;
  motors[2].balancePitch = -addingSpeed * multiplicateToSides;
  motors[3].balancePitch = -addingSpeed * multiplicateToSides;

  ReloadSpeedOnMotors();

  Serial.print(F("BalancePitch: "));
  Serial.print(addingSpeed);
  Serial.print(F(", isToFront: "));
  Serial.println(toFront);
  
}

void ReloadSpeedOnMotors()
{

  for (int index = 0; index < 4; index++)
  {
    motors[index].esc.write(motors[index].currentSpeed + motors[index].balancePitch + motors[index].balanceRoll);
    delay(20);
  }  
}

