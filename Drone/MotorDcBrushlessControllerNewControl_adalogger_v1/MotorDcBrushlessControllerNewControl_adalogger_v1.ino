/*
  Wille Esteche

  // In 1080 and 1090 always staqrt again after it got hang.

*/
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>

#define WithGYROSCOPE
#define WithAIRPRESSURE
#define WithSERIALPRINT

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
uint8_t requestId = 9;
float totalAltitude = 0;

typedef struct 
{
  float currentSpeed = ESC_LOW_DEFAULT;
  float balanceRoll = 0;
  float balancePitch = 0;
  void setSpeed(int newSpeed)
  {
    if (newSpeed > 0)
    {
      int speed = newSpeed + balancePitch + balanceRoll;
      int angle = map(speed, ESC_LOW_DEFAULT, ESC_HIGH_DEFAULT, 0, 180);
      esc.write(angle);
      delay(2);
    }
  };
  Servo esc;
} Motor;

Motor motors[4];

void setup() {

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
 
  motors[0].esc.attach(4);  // attaches the servo on pin 4 to the motor FrontLeft
  motors[1].esc.attach(5);  // attaches the servo on pin 5 to the motor FrontRight
  motors[2].esc.attach(6);  // attaches the servo on pin 6 to the motor BackRight
  motors[3].esc.attach(7);  // attaches the servo on pin 7 to the motor BackLeft

  delay(1000);

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
    motors[motorIndex].setSpeed(motors[motorIndex].currentSpeed);

  delay(1000);

  Wire.begin(8);
  Wire.onReceive(receiveEvent); 

#ifdef WithSERIALPRINT
  Serial.begin(9600);
  Serial.println("Motors initiated");
#endif

uint8_t tryingTimes = 0;
#ifdef WithAIRPRESSURE
  if (!bme.begin()) {
   #ifdef WithSERIALPRINT
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
   #endif
    while (1);
  }
  
  #ifdef WithSERIALPRINT
    Serial.println("Air Pressure ON");
  #endif
#endif

#ifdef WithGYROSCOPE 
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(1000);

  ride(true);
  
  #ifdef WithSERIALPRINT  
    Serial.println("Offset done!");
    Serial.println("Gyroscope ON");
  #endif
#endif


#ifdef WithSERIALPRINT  
  Serial.println("Ready to start.");
#endif

}

bool isCheching = false;
int8_t looptimes = 0;
void loop() {

#ifdef WithGYROSCOPE
  if (looptimes == 49)
  {
    if (rinding && !isRideProcessing)
      ride(false);

    looptimes = 0;
  }
  looptimes++;
#endif

#ifdef WithAIRPRESSURE
if (looptimes == 15 && looptimes == 35)
  calculatingAltitude();
#endif
}

void SetMortosToZero()
{
  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].currentSpeed = ESC_LOW_DEFAULT;
    motors[motorIndex].setSpeed(motors[motorIndex].currentSpeed);
  }
}

 
void getCommando (uint32_t commando)
{
    uint32_t processParameter = (commando / 10);
    int processId = commando - (processParameter * 10);
    #ifdef WithSERIALPRINT
      Serial.print(F("processParameter: ")); Serial.println(processParameter);
      Serial.print(F("processId: ")); Serial.println(processId);
    #endif  
    switch (processId)
    {
      case 1: //for speed vaalue on motors moving
      case 2:
      {
          String sMotors = String(processParameter).substring(0,4);
          int motors = sMotors.toInt();
          String IsPositive = String(processParameter).substring(4,5);
          String sSpeed = String(processParameter).substring(5);
          int speedValue = sSpeed.toInt();
          if (IsPositive == "1")
            speedValue *= -1;

          moveMotors(motors, speedValue);
        
        break;
      }
      case 3:
      {
        getASignal();
        //calibrateMotors(processParameter);
        #ifdef WithSERIALPRINT
          Serial.println("!!Process 3!! ");
        #endif
        break;
      }
#ifdef WithAIRPRESSURE        
      case 4:
      {
        requestId = 4;
        #ifdef WithSERIALPRINT
          Serial.println("!!Process 4!! ");
        #endif
        break;
      }
#endif          
      case 5:
      {
        turnOnOffMotors(processParameter);
        #ifdef WithSERIALPRINT
          Serial.println("!!Process 5!! ");
        #endif
        break;
      }        
#ifdef WithGYROSCOPE        
      case 6:
      {
        setMotorDifferenceMarginal(processParameter);
        #ifdef WithSERIALPRINT
          Serial.println("!!Process 6!! ");
        #endif        
        break;
      }
      case 7:
      {
        turnOnOffOffsets(processParameter);
        break;
      }
      case 8:
      {
        turnOnOffRiding(processParameter);
        #ifdef WithSERIALPRINT
          Serial.println("!!Process 8!! ");
        #endif        
        break;
      }
#endif        
      case 9:
      {
        requestId = 9;
        #ifdef WithSERIALPRINT
          Serial.println("!!Process 9!! ");
        #endif        
        break;
      }
    }        
}


bool signalOn = true;
void getASignal()
{
   #ifdef WithSERIALPRINT
      Serial.println(F("getASignal: "));
   #endif 

  if (signalOn = !signalOn)
    digitalWrite(LED_BUILTIN, LOW);
  else 
    digitalWrite(LED_BUILTIN, HIGH);
  
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
#ifdef WithSERIALPRINT        
      Serial.print("PitchValue: "); Serial.println(PitchMessures.currentValue);
#endif      
      Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true);
    }
    else if (PitchMessures.currentValue < (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
#ifdef WithSERIALPRINT        
      Serial.print("PitchValue: "); Serial.println(PitchMessures.currentValue);
#endif      
      Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false);
    }
    
    if (RollMessures.currentValue > (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
#ifdef WithSERIALPRINT        
      Serial.print("RollValue: "); Serial.println(RollMessures.currentValue);
#endif      
      Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true);
    }
    else if (RollMessures.currentValue < (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
#ifdef WithSERIALPRINT        
      Serial.print("RollValue: "); Serial.println(RollMessures.currentValue);
#endif      
      Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false);
    }
  }
  isRideProcessing = false;
}
#endif

void receiveEvent(int howMany) 
{
  uint32_t response = 0;
  
  response = (uint32_t) Wire.read() << 24;
  response |= (uint32_t) Wire.read() << 16;
  response |= (uint32_t) Wire.read() << 8;
  response |= (uint32_t) Wire.read(); 
  
#ifdef WithSERIALPRINT    
  Serial.println(F("Reviced y Event: "));Serial.println(response);         // print the integer
#endif   

  getCommando(response);
  
}

/*
void requestEvent() {  

  switch(requestId)
  {
    case 4:
      char caAltitude[8];
      String(totalAltitude).toCharArray(caAltitude,8);
      Wire.write(caAltitude); 
      break;
    case 9:
      long volt = readVcc();
      char caVolt[8];
      String(volt).toCharArray(caVolt,8);
      Wire.write(caVolt); 
      break;
  }   
}
*/

void turnOnOffMotors(int OnOff)
{
  if (OnOff == 1)
    {
      digitalWrite(PIN_RELAY, HIGH);
#ifdef WithSERIALPRINT
      Serial.println("Motors ON");
#endif      
    }
    else
    {
      SetMortosToZero();
      digitalWrite(PIN_RELAY, LOW);
#ifdef WithSERIALPRINT
      Serial.println("Motors OFF");
#endif            
    }
}

void calibrateMotors(int processParameter)
{
  
}

void moveMotors(int processParameter, int speedValue)
{
  if (processParameter >= 1111 && processParameter < 2222)
  {
    int newSpeed = (speedValue == 0) ? ESC_LOW_DEFAULT : speedValue;

    String motrosStringPins = String(processParameter);
#ifdef WithSERIALPRINT
    Serial.print(F("PinsString: "));
    Serial.print(motrosStringPins);
    Serial.print(F(". Value to add: "));
    Serial.println(newSpeed);
#endif
    for (int index = 0; index < 4; index++)
    {
#ifdef WithSERIALPRINT      
      Serial.print(F("Loop string part: "));
      Serial.println(motrosStringPins.substring(index, index + 1));
#endif    

      if (motrosStringPins.substring(index, index + 1) == "1")
      {
        if (motors[index].currentSpeed + newSpeed < ESC_LOW_DEFAULT)
          motors[index].currentSpeed = ESC_LOW_DEFAULT;
        else if (motors[index].currentSpeed + newSpeed > ESC_HIGH_DEFAULT)
          motors[index].currentSpeed = ESC_HIGH_DEFAULT;
        else
          motors[index].currentSpeed += newSpeed;

        motors[index].setSpeed(motors[index].currentSpeed);
#ifdef WithSERIALPRINT     
        Serial.print(F("Motor: "));
        Serial.print(index);
        Serial.print(F(" set to "));
        Serial.println(motors[index].currentSpeed);
#endif            
      }
    }
  }
}

#ifdef WithGYROSCOPE
void turnOnOffRiding(int OnOff)
{
  rinding = (OnOff == 1);
}

void setMotorDifferenceMarginal(int value)
{
  MotorDifferenceMarginal = value;
}

void turnOnOffOffsets(int OnOff)
{
    withoutOffsets = (OnOff == 1);
}
#endif

void SpeedUpOrDown(int commandoValue, int newSpeed)
{

  String motrosStringPins = String(commandoValue);
  for (int8_t index = 0; index < 4; index++)
  {

    if (motrosStringPins.substring(index, index + 1) == "1")
    {
      if (motors[index].currentSpeed + newSpeed < ESC_LOW_DEFAULT)
        motors[index].currentSpeed = ESC_LOW_DEFAULT;
      else if (motors[index].currentSpeed + newSpeed > ESC_HIGH_DEFAULT)
        motors[index].currentSpeed = ESC_HIGH_DEFAULT;
      else
        motors[index].currentSpeed += newSpeed;

      motors[index].setSpeed(motors[index].currentSpeed);

#ifdef WithSERIALPRINT
        Serial.print(F("Motor: "));
        Serial.print(index);
        Serial.print(F(" upset to "));
        Serial.println(motors[index].currentSpeed);
#endif        
      
    }
  }
}

#ifdef WithAIRPRESSURE

float summingTotalAltitude = 0;
float summingMaxAltitude = 0;
float summingMinAltitude = 0;
uint8_t summingTimesAltitude = 0;

void calculatingAltitude()
{
  summingTimesAltitude++;
  float newValue = bme.readAltitude(1022);

  if (summingTimesAltitude == 1)
  {
    summingTotalAltitude = summingMaxAltitude = summingMinAltitude = newValue;
  }  
  else if (summingTimesAltitude == 5)
  {
    totalAltitude = (summingTotalAltitude - summingMaxAltitude - summingMinAltitude) / 3;
    summingTotalAltitude = summingTimesAltitude = 0;
  }
  else
  {
    summingTotalAltitude += newValue;
    if (newValue > summingMaxAltitude)
      summingMaxAltitude = newValue;
    else if (newValue < summingMinAltitude)
      summingMinAltitude = newValue;
  }
}


#endif


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

#ifdef WithSERIALPRINT
  Serial.print(F("BalanceRoll: "));
  Serial.print(addingSpeed);
  Serial.print(F(", isToLeft: "));
  Serial.println(toLeft);
#endif  
 
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

#ifdef WithSERIALPRINT
  Serial.print(F("BalancePitch: "));
  Serial.print(addingSpeed);
  Serial.print(F(", isToFront: "));
  Serial.println(toFront);
#endif  
  
}

void ReloadSpeedOnMotors()
{

  for (int index = 0; index < 4; index++)
  {
    motors[index].setSpeed(motors[index].currentSpeed + motors[index].balancePitch + motors[index].balanceRoll);
    delay(20);
  }  
}

