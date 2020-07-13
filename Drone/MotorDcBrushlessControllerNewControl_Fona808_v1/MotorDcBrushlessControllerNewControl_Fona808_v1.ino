/*
  Wille Esteche

*/

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

//#define WithGYROSCOPE
//#define WithAIRPRESSURE
//#define WithPINCode
#define WithSERIALPRINT

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

char replybuffer[255];
String url = "http://droneapiv1.azurewebsites.net/api/Device2?device=2";

/*
String Longitude = "-1";
String Latitude = "-1";
String Altitude = "-1";
String GPSAltitude = "-1";
String GPSsatellitesUsed = "0";
String SignalLevel = "-1";
String SimBatteryLevel = "-1";
String DroneBatteryVolt = "-1";
*/
float Longitude = -1;
float Latitude = -1;
//float Altitude = -1;
float totalAltitude = 0;
float GPSAltitude = -1;
float GPSsatellitesUsed = 0;
//float SignalLevel = -1;
uint8_t SignalLevel = -1;
uint16_t SimBatteryLevel = -1;
long DroneBatteryVolt = -1;

bool isFonaOn = false;
bool isGpsOn = false;
bool manuallyTurnedOff = false;

uint8_t gsmFailed = 0;

struct CommandoDef
{
  int kind;
  uint32_t value;
};

#ifdef WithPINCode
  char PIN[4] = {'8','8','6','8' };
#endif

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
                PitchCommandos,
  //              YawCommandos,
                VerticalCommandos;
  //              SideCommandos;
  
#endif

// Altitude
#ifdef WithAIRPRESSURE

  #include <Adafruit_Sensor.h>
  #include "Adafruit_BMP3XX.h"

  #define SEALEVELPRESSURE_HPA (996.25) 
  #define BMP_CS 10 //CSB
  Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI

#endif


//Relay
#define PIN_RELAY 5

// Motors
#define ESC_HIGH_DEFAULT 1000
#define ESC_LOW_DEFAULT 20

//Servo escs[4];  // create servo object to control a servo  
//bool isInSettingMode = false;

bool rinding = false;
bool isRideProcessing = false;
uint8_t requestId = 9;

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

#ifdef WithSERIALPRINT
  Serial.begin(115000);
  Serial.println("FONA 808 with Arduino");
  Serial.println("Initializing....(May take 3 seconds)");
#endif


  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  pinMode(LED_BUILTIN, OUTPUT);

  motors[0].esc.attach(6);  // attaches the servo on pin 4 to the motor FrontLeft
  motors[1].esc.attach(7);  // attaches the servo on pin 5 to the motor FrontRight
  motors[2].esc.attach(8);  // attaches the servo on pin 6 to the motor BackRight
  motors[3].esc.attach(9);  // attaches the servo on pin 7 to the motor BackLeft

  delay(1000);

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
    motors[motorIndex].setSpeed(motors[motorIndex].currentSpeed);

#ifdef WithSERIALPRINT
  Serial.println("Motors initiated");
#endif

  delay(1000);

  Wire.begin();
  
 fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
#ifdef WithSERIALPRINT    
    Serial.println(F("Couldn't find FONA"));
#endif
    while (1);
  }
#ifdef WithSERIALPRINT  
  Serial.println("FONA found!!");
  Serial.println("GSM Ready to start.");
#endif  

uint8_t tryingTimes = 0;
#ifdef WithAIRPRESSURE
  if (!bmp.begin()) {
   #ifdef WithSERIALPRINT
    Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
   #endif
    while (1);
  }
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  
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

 if (!(isFonaOn && isGpsOn) && !manuallyTurnedOff)
  {
    /*
    Serial.print(F("isFonaOn:"));  Serial.println(isFonaOn);  
    Serial.print(F("isGpsOn:"));  Serial.println(isGpsOn);  
    Serial.print(F("manuallyTurnedOffn:"));  Serial.println(manuallyTurnedOff);  
    */
    initFona();
    delay(500);
  }
  else
  {
      updateFonaGPS();
      delay(200);
      updateFonaBatery();
      delay(100);
      updateSignalLevel();
      delay(200);      
      updateDroneBatteryLevel();
#ifdef WithAIRPRESSURE   
      delay(100);
      updatingAltitude();
#endif      
      delay(100);    
      getHttp();
      delay(300);
#ifdef WithGYROSCOPE
      if (rinding && !isRideProcessing)
        ride(false);
#endif      
  }

/*
#ifdef WithGYROSCOPE
  if (looptimes == 49)
  {
    if (rinding && !isRideProcessing)
      ride(false);

    looptimes = 0;
  }
  looptimes++;
#endif
*/
}

void initFona()
{

#ifdef WithPINCode
  bool unlocked = fona.unlockSIM(PIN);
   #ifdef WithSERIALPRINT
   if (unlocked)
    Serial.println(F("UNLOCKED!"));
   else
    Serial.println(F("Unlocking Failed"));
   #endif
#endif

  uint8_t tryTimes = 0;
  if (!isFonaOn)
  {    
    isFonaOn = fona.enableGPRS(true);
    while (!isFonaOn)
    {
      if (tryTimes == 10)
      {
    #ifdef WithSERIALPRINT        
        Serial.println(F("Failed when trying to Enable GPRS."));
    #endif
        isFonaOn = fona.enableGPRS(false);
        break;
      }
        
      tryTimes++;
      isFonaOn = fona.enableGPRS(true);
      delay(1000);
    }
    if (isFonaOn)
    {
  #ifdef WithSERIALPRINT
      Serial.println(F("GPRS is Enabled!"));
  #endif
    }
  }
  
  if (!isGpsOn)
  {
    tryTimes = 0;
    isGpsOn = fona.enableGPS(true);
    while (!isFonaOn)
    {
      if (tryTimes == 10)
      {
    #ifdef WithSERIALPRINT        
        Serial.println(F("Failed when trying to Enable GPS."));
    #endif
        isGpsOn = fona.enableGPS(false);
        break;
      }
        
      tryTimes++;
      isGpsOn = fona.enableGPS(true);
      
      delay(1000);
    }
    if (isGpsOn)
    {
  #ifdef WithSERIALPRINT        
      Serial.println(F("GPS is Enabled!"));
  #endif
    }
  }
  
   
}

void SetMotorsToZero()
{
  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].currentSpeed = ESC_LOW_DEFAULT;
    motors[motorIndex].setSpeed(motors[motorIndex].currentSpeed);
  }
}
 
void runCommando(uint32_t processParameter, uint16_t processId)
{
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
              PitchMessures,
//              SideMessures,
//              YawMessures,
              VerticalMessures;

  mpu6050.update();

  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {

    RollMessures.loopValue = mpu6050.getAccY();
    PitchMessures.loopValue = mpu6050.getAccX();
//    SideMessures.loopValue = mpu6050.getAccZ();
//    YawMessures.loopValue = mpu6050.getAngleZ();

#ifdef WithAIRPRESSURE
    if (bmp.performReading()) 
    {
      VerticalMessures.loopValue = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }
#endif

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
    if (counterIndex == 0)
      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
      VerticalMessures.maxValue = VerticalMessures.loopValue;
    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
      VerticalMessures.minValue = VerticalMessures.loopValue;

  }

  RollMessures.currentValue = ((int)(((RollMessures.accResult - RollMessures.maxValue - RollMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
  PitchMessures.currentValue = ((int)(((PitchMessures.accResult - PitchMessures.maxValue - PitchMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  SideMessures.currentValue = ((int)(((SideMessures.accResult - SideMessures.maxValue - SideMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  YawMessures.currentValue = ((int)(((YawMessures.accResult - YawMessures.maxValue - YawMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
  VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;

  if (onlyOffset)
  {
    PitchCommandos.Offset = PitchMessures.currentValue * -1;
    RollCommandos.Offset = RollMessures.currentValue * -1;
//    YawCommandos.Offset = YawMessures.currentValue * -1;
    VerticalCommandos.Offset = VerticalMessures.currentValue * -1;
//    SideCommandos.Offset = SideMessures.currentValue * -1;
  }
  else
  {
    if (!withoutOffsets)
    {    
      PitchMessures.currentValue += PitchCommandos.Offset;
      RollMessures.currentValue += RollCommandos.Offset;
      //    YawMessures.currentValue += YawCommandos.Offset;
      VerticalMessures.currentValue += VerticalCommandos.Offset;
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
      SetMotorsToZero();
      delay(2000);
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
        Serial.print(F("Motor: ")); Serial.print(index);
        Serial.print(F(" set to ")); Serial.println(motors[index].currentSpeed);
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

void updatingAltitude()
{
  summingTimesAltitude++;

  if (bmp.performReading()) {
 
    float newValue = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  
    if (summingTimesAltitude == 1)
    {
      summingTotalAltitude = summingMaxAltitude = summingMinAltitude = newValue;
    }  
    else if (summingTimesAltitude == 5)
    {
      totalAltitude = (summingTotalAltitude - summingMaxAltitude - summingMinAltitude) / 3;
      summingTotalAltitude = summingTimesAltitude = 0;
      //Altitude = String(totalAltitude);
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
}

#endif

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
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

// FUNCTIONS FOR FONA808

void turnOffGSM()
{
    uint8_t tryTimes = 0;
    manuallyTurnedOff = fona.enableGPRS(false);
    while (!manuallyTurnedOff)
    {
      if (tryTimes == 10)
      {
    #ifdef WithSERIALPRINT
        Serial.println(F("Failed when trying to Disable GPRS."));
    #endif
        break;
      }

      tryTimes++;
      manuallyTurnedOff = fona.enableGPRS(false);
      delay(1000);
    }
#ifdef WithSERIALPRINT
    Serial.println(F("GPRS Disabled"));
#endif
}

void getHttp()
{
   //GPRS Web =========================
   
        uint16_t statuscode;
        int16_t length;

        String entireUrl = getUrl();
        int urlLength = entireUrl.length();
        char urlChars[urlLength];
        entireUrl.toCharArray(urlChars, urlLength + 1);        

#ifdef WithSERIALPRINT          
  Serial.println("\nGPRS =========================");
  Serial.print("GET: "); Serial.println(urlChars);
#endif

        if (!fona.HTTP_GET_start(urlChars, &statuscode, (uint16_t *)&length)) {
  #ifdef WithSERIALPRINT          
          Serial.print("HTTP Failed! "); Serial.println(statuscode);
  #endif          
          gsmFailed++;

          switch (gsmFailed)
          {
            case 4:
              turnOffGSM();
              isFonaOn = false;
              manuallyTurnedOff = false;
              break;
            case 9:
              turnOnOffMotors(0);
              turnOffGSM();
              break;
          }
          delay(1000);
          return;
        }
        
      gsmFailed = 0;
          
      char charResultIn[length];
      int charIndex = 0,
          startCommandIndex = -1,
          commandsLength = 0,
          commandsIndex = -1,
          commandKind = -1;
    bool feedingValue = false;          
    String commandValuein = "";

    CommandoDef commands[8];
     
   while (length > 0) {
          while (fona.available()) {
            //Serial.write((char)fona.read());
            char charIn = fona.read();
            charResultIn[charIndex++] = charIn;

            if (charIndex == 1)
            {
              if (charIn != 's')
              {
                #ifdef WithSERIALPRINT
                   Serial.print(F("Response: NOT SUCCESS")); 
                #endif
                fona.HTTP_GET_end();
                return;
              }
            }
            else if (charIndex == 3)
            {
               if (charResultIn[1] == '0' && charResultIn[2] == '0')
               {
                 #ifdef WithSERIALPRINT
                    Serial.print(F("Response: SUCCESS. BUT NO ANY COMMANDS.")); 
                 #endif
                 fona.HTTP_GET_end();
                 return;
               }
              
               char buffer[3];
               buffer[0] = charResultIn[1];
               buffer[1] = charResultIn[2];
               buffer[2] = '\0';
 
               commandsLength = atoi(buffer);
               startCommandIndex = charIndex;
               commandsIndex++;
            }
            else if ((charIndex - startCommandIndex) == 2)
            {
               char buffer[3];
               buffer[0] = charResultIn[startCommandIndex];
               buffer[1] = charIn;
               buffer[2] = '\0';
               commandKind = atoi(buffer);
               feedingValue = true;
            }
            else if (charIn == '-')
            {
              int commandValueLength = commandValuein.length();
              char caValue[commandValueLength]; 
              commandValuein.toCharArray(caValue, commandValueLength + 1);
              commands[commandsIndex].kind = commandKind;
              commands[commandsIndex].value = atol(caValue);
              commandsIndex++;    
              feedingValue = false;
              commandValuein = "";
              commandKind = -1;
              startCommandIndex = charIndex;
            }
            else if (feedingValue)
            {
              commandValuein += charIn;
            }
            length--;
            if (! length) break;
          }
        }
        fona.HTTP_GET_end();
        // ================================

#ifdef WithSERIALPRINT
   Serial.print(F("commandsLength: ")); Serial.println(commandsLength);
#endif

  delay(50);
  
  for (int cmdindex = 0; cmdindex < commandsLength; cmdindex++)
  {
      #ifdef WithSERIALPRINT 
          Serial.print(F("sendData: value=")); Serial.print(commands[cmdindex].value); Serial.print(F(", kind=")); Serial.println(commands[cmdindex].kind);
      #endif
      if (commands[cmdindex].kind < 90)
        runCommando(commands[cmdindex].value, commands[cmdindex].kind);
      else 
      {
        switch (commands[cmdindex].kind)
        {
          case 90:
            turnOffGSM(); 
            break;
        }
      }
        
      delay(50);
  }
}

void updateSignalLevel()
{
  #ifdef WithSERIALPRINT
   Serial.println("\nMOILEDATA =========================");
  #endif
  
  SignalLevel = fona.getRSSI();
  //SignalLevel = String(rssi);
  #ifdef WithSERIALPRINT  
    Serial.print("Signal level: "); Serial.println(SignalLevel);
  #endif
}

void updateFonaGPS()
{
  #ifdef WithSERIALPRINT
   Serial.println("\nGPS =========================");
  #endif

  String _Latitude,_Longitude,_GPSAltitude,_GPSsatellitesUsed;

    char gpsdata[120];
    fona.getGPS(0, gpsdata, 120);

    char used_satellites;
    uint8_t gpsdataCommaIndex = 0;
    for (uint8_t gpsdataIndex; gpsdataIndex < 120; gpsdataIndex++)
    {
      if (gpsdata[gpsdataIndex] == ',')
        gpsdataCommaIndex++;
      else
      {
        switch (gpsdataCommaIndex)
        {
          case 3:
            _Latitude += gpsdata[gpsdataIndex];
            break;
          case 4:
            _Longitude += gpsdata[gpsdataIndex];
            break;
          case 5:
            _GPSAltitude += gpsdata[gpsdataIndex];
            break;
          case 15:
            _GPSsatellitesUsed = gpsdata[gpsdataIndex];
            if (_GPSsatellitesUsed != "0")
            {
               Latitude = _Latitude.toFloat();
               Longitude = _Longitude.toFloat();
               GPSAltitude = _GPSAltitude.toFloat();
               GPSsatellitesUsed = _GPSsatellitesUsed.toFloat();
               return;
            }
        }
      }
    }
  
    uint16_t returncode;
    if (!fona.getGSMLoc(&returncode, replybuffer, 250))
    {
      #ifdef WithSERIALPRINT
        Serial.println(F("Failed!"));
      #endif          
    }
    String gpsString = replybuffer;
    if (returncode == 0) 
    {
      int firstComma = gpsString.indexOf(',');

      if (firstComma != -1)
      {
        _Longitude = gpsString.substring(0,firstComma);
        firstComma++;
        int secoundComma = gpsString.indexOf(',',firstComma);
        if (secoundComma != -1)
        {
          _Latitude = gpsString.substring(firstComma,secoundComma);
        }
      }
      Latitude = _Latitude.toFloat();
      Longitude = _Longitude.toFloat();
    }
#ifdef WithSERIALPRINT      
    Serial.print(F("Latitude: ")); Serial.println(Latitude);
    Serial.print(F("Longitude: ")); Serial.println(Longitude);
    Serial.println("");
#endif      
}

void updateFonaBatery()
{
  uint16_t vbat;
  if (! fona.getBattPercent(&vbat)) 
  {
    #ifdef WithSERIALPRINT      
      Serial.println(F("Failed to read Batt"));
    #endif
  } 
  else 
  {
      SimBatteryLevel = vbat;
    #ifdef WithSERIALPRINT      
      Serial.println("\nBATTERY =========================");
      Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
    #endif
  }
}

void updateDroneBatteryLevel()
{
  DroneBatteryVolt = readVcc();
}

String getUrl()
{
  String returnUrl = url;
  returnUrl += "&GPSCoordinator.Latitude="; returnUrl += String(Latitude);
  returnUrl += "&GPSCoordinator.Longitude="; returnUrl += String(Longitude);
  returnUrl += "&Altitude="; returnUrl += String(totalAltitude);
  returnUrl += "&SignalLevel="; returnUrl += String(SignalLevel);
  returnUrl += "&SimBatteryLevel="; returnUrl += String(SimBatteryLevel);
  returnUrl += "&DroneBatteryVolt="; returnUrl += String(DroneBatteryVolt);
  return returnUrl;  
}

