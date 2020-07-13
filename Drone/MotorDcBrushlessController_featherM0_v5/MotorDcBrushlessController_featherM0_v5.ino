/*
  Wille Esteche

  // In 1080 and 1090 always staqrt again after it got hang.

*/
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

#define WithSERIALPRIINT
#define WithGYROSCOPE

//#define WithLOGGS

#define VBATPIN A7

// DataLogger
#ifdef WithLOGGS
  const int chipSelect = 4;
  bool foundSDcard = false;
 
  bool isDatalogOn = true;
  bool isSerialPortOn = false;
  
#endif

//Relay
#define PIN_RELAY 3

// Motors
#define ESC_HIGH_DEFAULT 1000
#define ESC_LOW_DEFAULT 20

//Servo escs[4];  // create servo object to control a servo  
//bool isInSettingMode = false;

/*
bool rinding = false;
bool isRideProcessing = false;
*/
uint8_t requestId = 9;
float totalAltitude = 0;
float MotorBalanceCoefficient = 0.1;

typedef struct 
{
  float currentSpeed = ESC_LOW_DEFAULT;
  float toSpeed = ESC_LOW_DEFAULT;
  float balanceRoll = 0;
  float balancePitch = 0;
  void setSpeed(int newSpeed)
  {
    if (newSpeed > 0)
    {
      int speed = newSpeed + balancePitch + balanceRoll;
      int angle = map(speed, ESC_LOW_DEFAULT, ESC_HIGH_DEFAULT, 0, 360);
      esc.write(angle);
      delay(2);
    }
  };
  Servo esc;
} Motor;

Motor motors[4];

// GyroScope
#ifdef WithGYROSCOPE
  #include <SparkFunLSM9DS1.h>
  LSM9DS1 imu;

  bool rinding = false;
  bool isRideProcessing = false;
  bool gyroActive = false;
  
  float VerticalAcceptanceMarginal = 1;
  float PlanningAxisAcceptanceMarginal = 1;
  
  #define DECLINATION 6.34 // Declination (degrees) in Boulder, CO.
  #define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
  #define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

  
  typedef struct AxisCommandos
  {
    float Offset = 0;
    float WishedValue = 0;

    float maxValue = 0;
    float minValue = 0;
    float accResult = 0;
    float loopValue = 0;
    float currentValue = 0;
    float lastDiffSent = 0;
    float diff()
    {
      return (WishedValue + Offset) - currentValue;
    }    
  };
  
  bool withoutOffsets = false;
  
  AxisCommandos RollCommandos,
                PitchCommandos,
                YawCommandos;
  //              VerticalCommandos,
  
#endif



///SETUP///
void setup() 
{

#ifdef WithSERIALPRIINT
  
  while (!Serial);
   
  Serial.begin(115200);
  Serial.println(F("M0 basic test"));
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#ifdef WithLOGGS
  foundSDcard = SD.begin(chipSelect);
  engraveGyroHeaders();
#endif
  
  motors[0].esc.attach(4);  // attaches the servo on pin 4 to the motor FrontLeft
  motors[1].esc.attach(5);  // attaches the servo on pin 5 to the motor FrontRight
  motors[2].esc.attach(6);  // attaches the servo on pin 6 to the motor BackRight
  motors[3].esc.attach(7);  // attaches the servo on pin 7 to the motor BackLeft

  delay(100);

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
    motors[motorIndex].setSpeed(motors[motorIndex].currentSpeed);

  delay(100);

  Wire.begin();

#ifdef WithLOGGS
  logtext("Motors initiated",true);
#endif

#ifdef WithSERIALPRIINT
  Serial.println("Motors initiated");
#endif

uint8_t tryingTimes = 0;

#ifdef WithLOGGS
  logtext("Ready to start.", true);
#endif
  
#ifdef WithGYROSCOPE 

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  gyroActive = imu.begin();
  Serial.println("HERE 03");
  //ride(true);
 
 #ifdef WithSERIALPRIINT
  Serial.println("Gyroscope ON");
 #endif
#endif

}

bool isCheching = false;
int8_t looptimes = 0;

///LOOP///
void loop() 
{
  updateCleanerGyroValues();
  ride(rinding);
}

#ifdef WithLOGGS
void engrave(String data, bool isGyro, bool withNewLine)
{
  if (data == "")
    return;

  String fileName = isGyro ? "gyrolog.csv" : "datalogger.txt";
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    if (withNewLine)
      dataFile.println(data);
    else
      dataFile.print(data);
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else { 
    Serial.println("error opening " + fileName);
  }
}

void engraveGyroHeaders()
{
  engrave("Pitch; Roll; Heading;", true, true);
}

void engraveGyroMetrics()
{
  String dataString = "";
 
  dataString += String(PitchMessures.loopValue);
  dataString += ";";
  dataString += String(PitchMessures.loopValue);
  dataString += ";";                 
  dataString += String(YawMessures.loopValue);
  dataString += ";";                 

  if (dataString != "")
    engrave(dataString, true,true);
}
#endif

void SetMortosToZero()
{
  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].toSpeed = motors[motorIndex].currentSpeed = ESC_LOW_DEFAULT;
    motors[motorIndex].setSpeed(motors[motorIndex].currentSpeed);
  }
}

 
void getCommando (uint32_t commando)
{

    uint32_t parameter = (commando / 10);
    uint8_t processId = commando - (parameter * 10);
    
    long processParameter = (parameter / 10);
    uint8_t plusminus = parameter - (processParameter * 10);
    if (plusminus == 2)
      processParameter *= -1;
    /*
    #ifdef WithLOGGS
      logtext("processParameter: "); logtext(String(processParameter), true);
      logtext("processId: "); logtext(String(processId), true);
    #endif  
    #ifdef WithSERIALPRIINT  
      Serial.print(F("processParameter: "));Serial.println(processParameter);
      Serial.print(F("processId: "));Serial.println(processId);
    #endif
    */
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
        #ifdef WithLOGGS
          logtext("!!Process 3!! ", true);
        #endif
        break;
      }
      case 4:
      {
       requestId = 4; 
         #ifdef WithLOGGS
          logtext("!!Process 4!! ", true);
        #endif        
        break;
      }
      case 5:
      {
        turnOnOffMotors(processParameter);
        #ifdef WithLOGGS
          logtext("!!Process 5!! ", true);
        #endif
        break;
      }        
      case 6: //NEW BalanceCoefficient ARRIVED times 100
      {
        MotorBalanceCoefficient = processParameter / 100;
        #ifdef WithLOGGS
          logtext("!!Process 6!! ", true);
        #endif        
        break;
      }
      case 7: //NEW ROLL ARRIVED
      {
        float rollDiff = processParameter / 100;
        float speedToChange = rollDiff * MotorBalanceCoefficient;
        if (rollDiff < 0)
        {
          speedToChange *= -1;
          Motors_BalancePitch(speedToChange, true);
        }
        else
          Motors_BalancePitch(speedToChange, false);
          
        #ifdef WithLOGGS
          logtext("!!Process 7!! ", true);
        #endif        
        break;
      }
      case 8: //NEW PITCH ARRIVED
      {
        float picthDiff = processParameter / 100;
        float speedToChange = picthDiff * MotorBalanceCoefficient;
        if (picthDiff < 0)
        {
          speedToChange *= -1;
          Motors_BalanceRoll(speedToChange, true);
        }
        else
          Motors_BalanceRoll(speedToChange, false);
        
        #ifdef WithLOGGS
          logtext("!!Process 8!! ", true);
        #endif  
        break;
      }
      case 9:
      {
        requestId = 9;
        #ifdef WithLOGGS
          logtext("!!Process 9!! ", true);
        #endif        
        break;
      }
    }        
}


bool signalOn = true;
void getASignal()
{
   #ifdef WithLOGGS
      logtext("getASignal: ", true);
   #endif 

  if (signalOn = !signalOn)
    digitalWrite(LED_BUILTIN, LOW);
  else 
    digitalWrite(LED_BUILTIN, HIGH);
  
}




void turnOnOffMotors(int OnOff)
{
  if (OnOff == 1)
    {
      digitalWrite(PIN_RELAY, HIGH);
#ifdef WithLOGGS
      logtext("Motors ON",true);
#endif      
    }
    else
    {
      SetMortosToZero();
      digitalWrite(PIN_RELAY, LOW);
#ifdef WithLOGGS
      logtext("Motors OFF",true);
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

 #ifdef WithSERIALPRIINT
  Serial.print("Pi
  '¨ äåb¨äöbpövgpopnsString: ");Serial.print(motrosStringPins);
  Serial.print(". Value to add: ");Serial.println(newSpeed);
#endif
#ifdef WithLOGGS
    logtext("PinsString: ");
    logtext(motrosStringPins);
    logtext(". Value to add: ");
    logtext(String(newSpeed),true);
#endif

    int8_t stepsSppeding = 0;
    int8_t stepsSppeded = 0;
    for (int index = 0; index < 4; index++)
    {
#ifdef WithLOGGS      
      logtext("Loop string part: ");
      logtext(motrosStringPins.substring(index, index + 1),true);
#endif    
#ifdef WithSERIALPRIINT
  Serial.print("Loop string part: ");Serial.println(motrosStringPins.substring(index, index + 1));
#endif

      if (motrosStringPins.substring(index, index + 1) == "1")
      {
        if (motors[index].currentSpeed + newSpeed < ESC_LOW_DEFAULT)
          motors[index].toSpeed = ESC_LOW_DEFAULT;
        else if (motors[index].currentSpeed + newSpeed > ESC_HIGH_DEFAULT)
          motors[index].toSpeed = ESC_HIGH_DEFAULT;
        else
          motors[index].toSpeed += newSpeed;

        stepsSppeding = motors[index].toSpeed - motors[index].currentSpeed;

#ifdef WithLOGGS     
        logtext("Motor: "); logtext(String(index));
        logtext(" move to "); logtext(String(motors[index].currentSpeed), true);
#endif  
      }
    }

    int stepInterval = 2;

    bool allMotorsUpdated;
    do
    {
      allMotorsUpdated = true;
      for (int index = 0; index < 4; index++)
      {
        if (motrosStringPins.substring(index, index + 1) == "1")
        {
          if (motors[index].currentSpeed > motors[index].toSpeed)
          {
            if (motors[index].currentSpeed - stepInterval < motors[index].toSpeed)
            {
               motors[index].currentSpeed = motors[index].toSpeed;
            }
            else
            {
              motors[index].currentSpeed -= stepInterval;
              allMotorsUpdated = false;
            }            
            motors[index].setSpeed(motors[index].currentSpeed);
          }
          else if (motors[index].currentSpeed < motors[index].toSpeed)
          {
            if (motors[index].currentSpeed + stepInterval > motors[index].toSpeed)
            {
              motors[index].currentSpeed = motors[index].toSpeed;
            }
            else
            {
              motors[index].currentSpeed += stepInterval;
              allMotorsUpdated = false;
            }
            motors[index].setSpeed(motors[index].currentSpeed);
          }

          if (motors[index].currentSpeed != motors[index].toSpeed)
            allMotorsUpdated = false;
        }
      }
    } while (!allMotorsUpdated);

    
  }
}

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

#ifdef WithLOGGS
        logtext("Motor: ");
        logtext(String(index));
        logtext(" upset to ");
        logtext(String(motors[index].currentSpeed),true);
#endif        
      
    }
  }
}

float readVcc() {
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  
  measuredvbat -= 3277;  
  measuredvbat *= 100;
  measuredvbat /= 1023;
  
  // MAX 4300
  // MIN 3277
  // DiffInterval 1023
  
  return measuredvbat;  
}

void Motors_BalanceRoll(int8_t addingSpeed, bool toLeft)
{
  
#ifdef WithSERIALPRIINT
  Serial.print("Motors_BalanceRoll toLeft: ");Serial.println(toLeft);
  Serial.print("Motors_BalanceRoll addingSpeed: ");Serial.println(addingSpeed);
#endif

  addingSpeed /= 2;

  float multiplicateToSides = toLeft ? 1 : -1;

  if ((motors[0].balanceRoll == addingSpeed * multiplicateToSides))
    return;

  motors[0].balanceRoll = addingSpeed * multiplicateToSides;
  motors[1].balanceRoll = -addingSpeed * multiplicateToSides;
  motors[2].balanceRoll = -addingSpeed * multiplicateToSides;
  motors[3].balanceRoll = addingSpeed * multiplicateToSides;

  ReloadSpeedOnMotors();


#ifdef WithLOGGS
  logtext("BalanceRoll: ");
  logtext(String(addingSpeed));
  logtext(", isToLeft: ");
  logtext(String(toLeft),true);
#endif  
 
}

void Motors_BalancePitch(int8_t addingSpeed, bool toFront)
{
  
#ifdef WithSERIALPRIINT
  Serial.print("Motors_BalanceRoll toFront: ");Serial.println(toFront);
  Serial.print("Motors_BalanceRoll addingSpeed: ");Serial.println(addingSpeed);
#endif

  addingSpeed /= 2;

  float multiplicateToSides = toFront ? 1 : -1;

  if ((motors[0].balancePitch == addingSpeed * multiplicateToSides))
    return;

  motors[0].balancePitch = addingSpeed * multiplicateToSides;
  motors[1].balancePitch = addingSpeed * multiplicateToSides;
  motors[2].balancePitch = -addingSpeed * multiplicateToSides;
  motors[3].balancePitch = -addingSpeed * multiplicateToSides;

  ReloadSpeedOnMotors();

#ifdef WithLOGGS
  logtext("BalancePitch: ");
  logtext(String(addingSpeed));
  logtext(", isToFront: ");
  logtext(String(toFront), true);
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

#ifdef WithLOGGS
void logtext(String text, bool withNewLine)
{
  if (isDatalogOn)
  {
    engrave(text, false, withNewLine);
  }
  if (isSerialPortOn)
  {
    if (withNewLine)
      Serial.println(text);
    else
      Serial.print(text);
  }
}

void logtext(String text)
{
  if (isDatalogOn)
  {
     engrave(text, false, false);
  }

  if (isSerialPortOn)
  {
      Serial.print(text);
  }
}
#endif


// ======== RIDING =========================
#ifdef WithGYROSCOPE

void updateGyroBasicValues()
{
  /*
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  */
  Serial.print("HERE1");
  if ( imu.accelAvailable() && imu.magAvailable() )
  {
    Serial.print("HERE2");

    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    
    float ax = imu.ax;
    float ay = imu.ay;
    float az = imu.az;
    float mz = imu.mz;
    float my = -imu.mx;
    float mx = -imu.my;
 
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
    float heading;
    if (my == 0)
      heading = (mx < 0) ? PI : 0;
    else
      heading = atan2(mx, my);
      
    heading -= DECLINATION * PI / 180;
    
    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);
    
    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    RollCommandos.loopValue = roll;
    PitchCommandos.loopValue = pitch;
    YawCommandos.loopValue = heading;
  }
}

void updateCleanerGyroValues()
{
  int8_t bestOf = 3;

  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {
    updateGyroBasicValues();

    if (counterIndex == 0)
      RollCommandos.maxValue = RollCommandos.minValue = RollCommandos.loopValue;
    else if (RollCommandos.loopValue > RollCommandos.maxValue)
      RollCommandos.maxValue = RollCommandos.loopValue;
    else if (RollCommandos.loopValue < RollCommandos.minValue)
      RollCommandos.minValue = RollCommandos.loopValue;

    if (counterIndex == 0)
      PitchCommandos.maxValue = PitchCommandos.minValue = PitchCommandos.loopValue;
    else if (PitchCommandos.loopValue > PitchCommandos.maxValue)
      PitchCommandos.maxValue = PitchCommandos.loopValue;
    else if (PitchCommandos.loopValue < PitchCommandos.minValue)
      PitchCommandos.minValue = PitchCommandos.loopValue;

    if (counterIndex == 0)
      YawCommandos.maxValue = YawCommandos.minValue = YawCommandos.loopValue;
    else if (YawCommandos.loopValue > YawCommandos.maxValue)
      YawCommandos.maxValue = YawCommandos.loopValue;
    else if (YawCommandos.loopValue < YawCommandos.minValue)
      YawCommandos.minValue = YawCommandos.loopValue;

//    if (counterIndex == 0)
//      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
//      VerticalMessures.maxValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
//      VerticalMessures.minValue = VerticalMessures.loopValue;

  RollCommandos.currentValue = ((int)(((RollCommandos.accResult - RollCommandos.maxValue - RollCommandos.minValue) / (bestOf - 2)) * 100)) / 100.0;
  PitchCommandos.currentValue = ((int)(((PitchCommandos.accResult - PitchCommandos.maxValue - PitchCommandos.minValue) / (bestOf - 2)) * 100)) / 100.0;
  YawCommandos.currentValue = ((int)(((YawCommandos.accResult - YawCommandos.maxValue - YawCommandos.minValue) / (bestOf - 2)) * 100)) / 100.0;  
//  VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;

  }
}

void ride(bool onlyOffset)
{
  isRideProcessing = true;

  updateCleanerGyroValues();
  
  if (onlyOffset)
  {
    PitchCommandos.Offset = PitchCommandos.currentValue * -1;
    RollCommandos.Offset = RollCommandos.currentValue * -1;
    YawCommandos.Offset = YawCommandos.currentValue * -1;
//    VerticalCommandos.Offset = VerticalMessures.currentValue * -1;

    isRideProcessing = false;
    return;
  }
  
  if (!withoutOffsets)
  {    
    PitchCommandos.currentValue += PitchCommandos.Offset;
    RollCommandos.currentValue += RollCommandos.Offset;
    YawCommandos.currentValue += YawCommandos.Offset;
    //    VerticalMessures.currentValue += VerticalCommandos.Offset;
  }

/*
#ifdef WithSERIALPRIINT
Serial.print("PitchCommandos.currentValue: ");Serial.print(PitchCommandos.currentValue);
Serial.print("; RollCommandos.currentValue: ");Serial.print(RollCommandos.currentValue);
Serial.print("; YawCommandos.currentValue: ");Serial.println(YawCommandos.currentValue);
Serial.print("PitchCommandos.diff(): ");Serial.println(PitchCommandos.diff());
Serial.print("RollCommandos.diff(): ");Serial.println(RollCommandos.diff());
//Serial.print("PlanningAxisAcceptanceMarginal: ");Serial.println(PlanningAxisAcceptanceMarginal);
Serial.print("PitchCommandos.diff() - PitchCommandos.lastDiffSent: ");Serial.println(PitchCommandos.diff() - PitchCommandos.lastDiffSent);
Serial.print("RollCommandos.diff() - RollCommandos.lastDiffSent: ");Serial.println(RollCommandos.diff() - RollCommandos.lastDiffSent);
#endif
*/


if (((PitchCommandos.diff() - PitchCommandos.lastDiffSent) > PlanningAxisAcceptanceMarginal) || (PitchCommandos.diff() - PitchCommandos.lastDiffSent) < PlanningAxisAcceptanceMarginal * -1)
{
  PitchCommandos.lastDiffSent = PitchCommandos.diff();
  //sendData(PitchCommandos.diff() * 100,7);  
}

if (((RollCommandos.diff() - RollCommandos.lastDiffSent) > PlanningAxisAcceptanceMarginal) || (RollCommandos.diff() - RollCommandos.lastDiffSent) < PlanningAxisAcceptanceMarginal * -1)
{
  RollCommandos.lastDiffSent = RollCommandos.diff();
  //sendData(RollCommandos.diff() * 100,8);  
}


/*
  if (PitchCommandos.diff() > PlanningAxisAcceptanceMarginal ||  PitchCommandos.diff() < PlanningAxisAcceptanceMarginal * -1)
  {
    #ifdef WithSERIALPRIINT    
      //Serial.print(F("PitchCommandos.diff()"));Serial.println(PitchCommandos.diff());
    #endif
    sendData(PitchCommandos.diff() * 100,7);
  }

  if (RollCommandos.diff() > PlanningAxisAcceptanceMarginal ||  RollCommandos.diff() < PlanningAxisAcceptanceMarginal * -1)
  {
    #ifdef WithSERIALPRIINT    
      //Serial.print(F("RollCommandos.diff()"));Serial.println(RollCommandos.diff());
      Serial.print(F("RollCommandos.diff()"));Serial.println(RollCommandos.diff()* 100);
    #endif
    sendData(RollCommandos.diff() * 100,8);
  }
*/

  /*
  if (YawCommandos.diff() > PlanningAxisAcceptanceMarginal ||  YawCommandos.diff() < PlanningAxisAcceptanceMarginal * -1)
  {    
  }
  */
  
  isRideProcessing = false;
}

void SendGyro()
{
    postGyro = false;
    
    uint16_t statuscode1;
    int16_t length1;
   
    String entireUrl = String(url) + "/gyro?id=1";

    entireUrl += "&pitch="; entireUrl += String(PitchCommandos.currentValue);
    entireUrl += "&roll="; entireUrl += String(RollCommandos.currentValue);
    entireUrl += "&yaw="; entireUrl += String(YawCommandos.currentValue);
      
    int urlLength = entireUrl.length();
    char urlChars1[urlLength];
    entireUrl.toCharArray(urlChars1, urlLength + 1);  

#ifdef WithSERIALPRIINT
  Serial.print("GET GYRO: ");  Serial.println(entireUrl);
#endif

  if (!fona.HTTP_GET_start(urlChars1, &statuscode1, (uint16_t *)&length1)) 
  {
    #ifdef WithSERIALPRIINT
      Serial.print("GYRO SEND. Failed! "); Serial.println(statuscode1); 
    #endif
  }
  else
  {
    fona.HTTP_GET_end();
  }
        
}

#endif

String requestData()
{
  Wire.requestFrom(8, 8);    // request 6 bytes from slave device #8
  
  char receiving[8];
  int index =0;
  while (Wire.available()) { // slave may send less than requested
    int charIndex = Wire.read();
    receiving[index] = (charIndex == 255) ? 0 : receiving[index] = charIndex;
    index++;
  }
    
  #ifdef WithSERIALPRIINT      
      Serial.print(F("Arrived: ")); Serial.println(receiving); 
  #endif

  return String(receiving);
  
}

void sendData(long value, uint16_t pcoessId)
{
  uint8_t plusminus = 1;

  if (value < 0)
  { 
    plusminus = 2;
    value *= -1;
  }

  //uint32_t commando = (((value * 10) + pcoessId) * 10) + sendingId;
  //uint32_t commando = ((value * 10) + pcoessId);
  uint32_t commando = (((value * 10) + plusminus) * 10) + pcoessId;

  byte toSend[4];
  toSend[0] = (commando >> 24) & 0xFF;
  toSend[1] = (commando >> 16) & 0xFF;
  toSend[2] = (commando >> 8) & 0xFF;
  toSend[3] = commando & 0xFF;

  Wire.beginTransmission(6); // transmit to device #8
  Wire.write(toSend, 4);              // sends one byte
  Wire.endTransmission();    // stop transmitting

#ifdef WithSERIALPRIINT    
  Serial.print(F("ValueToSend: "));Serial.println(value);
  Serial.print(F("PcoessIdToSend: "));Serial.println(pcoessId);
  Serial.print(F("Sending: "));Serial.println(commando);
#endif

}
