#include <Wire.h>
#include "Adafruit_FONA.h"

//#define WithPINCode
//#define WithAIRPRESSURE
#define WithGYROSCOPE
#define WithSERIALPRIINT
 
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

char replybuffer[255];
const char* url = "http://droneapiv1.azurewebsites.net";

String Longitude = "-1";
String Latitude = "-1";
String Altitude = "-1";
String SignalLevel = "-1";
String SimBatteryLevel = "-1";
String DroneBatteryVolt = "-1";

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

bool isFonaOn = false,
     sendGps = false,
     manuallyTurnedOff = false,
     postGyro = false;
     
uint8_t gsmFailed = 0;
int8_t MotorDifferenceMarginal = 6;

struct CommandoDef
{
  int kind;
  uint32_t value;
};

// GyroScope
#ifdef WithGYROSCOPE
  #include <SparkFunLSM9DS1.h>
  LSM9DS1 imu;

  bool rinding = false;
  bool isRideProcessing = false;
  bool gyroActive = false;

  #define DECLINATION 6.34 // Declination (degrees) in Boulder, CO.
  #define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
  #define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

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
                YawCommandos;
  //              VerticalCommandos,
  //              SideCommandos;

  AvgMessures RollMessures,
              PitchMessures,
              YawMessures;
//              SideMessures,
//              VerticalMessures;

  
#endif

bool isCheching = false;
int looptimes = 0;
int8_t gyrolooptimes = 0;
int8_t loopproccess = 0;

#ifdef WithPINCode
  char PIN[4] = {'8','8','6','8' };
#endif

//=====SETUP=====//
void setup() {
  
   //while (!Serial);

#ifdef WithSERIALPRIINT
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));
#endif

  Wire.begin();
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
#ifdef WithSERIALPRIINT    
    Serial.println(F("Couldn't find FONA"));
#endif
    while (1);
  }
#ifdef WithSERIALPRIINT  
  Serial.println(F("FONA found!!"));
  Serial.println("Ready to start.");
#endif  


#ifdef WithGYROSCOPE 

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  gyroActive = imu.begin();
  //ride(true);
 
 #ifdef WithSERIALPRIINT
  Serial.println("Gyroscope ON");
 #endif
#endif

}

//=====LOOP=====//
void loop() {

  if (manuallyTurnedOff)
  {
    delay(60000);
    return;
  }
  
  if (!isFonaOn)
  {
    initFona();
    delay(500);
  }
  else
  {
    if (looptimes > 300)
    {
      switch(loopproccess)
      {
        case 0:
          if (sendGps)
            updateFonaGPS();
            
          break;
        case 1:
          if (postGyro)
          {
             updateCleanerGyroValues();
             SendGyro();
          }
            
          updateFonaBatery();
          break;
        case 2:
          updateSignalLevel();
          break;
        case 3:
          getDroneBatteryLevel();
          break;
        case 4:
          getDroneAltitude();
          break;
        case 5:
          getHttp();
          loopproccess = -1;
          break;
      }
      looptimes = 0;
      loopproccess++;
    }
    else looptimes++;
 
    #ifdef WithGYROSCOPE
    if (gyrolooptimes == 20)
    {
      if (rinding && !isRideProcessing)
        ride(false);

      gyrolooptimes = 0;
    }
    gyrolooptimes++;
    #endif
  }
}

void initFona()
{

#ifdef WithPINCode
  bool unlocked = fona.unlockSIM(PIN);
   #ifdef WithSERIALPRIINT
   if (unlocked)
    Serial.println(F("UNLOCKED!"));
   else
    Serial.println(F("Unlocking Failed"));
   #endif
#endif
  
    uint8_t tryTimes = 0;
    isFonaOn = fona.enableGPRS(true);
    while (!isFonaOn)
    {
      if (tryTimes == 10)
      {
    #ifdef WithSERIALPRIINT        
        Serial.println(F("Failed when trying to Enable GPRS."));
    #endif
        break;
      }
        
      tryTimes++;
      isFonaOn = fona.enableGPRS(true);
      delay(1000);
    }
#ifdef WithSERIALPRIINT    
    Serial.println(F("GPRS Enabled"));
#endif    

  if (isFonaOn)
  {
    updateFonaGPS();
    //delay(200);
  }
}

void turnOffGSM()
{
    uint8_t tryTimes = 0;
    manuallyTurnedOff = fona.enableGPRS(false);
    while (!manuallyTurnedOff)
    {
      if (tryTimes == 10)
      {
    #ifdef WithSERIALPRIINT
        Serial.println(F("Failed when trying to Disable GPRS."));
    #endif
        break;
      }

      tryTimes++;
      manuallyTurnedOff = fona.enableGPRS(false);
      delay(1000);
    }
#ifdef WithSERIALPRIINT
    Serial.println(F("GPRS Disabled"));
#endif
}

void getHttp()
{
   //GPRS Web =========================
   
      uint16_t statuscode;
      int16_t length;

      String entireUrl = String(url) + "/data?id=1";

      entireUrl += "&alt="; entireUrl += Altitude;
      entireUrl += "&sgnl="; entireUrl += SignalLevel;
      entireUrl += "&sbty="; entireUrl += SimBatteryLevel;
      entireUrl += "&dbty="; entireUrl += DroneBatteryVolt;
      entireUrl += "&lat="; entireUrl += Latitude;
      entireUrl += "&lon="; entireUrl += Longitude;
            
      int urlLength = entireUrl.length();
      char urlChars[urlLength];
      entireUrl.toCharArray(urlChars, urlLength + 1);        

#ifdef WithSERIALPRIINT          
  Serial.println("\nGPRS =========================");
  Serial.print("GET: "); Serial.println(urlChars);
#endif          

        if (!fona.HTTP_GET_start(urlChars, &statuscode, (uint16_t *)&length)) {
  #ifdef WithSERIALPRIINT          
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
              sendData(0,5);
              turnOffGSM();
              break;
          }
          //delay(1000);
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
            
            char charIn = fona.read();
            charResultIn[charIndex++] = charIn;

            if (charIndex == 1)
            {
              if (charIn != 's')
              {
                #ifdef WithSERIALPRIINT
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
                 #ifdef WithSERIALPRIINT
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

#ifdef WithSERIALPRIINT
   Serial.print(F("commandsLength: ")); Serial.println(commandsLength);
#endif

  //delay(50);
  
  for (int cmdindex = 0; cmdindex < commandsLength; cmdindex++)
  {
      #ifdef WithSERIALPRIINT 
          Serial.print(F("sendData: value=")); Serial.print(commands[cmdindex].value); Serial.print(F(", kind=")); Serial.println(commands[cmdindex].kind);
      #endif
      if (commands[cmdindex].kind < 90)
        sendData(commands[cmdindex].value, commands[cmdindex].kind);
      else 
      {
        switch (commands[cmdindex].kind)
        {
          case 90:
            turnOffGSM(); 
            break;
          case 91:
            sendGps = (commands[cmdindex].value == 1);            
            break;
         #ifdef WithGYROSCOPE
          case 92:
            rinding = (commands[cmdindex].value == 1);
            break;
          case 93:
            MotorDifferenceMarginal = commands[cmdindex].value;
            break;
          case 94:
            withoutOffsets = (commands[cmdindex].value == 1);
            break;
          case 95:
            postGyro = true;
            break;
         #endif
        }
      }
        
      //delay(50);
  }
}

void updateSignalLevel()
{
  #ifdef WithSERIALPRIINT
   Serial.println("\nMOILEDATA =========================");
  #endif
  
  uint8_t rssi = fona.getRSSI();
  SignalLevel = String(rssi);
  #ifdef WithSERIALPRIINT  
    Serial.print("Signal level: "); Serial.println(SignalLevel);
  #endif
}

void updateFonaGPS()
{
  #ifdef WithSERIALPRIINT
   Serial.println("\nGPS =========================");
  #endif
  
    uint16_t returncode;
    if (!fona.getGSMLoc(&returncode, replybuffer, 250))
    {
      #ifdef WithSERIALPRIINT
        Serial.println(F("Failed!"));
      #endif          
    }
    String gpsString = replybuffer;
    if (returncode == 0) 
    {
      int firstComma = gpsString.indexOf(',');

      if (firstComma != -1)
      {
        Longitude = gpsString.substring(0,firstComma);
        firstComma++;
        int secoundComma = gpsString.indexOf(',',firstComma);
        if (secoundComma != -1)
        {
          Latitude = gpsString.substring(firstComma,secoundComma);
        }
      }
    }
#ifdef WithSERIALPRIINT      
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
    #ifdef WithSERIALPRIINT      
      Serial.println(F("Failed to read Batt"));
    #endif
  } 
  else 
  {
      SimBatteryLevel = vbat;
    #ifdef WithSERIALPRIINT      
      Serial.println("\nBATTERY =========================");
      Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
    #endif
  }
}

void getDroneBatteryLevel()
{
  sendData(0, 9);
  //delay(100);
  //DroneBatteryVolt = 4505; 
  DroneBatteryVolt = requestData();
  DroneBatteryVolt.trim();
  
  if (DroneBatteryVolt == "")
    DroneBatteryVolt = -1;

  #ifdef WithSERIALPRIINT      
    Serial.print(F("DroneBatteryVolt: ")); Serial.println(DroneBatteryVolt);
  #endif  
}

void getDroneAltitude()
{
  sendData(0,4);
  //delay(100);
  //Altitude = 25; 
  Altitude = String(requestData().toInt());
    
  if (Altitude == "")
    Altitude = -1;
    
  #ifdef WithSERIALPRIINT 
    Serial.print(F("Altitude: ")); Serial.println(Altitude);
  #endif  
}

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


void sendData(uint32_t value, uint16_t pcoessId)
{
  //uint32_t commando = (((value * 10) + pcoessId) * 10) + sendingId;
  uint32_t commando = ((value * 10) + pcoessId);

  byte toSend[4];
  toSend[0] = (commando >> 24) & 0xFF;
  toSend[1] = (commando >> 16) & 0xFF;
  toSend[2] = (commando >> 8) & 0xFF;
  toSend[3] = commando & 0xFF;

  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(toSend, 4);              // sends one byte
  Wire.endTransmission();    // stop transmitting

#ifdef WithSERIALPRIINT 
  Serial.print(F("Sending: "));Serial.println(commando);
#endif

}

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
  
  if ( imu.accelAvailable() && imu.magAvailable() )
  {
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

    RollMessures.loopValue = roll;
    PitchMessures.loopValue = pitch;
    YawMessures.loopValue = heading;
  }
}

void updateCleanerGyroValues()
{
  int8_t bestOf = 5;

  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {
    updateGyroBasicValues();

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
      YawMessures.maxValue = YawMessures.minValue = YawMessures.loopValue;
    else if (YawMessures.loopValue > YawMessures.maxValue)
      YawMessures.maxValue = YawMessures.loopValue;
    else if (YawMessures.loopValue < YawMessures.minValue)
      YawMessures.minValue = YawMessures.loopValue;

//    if (counterIndex == 0)
//      SideMessures.maxValue = SideMessures.minValue = SideMessures.loopValue;
//    else if (SideMessures.loopValue > SideMessures.maxValue)
//      SideMessures.maxValue = SideMessures.loopValue;
//    else if (SideMessures.loopValue < SideMessures.minValue)
//      SideMessures.minValue = SideMessures.loopValue;

//    if (counterIndex == 0)
//      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
//      VerticalMessures.maxValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
//      VerticalMessures.minValue = VerticalMessures.loopValue;

  RollMessures.currentValue = ((int)(((RollMessures.accResult - RollMessures.maxValue - RollMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
  PitchMessures.currentValue = ((int)(((PitchMessures.accResult - PitchMessures.maxValue - PitchMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
  YawMessures.currentValue = ((int)(((YawMessures.accResult - YawMessures.maxValue - YawMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;  
//  SideMessures.currentValue = ((int)(((SideMessures.accResult - SideMessures.maxValue - SideMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;

  }
}

void ride(bool onlyOffset)
{
  isRideProcessing = true;
  
  float VerticalAcceptanceMarginal = 0.2;
  float PlanningAxisAcceptanceMarginal = 0.1;
  float MaxAngleOnAxis = 0.45;

  int8_t UpOrDownSpeed = 0;

  updateCleanerGyroValues();
  
  if (onlyOffset)
  {
    PitchCommandos.Offset = PitchMessures.currentValue * -1;
    RollCommandos.Offset = RollMessures.currentValue * -1;
    YawCommandos.Offset = YawMessures.currentValue * -1;
//    VerticalCommandos.Offset = VerticalMessures.currentValue * -1;
//    SideCommandos.Offset = SideMessures.currentValue * -1;
  }
  else
  {
    if (!withoutOffsets)
    {    
      PitchMessures.currentValue += PitchCommandos.Offset;
      RollMessures.currentValue += RollCommandos.Offset;
      YawMessures.currentValue += YawCommandos.Offset;
      //    VerticalMessures.currentValue += VerticalCommandos.Offset;
      //    SideMessures.currentValue += SideCommandos.Offset;
    }

#ifdef WithSERIALPRIINT
  Serial.print("PitchMessures.currentValue: ");Serial.println(PitchMessures.currentValue);
  Serial.print("RollMessures.currentValue: ");Serial.println(RollMessures.currentValue);
  Serial.print("YawMessures.currentValue: ");Serial.println(YawMessures.currentValue);
#endif

    if (PitchMessures.currentValue > (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
      //Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true);
    }
    else if (PitchMessures.currentValue < (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
      //Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false);
    }
    
    if (RollMessures.currentValue > (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
      //Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true);
    }
    else if (RollMessures.currentValue < (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
     //Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false);
    }
  }
  isRideProcessing = false;
}

void SendGyro()
{
    postGyro = false;
    
    uint16_t statuscode1;
    int16_t length1;
   
    String entireUrl = String(url) + "/gyro?id=1";

    entireUrl += "&pitch="; entireUrl += String(PitchMessures.currentValue);
    entireUrl += "&roll="; entireUrl += String(RollMessures.currentValue);
    entireUrl += "&yaw="; entireUrl += String(YawMessures.currentValue);
      
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
