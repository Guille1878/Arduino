#include <Wire.h>
#include "Adafruit_FONA.h"

//#define WithPINCode
//#define WithAIRPRESSURE

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
int8_t MotorBalanceCoefficient = 0.1;

struct CommandoDef
{
  int kind;
  uint32_t value;
};

bool isStramingCommando = false;

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

  Wire.begin(6);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent); 
  
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

}

//=====LOOP=====//
void loop() {

  if (isStramingCommando)
      return;

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
        #endif
          break;
        case 2:
          updateSignalLevel();
          break;
        case 3:
          getDroneBatteryLevel();
          break;
        /*
        case 4:
          getDroneAltitude();
          break;
        */
        case 5:
          getHttp();
          loopproccess = -1;
          break;
      }
      looptimes = 0;
      loopproccess++;
    }
    else looptimes++;

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

    isStramingCommando = true;
  
    if (!fona.HTTP_GET_start(urlChars, &statuscode, (uint16_t *)&length)) {
#ifdef WithSERIALPRIINT          
      Serial.print("HTTP Failed! "); Serial.println(statuscode);
#endif     
     isStramingCommando = false;
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
                isStramingCommando = false;
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
                 isStramingCommando = false;
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
        isStramingCommando = false;
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
          case 92:
            rinding = (commands[cmdindex].value == 1);
            break;
          case 93:
            MotorBalanceCoefficient = commands[cmdindex].value;
            break;
          case 94:
            withoutOffsets = (commands[cmdindex].value == 1);
            break;
          case 95:
            postGyro = true;
            break;
          case 96:
            RollCommandos.WishedValue = commands[cmdindex].value;
            break;
          case 97:
            PitchCommandos.WishedValue = commands[cmdindex].value;
            break;
          /*
          case 97:
            YawCommandos.WishedValue = commands[cmdindex].value;
            break;
          */
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

void receiveEvent(int howMany)
{
  uint32_t response = 0;
  
  response = (uint32_t) Wire.read() << 24;
  response |= (uint32_t) Wire.read() << 16;
  response |= (uint32_t) Wire.read() << 8;
  response |= (uint32_t) Wire.read(); 

  //if (response == 4294967295) return;
    
#ifdef WithLOGGS    
  logtext("Reviced Event: ");logtext(String(response),true);         // print the integer
#endif   
/*
#ifdef WithSERIALPRIINT  
  Serial.print(F("Reviced Event: "));Serial.println(String(response));
#endif
*/
  getCommando(response);
  
}


void requestEvent() {  

#ifdef WithSERIALPRIINT  
  Serial.print(F("requestId: "));Serial.println(String(requestId));
#endif

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
/*
    #ifdef WithSERIALPRIINT  
      Serial.print(F("Requested volt: "));Serial.println(volt);
    #endif
  */  
      break;
      
  }   
  requestId = 0;
}
