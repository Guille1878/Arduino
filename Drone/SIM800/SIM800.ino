#include "Adafruit_FONA.h"

#define WithAIRPRESSURE
#define WithSERIALPRIINT

#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_RI  7

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

char replybuffer[255];
char url[80] = "http://intellicasa20171130112601.azurewebsites.net/api/Casas";

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

bool isFonaOn = false;

void setup() {
  
   while (!Serial);

#ifdef WithSERIALPRIINT
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));
#endif

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
#ifdef WithSERIALPRIINT    
    Serial.println(F("Couldn't find FONA"));
#endif
    while (1);
  }
#ifdef WithSERIALPRIINT  
  Serial.println(F("FONA found!!"));
#endif

  delay(10000);

#ifdef WithAIRPRESSURE
  if (!bme.begin()) {
    #ifdef WithSERIALPRIINT    
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    #endif    
    while (1);
  }
  #ifdef WithSERIALPRIINT  
    else
      Serial.println("Air Pressure ON");
  #endif
#endif

  Serial.println("Ready to start.");

}

void loop() {

  if (!isFonaOn)
  {
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
    
  }
  else
  {
      //ALTITUDE =========================
      

      //BATTERY =========================
      uint16_t vbat;
      if (! fona.getBattPercent(&vbat)) {
          Serial.println(F("Failed to read Batt"));
        } else {
          Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
        }
      // ================================
    
      //GPS =========================
      uint16_t returncode;
      if (!fona.getGSMLoc(&returncode, replybuffer, 250))
          Serial.println(F("Failed!"));
        if (returncode == 0) {
          Serial.print(F("GPS: ##"));
          Serial.print(replybuffer);
          Serial.println(F("##"));
        } else {
          Serial.print(F("Fail code #")); Serial.println(returncode);
        }
      delay(1000);
      // ================================


      //GPRS Web =========================
        uint16_t statuscode;
        int16_t length;
        
        if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
          delay(1000);
          return;
        }
       
        Serial.println("Result: ##");
        
         while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

            // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif
            length--;
            if (! length) break;
          }
        }
        Serial.println("## END Result");
        Serial.println(F("\n****"));
        fona.HTTP_GET_end();
        // ================================
    
  }
  
  delay(2000);


}


