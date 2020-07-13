// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain
int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;

#include "DHT.h"
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

#define DHTPIN 11 
RH_ASK driver;

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHTxx test!");

  if (!driver.init())
       Serial.println("init failed");
         
  dht.begin();
}


void loop() {
  // Wait a few seconds between measurements.
  delay(5000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    return;
  }
  
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  photocellReading = analogRead(photocellPin);  

  SendDataByF433(t,h,hic,photocellReading);
  
  
}


  // 5 places for eg "+1221" that means "12.21" or "-0563" that means "-5.63"
  // temp  hum   hic phCell
  // Lenght 23
  //"_____*_____*_____*_____"
  char *msg = "_____*_____*_____*_____";
void SendDataByF433(float temperature, float humidity, float heatIndex,float darkness)
{
  String stringTemperature = String(temperature);
  String stringHumidity = String(humidity);
  String stringHeatIndex = String(heatIndex);
  String stringDarkness = String(darkness);

  //const char *msg = "Hello World!";
  
  const char *msg = stringTemperature + "*" + stringHumidity + "*" + stringHeatIndex + "*" + stringDarkness;
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  delay(1000);
    
}

String ConvertFloatToSendingString(float value)
{
  String result = String(value);
  result.replace(".","");
  result.replace(",","");
  result.replace("-","");
  result.padLeft("0",4);

  String leftSign = value < 0 ? "-" : "+";
  
  result = leftSign + result;

  return result;
}

void String::padLeft(const char cPadWith, const unsigned char cMaxLen)
{
  String strResult = *this;

  *this = "";
  while (this->length() < cMaxLen)
    *this += String(cPadWith);
  *this += strResult;
}

