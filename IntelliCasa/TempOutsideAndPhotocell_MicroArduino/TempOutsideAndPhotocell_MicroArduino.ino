
// Board it's working as ARDUINO LEANDRO ETH
// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain
int photocellPin = A0;     // the cell and 10K pulldown are connected to a0
int photocellReading;

#include "DHT.h"

#include <SPI.h> // Not actually used but needed to compile
#include <RCSwitch.h>
#define DHTPIN 6 

RCSwitch mySwitch = RCSwitch();

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

float outsideTemperature = -1;
float outsideHumidity = -1;
float outsideHeatIndex = -1; 
float outsideDarknes = -1;

void setup() {
//  Serial.begin(9600);
//  Serial.println("Start!");

  mySwitch.enableTransmit(8);   // Transmitter is connected to Arduino Pin #8  

  mySwitch.setProtocol(2);

  /*
  if (!driver.init())
       Serial.println("F433 init failed");
  */
         
  dht.begin();
}

void loop() {

  CheckTemperature();

  CheckDarkness();

  SendDataByF433();

  delay(20000);
  
}

void CheckDarkness()
{
    outsideDarknes = analogRead(photocellPin);  
}

void CheckTemperature()
{
    float inHumidity = dht.readHumidity();
  
    float inTemperature = dht.readTemperature();

//  Serial.print("Checking temp: ");
//  Serial.println(inTemperature);

    if (!isnan(inHumidity)) 
     outsideHumidity = inHumidity * 100;
  
    if (!isnan(inTemperature)) 
      outsideTemperature = inTemperature * 100;
      
    if (!isnan(inHumidity) && !isnan(inTemperature)) 
      outsideHeatIndex = dht.computeHeatIndex(inTemperature, inHumidity, false) * 100; 
}

void SendDataByF433()
{

  //mySwitch.send(5393, 24);
  //delay(50);
  
  mySwitch.send(88888, 24);
  delay(50);
  
  //Serial.print("outsideTemper: ");
  //Serial.println(outsideTemperature);
  
  mySwitch.send(ToSendingNumber(outsideTemperature,1), 24);
  delay(50);

  //Serial.print("outsideHumidity: ");
  //Serial.println(outsideHumidity);

  mySwitch.send(ToSendingNumber(outsideHumidity,2), 24);
  delay(50);
  
  //Serial.print("outsideHeatIndex: ");
  //Serial.println(outsideHeatIndex);

  mySwitch.send(ToSendingNumber(outsideHeatIndex,3), 24);
  delay(50);

  //Serial.print("outsideDarknes: ");
  //Serial.println(outsideDarknes);

  mySwitch.send(ToSendingNumber(outsideDarknes,4), 24);
  delay(50);

  mySwitch.send(99999, 24);
  delay(100);

  /*
  String stringTemperature = String(outsideTemperature);
  String stringHumidity = String(outsideHumidity);
  String stringHeatIndex = String(outsideHeatIndex);
  String stringDarkness = String(outsideDarknes);

  //const char *msg = "Hello World!";
  
  const char *msg = stringTemperature + "*" + stringHumidity + "*" + stringHeatIndex + "*" + stringDarkness;
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  delay(1000);
  */
    
}

float ToSendingNumber(float number, int sortOrder)
{
  return number < 0 ? (number * -1) + (100000 * sortOrder) + 10000: number + (100000 * sortOrder);
}

/*
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
*/
