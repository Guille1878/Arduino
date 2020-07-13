
// Board it's working as ARDUINO LEANDRO ETH
// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain
int photocellPin = A0;     // the cell and 10K pulldown are connected to a0

#include "DHT.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <SPI.h> // Not actually used but needed to compile

#define DHTPIN 6 

#define ESP_SSID "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "taipeimontevideo7816" //"12987351" // Your network password here
#define serverAddress "IntelliCasa20171130112601.azurewebsites.net"
#define deviceId "25d01342-9fae-475f-b6c4-9b2ecd150f4d"

WiFiClient client;

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
long vcc = -1;

void setup() {
//  Serial.begin(9600);
//  Serial.println("Start!");
         
  dht.begin();
}

void loop() { 

  CheckTemperature();
  delay(1000);
  CheckDarkness();
  delay(1000);
  /*
  ReadVcc();
  delay(1000);
  */
  
  SendInformation(outsideHumidity, outsideTemperature, outsideHeatIndex, outsideDarknes);

  delay(15000);
  
}

void CheckDarkness()
{
    outsideDarknes = analogRead(photocellPin);  
    Blink(50,2);
}

void CheckTemperature()
{
    float inHumidity = dht.readHumidity();
  
    float inTemperature = dht.readTemperature();

//  Serial.print("Checking temp: ");Serial.println(inTemperature);

    if (!isnan(inHumidity)) 
     outsideHumidity = inHumidity * 100;
  
    if (!isnan(inTemperature)) 
      outsideTemperature = inTemperature * 100;
      
    if (!isnan(inHumidity) && !isnan(inTemperature)) 
      outsideHeatIndex = dht.computeHeatIndex(inTemperature, inHumidity, false) * 100; 

    Blink(50,3);
}


void SendInformation(float humidity, float temperature,float feeltemperature, float darkness)
{

  const int httpPort = 80;
  if (!client.connect(serverAddress, httpPort)) {
    return;
  }
   
  client.print(String("GET ") + "/api/Devices?deviceId=" + deviceId + 
               "&value=" + humidity + 
               "&value=" + temperature + 
               "&value=" + feeltemperature + 
               "&value=" + darkness + 
               " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n" + 
               "Connection: close\r\n\r\n");
  
  Blink(100,4);
}


void Blink(int interval, uint8_t times )
{
  for(uint8_t i=0; i<times; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(interval);
    digitalWrite(LED_BUILTIN, LOW);
    delay(interval);
  }
}

/*
void ReadVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  vcc = 1126400L / result; // Back-calculate AVcc in mV
  
}
*/
