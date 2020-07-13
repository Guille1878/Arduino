#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "DHT.h"

#define ESP_SSID "localinda2" // "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "1578taipeimontevideo" // "taipeimontevideo7816" //"12987351" // Your network password here

#define serverAddress "IntelliCasa20171130112601.azurewebsites.net"

#define deviceId "ee278c7a-4fed-47b8-8462-5af18f52a20d"

#define DHTPIN D4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

//#define WithSERIALPRINT

DHT dht(DHTPIN, DHTTYPE);

float insideTemperature = -9999;
float insideHumidity = -9999;

  WiFiClient client;
  const int httpPort = 80;

void setup() {

#ifdef WithSERIALPRINT
  Serial.begin(9600);
  Serial.println("Start!");
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ESP_SSID, ESP_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {

    Blink(200,3);
    delay(500);
  }

  dht.begin();

  Blink(300,5);

}

void loop() {

  CheckTemperature();  
  delay(3000);
  SendInformation();
  delay(7000);

}

void CheckTemperature()
{
    float inHumidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
    float inTemperature = dht.readTemperature();

    if (!isnan(inHumidity)) 
     insideHumidity = inHumidity;
  
    if (!isnan(inTemperature)) 
      insideTemperature = inTemperature;

#ifdef WithSERIALPRINT
   Serial.print("Temp: ");
   Serial.println( inTemperature );
#endif    
 
}



void SendInformation()
{  

  if (!client.connect(serverAddress, httpPort)) return;

  String toSend = String("GET ") + "/api/Devices?deviceId=" + deviceId + "&value=" + insideTemperature + "&value=" + insideHumidity + " HTTP/1.1\r\n" + "Host: " + serverAddress + "\r\n";

#ifdef WithSERIALPRINT
  Serial.println(toSend);   
#endif

  client.print(toSend);
  client.println();

  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

Blink(500,2);

#ifdef WithSERIALPRINT
  Serial.println("Sent!");
#endif 
}

void Blink(int interval, int times )
{
  for(uint8_t i=0; i<times; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(interval);
    digitalWrite(LED_BUILTIN, LOW);
    
    if (i == times - 1)
      return;
      
    delay(interval);
  }
}
