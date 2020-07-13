#include <WiFiClient.h>
#include <ESP8266WiFi.h>

#define ESP_SSID "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "taipeimontevideo7816" //"12987351" // Your network password here

void setup() {

Serial.begin(9600);

Serial.println("Started:");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ESP_SSID, ESP_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.println("Conected!");
  String clientMac = "";
  unsigned char mac[6];
  WiFi.macAddress(mac);
  clientMac += macToStr(mac);

  Serial.println(clientMac);
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) 
  {
    result += String(mac[i], 16);
    if (i < 5)
    result += ':';
  }
  return (result);
}


void loop() {

  
  // put your main code here, to run repeatedly:

}
