#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>

#define ESP_SSID "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "taipeimontevideo7816" //"12987351" // Your network password here

#define serverAddress "IntelliCasa20171130112601.azurewebsites.net"
#define deviceId "01d7dfc5-810b-4255-8eb1-43a62a6ba1de"

//#define SERIAL_PORT_ON

byte nextStopDimming = 128;
bool anyChanges = false;

void setup() {
  
#ifdef SERIAL_PORT_ON
  Serial.begin(9600); Serial.println("Started");
#endif
  
  Wire.begin(); // join i2c bus (address optional for master)
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ESP_SSID, ESP_PASS);

   while (WiFi.status() != WL_CONNECTED) {
    //Serial.println("No connection");
    Blink(200,3);
  }

#ifdef SERIAL_PORT_ON  
  Serial.println("WIFI On");
#endif  
}

void Blink(int interval, int times )
{
  for(uint8_t i=0; i<times; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(interval);
    digitalWrite(LED_BUILTIN, LOW);
    delay(interval);
  }
}

void loop() {
  
  SearchNewCommando();

  if (anyChanges)
    SendCommandoForward(nextStopDimming);

  delay(500);
}

void SendCommandoForward(byte newValue)
{
  #ifdef SERIAL_PORT_ON  
    Serial.print("SendToDimmer: "); Serial.println(newValue);
  #endif  
  
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(newValue);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  anyChanges = false;

}

void SearchNewCommando()
{  

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(serverAddress, httpPort)) {
    return;
  }

#ifdef SERIAL_PORT_ON
  Serial.println(String("GET ") + "/api/Devices?deviceId=" + deviceId + " HTTP/1.1\r\n" + "Host: " + serverAddress + "\r\n");
#endif
     
  client.print(String("GET ") + "/api/Devices?deviceId=" + deviceId + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n");// + 
               //"Connection: close\r\n\r\n";);
  client.println();

  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

   while (client.available()) 
   {
    
      Blink(50,5);
      String line = client.readString();

      int indexStartResponse = line.indexOf("{");
      int indexEndResponse = line.lastIndexOf("}");

      String responseString = line.substring(indexStartResponse,indexEndResponse + 1);

#ifdef SERIAL_PORT_ON      
      Serial.println("RESPONSE: "); Serial.println(responseString);
#endif
      int indexSuccess = responseString.indexOf("processed"); //+12
      
#ifdef SERIAL_PORT_ON      
      Serial.print("processed:"); Serial.println(responseString.substring(indexSuccess + 11,indexSuccess + 15));
#endif
      
      if (responseString.substring(indexSuccess + 11,indexSuccess + 15) == "true")
      {
        int indexActionToDo = responseString.indexOf("hasActionsToDo"); // +18
#ifdef SERIAL_PORT_ON        
        Serial.print("hasActionsToDo:"); Serial.println(responseString.substring(indexActionToDo + 16,indexActionToDo + 20));
#endif        
        if (responseString.substring(indexActionToDo + 16,indexActionToDo + 20) == "true")
        {
          int indexActionType = responseString.indexOf("deviceActionType"); // +19
#ifdef SERIAL_PORT_ON          
          Serial.print("deviceActionType:"); Serial.println(responseString.substring(indexActionType + 19,indexActionType + 24));
#endif          
          if (responseString.substring(indexActionType + 19,indexActionType + 25) == "Dimmer")
          {
            int indexActionValue = responseString.indexOf("actionValue"); // +14
            String strNewValue = responseString.substring(indexActionValue + 13,indexActionValue + 16);
            int newValue = strNewValue.toInt();
#ifdef SERIAL_PORT_ON            
            Serial.print("newValue: ");Serial.println(responseString.substring(indexActionValue + 13,indexActionValue + 16));
#endif
            int newValuePorcent = ((100 - newValue) * 128) / 100;
#ifdef SERIAL_PORT_ON            
            Serial.print("newValuePorcent: ");Serial.println(newValuePorcent);
#endif
            if (newValuePorcent != nextStopDimming)
            {
              nextStopDimming = newValuePorcent;
              anyChanges = true;
            }
            int indexCommandoId = responseString.indexOf("commandoId"); // +13
            String lastLine = line.substring(indexStartResponse + indexCommandoId + 12,indexEndResponse -1);
            int secondLastParentisis = lastLine.lastIndexOf("}");
          
            String commandoId = lastLine.substring(0,secondLastParentisis);
            //Serial.print("commandoId:");
            //Serial.println(commandoId);
            
            delay(300);
#ifdef SERIAL_PORT_ON
            Serial.println(String("GET ") + "/api/Commandos?commandoId=" + commandoId + " HTTP/1.1\r\n" + "Host: " + serverAddress + "\r\n" + "Connection: close\r\n\r\n");
#endif
            client.print(String("GET ") + "/api/Commandos?commandoId=" + commandoId + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n" +
               "Connection: close\r\n\r\n");  
          }          
        }
      }
   }
}
