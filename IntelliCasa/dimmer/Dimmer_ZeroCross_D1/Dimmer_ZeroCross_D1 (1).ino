/*
PIN for PWM: D3
PIN for ZeroCross: D2 

*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

int AC_LOAD = 16;    // Output to Opto Triac pin
int dimming = 128;  // Dimming level (0-128)  0 = ON, 128 = OFF
int nextStopDimming = 128;
bool anyChanges = false;

#define ESP_SSID "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "taipeimontevideo7816" //"12987351" // Your network password here

#define serverAddress "IntelliCasa20171130112601.azurewebsites.net"

#define deviceId "01d7dfc5-810b-4255-8eb1-43a62a6ba1de"

void setup()
{
  Serial.begin(9600); Serial.println("Started");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ESP_SSID, ESP_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.println("No connection");
    Blink(200,3);
    delay(500);
  }
  Serial.println("WIFI On");
  
  pinMode(AC_LOAD, OUTPUT);// Set AC Load pin as output
  attachInterrupt(4, zero_crosss_int, RISING);  // Choose the zero cross interrupt # from the table above
  delay(500);
  
  Serial.println("Initializated");

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

// the interrupt function must take no parameters and return nothing
void zero_crosss_int()  // function to be fired at the zero crossing to dim the light
{
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms 
  // Every zerocrossing thus: (50Hz)-> 10ms (1/2 Cycle) For 60Hz => 8.33ms (10.000/120)
  // 10ms=10000us
  // (10000us - 10us) / 128 = 75 (Approx) For 60Hz =>65

  int dimtime = (65*dimming);    // For 60Hz =>65    
  delayMicroseconds(dimtime);    // Off cycle
  digitalWrite(AC_LOAD, HIGH);   // triac firing
  delayMicroseconds(10);         // triac On propogation delay (for 60Hz use 8.33)
  digitalWrite(AC_LOAD, LOW);    // triac Off
}

void loop()  {
   
  nextStopDimming -= 32;

  if (anyChanges)
    SetDimmer();

  SendInformation();
  delay(3000);
}

void SetDimmer()
{
  anyChanges = false;
  
  int dimmingDiff = nextStopDimming - dimming;
  
  if (dimmingDiff != 0)
  {
   
    int initDimming = dimming;
    
    if (dimmingDiff > 0)
    {
      for (int i=initDimming; i <= nextStopDimming; i++)
      {
        dimming=i;
        delay(12);
      }
    }
    else if (dimmingDiff < 0)
    {
      for (int i=initDimming; i > nextStopDimming; i--)
      {
        dimming=i;
        delay(12);
      }
    }
    Blink(400,2);
  } 
}

void SendInformation()
{  

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(serverAddress, httpPort)) {
    return;
  }

  Serial.println(String("GET ") + "/api/Devices?deviceId=" + deviceId + " HTTP/1.1\r\n" + "Host: " + serverAddress + "\r\n");
     
  client.print(String("GET ") + "/api/Devices?deviceId=" + deviceId + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n" +
               "Connection: close\r\n\r\n");
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
      
      Serial.println("RESPONSE: "); Serial.println(responseString);

      int indexSuccess = responseString.indexOf("processed"); //+12
      Serial.print("processed:"); Serial.println(responseString.substring(indexSuccess + 11,indexSuccess + 15));
      if (responseString.substring(indexSuccess + 11,indexSuccess + 15) == "true")
      {
        int indexActionToDo = responseString.indexOf("hasActionsToDo"); // +18
        Serial.print("hasActionsToDo:"); Serial.println(responseString.substring(indexActionToDo + 16,indexActionToDo + 20));
        if (responseString.substring(indexActionToDo + 16,indexActionToDo + 20) == "true")
        {
          int indexActionType = responseString.indexOf("deviceActionType"); // +19
          Serial.print("deviceActionType:"); Serial.println(responseString.substring(indexActionType + 19,indexActionType + 24));
          if (responseString.substring(indexActionType + 19,indexActionType + 25) == "Dimmer")
          {
            
            int indexActionValue = responseString.indexOf("actionValue"); // +14
            String strNewValue = responseString.substring(indexActionValue + 13,indexActionValue + 16);
            int newValue = strNewValue.toInt();

            nextStopDimming = (100 - newValue) * 128 * 100;
            anyChanges = true;
            
            Serial.print("newValue:");Serial.println(responseString.substring(indexActionValue + 13,indexActionValue + 16));
                        
            int indexCommandoId = responseString.indexOf("commandoId"); // +13
            String lastLine = line.substring(indexStartResponse + indexCommandoId + 12,indexEndResponse -1);
            int secondLastParentisis = lastLine.lastIndexOf("}");
          
            String commandoId = lastLine.substring(0,secondLastParentisis);
            //Serial.print("commandoId:");
            //Serial.println(commandoId);
            
            //delay(500);

            Serial.println(String("GET ") + "/api/Commandos?commandoId=" + commandoId + " HTTP/1.1\r\n" + "Host: " + serverAddress + "\r\n" + "Connection: close\r\n\r\n");

            client.print(String("GET ") + "/api/Commandos?commandoId=" + commandoId + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n" +
               "Connection: close\r\n\r\n");  
          }          
        }
      }
   }  
}

