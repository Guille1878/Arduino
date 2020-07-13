#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>

#define DHTPIN D3
#define RELAYPIN D1
#define LEDPIN D2

#define DHTTYPE DHT11 

#define ESP_SSID "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "taipeimontevideo7816" //"12987351" // Your network password here

#define serverAddress "IntelliCasa20171130112601.azurewebsites.net"

#define deviceId "e698e1cc-9f01-46ca-91be-38c57b9fb1e4"

bool isRelayOn = false;

DHT dht(DHTPIN, DHTTYPE);

void setup() {

  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
    
  WiFi.mode(WIFI_STA);
  WiFi.begin(ESP_SSID, ESP_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {

    Blink(200);
    delay(500);
  }

  dht.begin();

  digitalWrite(RELAYPIN, LOW);
  digitalWrite(LEDPIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
    
}

void loop() {

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  int trying = 0;
  while ((isnan(humidity) || isnan(temperature)) && trying < 3)
  {
    trying++;
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
  }

  if (!(isnan(humidity) || isnan(temperature)))
    SendInformation(temperature,humidity);
    
  delay(20000);

}

void Blink(int interval )
{
  for(uint8_t i=0; i<3; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(interval);
    digitalWrite(LED_BUILTIN, LOW);
    delay(interval);
  }
}

void SendInformation(float humidity, float temperature)
{  

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(serverAddress, httpPort)) {
    return;
  }
   
  client.print(String("GET ") + "/api/Devices?deviceId=" + deviceId + "&value=" + humidity + "&value=" + temperature + " HTTP/1.1\r\n" +
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
    
      Blink(50);
      String line = client.readString();

      int indexStartResponse = line.indexOf("{");
      int indexEndResponse = line.lastIndexOf("}");

      String responseString = line.substring(indexStartResponse,indexEndResponse + 1);
      //Serial.println(responseString);

      int indexSuccess = responseString.indexOf("processed"); //+12
      //Serial.print("processed:");Serial.println(responseString.substring(indexSuccess + 11,indexSuccess + 15));
      if (responseString.substring(indexSuccess + 11,indexSuccess + 15) == "true")
      {
        int indexActionToDo = responseString.indexOf("hasActionsToDo"); // +18
        //Serial.print("hasActionsToDo:");Serial.println(responseString.substring(indexActionToDo + 16,indexActionToDo + 20));
        if (responseString.substring(indexActionToDo + 16,indexActionToDo + 20) == "true")
        {
          int indexActionType = responseString.indexOf("deviceActionType"); // +19
          //Serial.print("deviceActionType:");Serial.println(responseString.substring(indexActionType + 19,indexActionType + 24));
          if (responseString.substring(indexActionType + 19,indexActionType + 24) == "Relay")
          {
            
            int indexActionValue = responseString.indexOf("actionValue"); // +14
            bool actionValue = (responseString.substring(indexActionValue + 13,indexActionValue + 14));
            
            //Serial.print("actionValue:");Serial.println(responseString.substring(indexActionValue + 13,indexActionValue + 14) == "1");
                        
            SwitchRelay(responseString.substring(indexActionValue + 13,indexActionValue + 14) == "1");

            int indexCommandoId = responseString.indexOf("commandoId"); // +13
            String lastLine = line.substring(indexStartResponse + indexCommandoId + 12,indexEndResponse -1);
            int secondLastParentisis = lastLine.lastIndexOf("}");
          
            String commandoId = lastLine.substring(0,secondLastParentisis);
            //Serial.print("commandoId:");Serial.println(commandoId);
            
            delay(500);

            client.print(String("GET ") + "/api/Commandos?commandoId=" + commandoId + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n" + 
               "Connection: close\r\n\r\n");  
          } 
        }
      }
   }   
}

void SwitchRelay(bool onOff)
{

  if (onOff && !isRelayOn)
  {
    isRelayOn = true;
    digitalWrite(LEDPIN, HIGH); // turn on LED with voltage HIGH
    digitalWrite(RELAYPIN, HIGH); // turn on relay with voltage HIGH
  }
  else if (!onOff && isRelayOn)
  {
    isRelayOn = false;
    digitalWrite(LEDPIN, LOW); // turn off LED with voltage HIGH
    digitalWrite(RELAYPIN, LOW); // turn off relay with voltage HIGH
  }  
}
