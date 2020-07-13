#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "DHT.h"
#include <RCSwitch.h>

#define ESP_SSID "localinda"//"Willesrouter" // Your network name here
#define ESP_PASS "taipeimontevideo7816" //"12987351" // Your network password here

#define serverAddress "IntelliCasa20171130112601.azurewebsites.net"

#define deviceId "ee278c7a-4fed-47b8-8462-5af18f52a20d"

#define DHTPIN D4     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);

RCSwitch mySwitch = RCSwitch();

float insideTemperature = -9999;
float insideHumidity = -9999;
float outsideTemperature = -9999;
float outsideHumidity = -9999;
float outsideHeatIndex = -9999; 
float outsideDarknes = -9999;

int checkingRecivingCountWithZero = 0;
bool isRecivingData = false;

int isTimeToCheckTemperature = 0;
int isTimeToSendInformation = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("Start!");

  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2

  WiFi.mode(WIFI_STA);
  WiFi.begin(ESP_SSID, ESP_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {

    //Blink(200,3);
    delay(500);
  }

  dht.begin();

  //Blink(300,5);

}

void loop() {

  if (!isRecivingData)
  {
    isTimeToCheckTemperature++;
    isTimeToSendInformation++;
  }
  
  ReciveDataF433();

 if (isTimeToCheckTemperature == 100000)
 {
  isTimeToCheckTemperature = 0;
  CheckTemperature();  
 }
        
  if (!isRecivingData)
  {
    if (isTimeToSendInformation == 3000000)
    {
      isTimeToSendInformation = 0;
      SendInformation();
    }
  } 
  else if (checkingRecivingCountWithZero == 10)
  {
    checkingRecivingCountWithZero = 0;
    isRecivingData = false;
  }
  else
  {
    checkingRecivingCountWithZero++;
  }

  //delay(1);

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


   Serial.print("Temp: ");
    Serial.println( inTemperature );

    
 
}

void SetOutsideData(float recivedValue)
{

  String stringValue = String(recivedValue);

  String stringSortOrder = stringValue.substring(0,1);
  String stringSignal = stringValue.substring(1,2);
  String stringOnlyValue = stringValue.substring(2);

/*
  Serial.print("stringSortOrder: ");
  Serial.println(stringSortOrder);
  Serial.print("stringSignal: ");
  Serial.println(stringSignal);
  Serial.print("stringOnlyValue: ");
  Serial.println(stringOnlyValue);
  */

  int sortOrder = stringSortOrder.toInt();
 
  if (sortOrder == 0 || sortOrder > 4)
  {
    isRecivingData = false;
    checkingRecivingCountWithZero = 0;
    return;
  }
   
  float value = stringOnlyValue.toFloat();

  if (stringSignal == "1")
    value *= -1;

  switch (sortOrder)
  {
    case 1:
      outsideTemperature = value / 100;
      break;
    case 2:
      outsideHumidity = value / 100;
      break;
    case 3:
      outsideHeatIndex = value / 100;
      break;
    case 4:
      outsideDarknes = value;
      isRecivingData = false;
      break;
  }  
}


void ReciveDataF433()
{

  do 
  {

    if (mySwitch.available()) 
    {
  
      int recivedValue = mySwitch.getReceivedValue();
      
      Serial.print("Received ");
      Serial.println(recivedValue );
  
      if (recivedValue)
      {
        checkingRecivingCountWithZero = 0;
        
          switch (recivedValue)
          {
            case 88888:
              isRecivingData = true;
              break;
            case 99999:
              isRecivingData = false;
              break;
            default:
              isRecivingData = true;
              SetOutsideData(recivedValue);
          }
          //delay(50);
      }
      else
      {
        checkingRecivingCountWithZero++;
      }      
      mySwitch.resetAvailable();
    }
    
    if (checkingRecivingCountWithZero > 10)
      isRecivingData = false;
      
  } while (isRecivingData == true);
}

void SendInformation()
{  

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(serverAddress, httpPort)) {
    return;
  }


  String toSend = String("GET ") + "/api/Devices?deviceId=" + deviceId + "&value=" + insideTemperature + "&value=" + insideHumidity + "&value=" + outsideTemperature + "&value=" + outsideHumidity + "&value=" + outsideHeatIndex + "&value=" + outsideDarknes + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n";
        
  Serial.println(toSend);   
  
  /*   
  client.print(String("GET ") + "/api/Devices?deviceId=" + deviceId + "&value=" + insideTemperature + "&value=" + insideHumidity + "&value=" + outsideTemperature + "&value=" + outsideHumidity + "&value=" + outsideHeatIndex + "&value=" + outsideDarknes + " HTTP/1.1\r\n" +
               "Host: " + serverAddress + "\r\n");
  */
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


  Serial.println("Sent!");
  //Blink(500,2);
  
   
}
/*
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
*/
