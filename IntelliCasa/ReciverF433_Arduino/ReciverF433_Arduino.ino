#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

float insideTemperature = -1;
float insideHumidity = -1;
float outsideTemperature = -1;
float outsideHumidity = -1;
float outsideHeatIndex = -1; 
float outsideDarknes = -1;

int checkingRecivingCountWithZero = 0;
int recivingCountOrder = 0;
int isTimeToCheckTemperature = 0;
int isTimeToSendInformation = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("Start!");

  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2 or (D3 in NodeMCU/Wemos)

}

void loop() {

   
  ReciveDataF433();
/*
  Serial.print("outsideDarknes: ");
  Serial.println(outsideDarknes);
*/
 

 delay(150); 

}


void ReciveDataF433()
{

  if (mySwitch.available()) {

    Serial.print("Received ");
    Serial.println( mySwitch.getReceivedValue() );
/*
    Serial.print("Received ");
    Serial.print( mySwitch.getReceivedValue() );
    Serial.print(" / ");
    Serial.print( mySwitch.getReceivedBitlength() );
    Serial.print("bit ");
    Serial.print("Protocol: ");
    Serial.println( mySwitch.getReceivedProtocol() );
  */  
    int recivedValue = mySwitch.getReceivedValue();
 
    if (recivedValue)
    {
      checkingRecivingCountWithZero = 0;

      
        switch (recivedValue)
        {
          case 88888:
            recivingCountOrder = 1;
            break;
          case 99999:
            recivingCountOrder = 0;
            break;
          default:
            SetOutsideData(recivedValue);
        }
        //delay(50);
    }        
    mySwitch.resetAvailable();
  }
}

void SetOutsideData(float recivedValue)
{
  if (recivingCountOrder == 0)
    return;
   
  if (recivingCountOrder > 4)
  {
    recivingCountOrder = 0;
    return;
  }

  switch (recivingCountOrder)
  {
    case 1:
      outsideTemperature = recivedValue;
      break;
    case 2:
      outsideHumidity = recivedValue;
      break;
    case 3:
      outsideHeatIndex = recivedValue;
      break;
    case 4:
      outsideDarknes = recivedValue;
  }  
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
