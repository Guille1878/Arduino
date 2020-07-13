/*

 Door Sensor switch relay

 Guille W. Esteche C.
 2018-04-08
*/

#define RELAYPIN 8
#define DOORSENSORPIN 9

int relayStatus = LOW;

void setup() {

  pinMode(DOORSENSORPIN, INPUT);

  pinMode(RELAYPIN, OUTPUT);

  //Serial.begin(9600);

}

int countingZeros = 0;
int isOn = 0;
int lastStatus = 0;

void loop() {
  
  int currentStatus = digitalRead(DOORSENSORPIN);
  //Serial.println(currentStatus);

if (currentStatus == lastStatus)
{
  countingZeros++;
  isOn = (countingZeros < 50);
}
else
  {
    countingZeros = 0;
    isOn = true;
  }

  lastStatus = currentStatus;

  if (relayStatus != isOn)  
  {
    relayStatus = isOn;
    //Serial.print("CHANGING: ");
    //Serial.println(isOn);
    digitalWrite(RELAYPIN, currentStatus);
  }
  
  delay(50);
  
}
