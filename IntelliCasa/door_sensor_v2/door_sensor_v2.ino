/*

 Door Sensor switch relay

 Guille W. Esteche C.
 2018-04-08
*/
#include <Adafruit_VL53L0X.h>
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//#define WithSERIALPRINT
#define RELAYPIN 8
unsigned long lastTimeChange;

void setup() {
 
  pinMode(RELAYPIN, OUTPUT);
  Serial.begin(115200);
  
#ifdef WithSERIALPRINT
  Serial.println("Dorr Sensor with Adafruit VL53L0X started!");
#endif 

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  digitalWrite(RELAYPIN, HIGH);
}
void(* resetFunc) (void) = 0; //declare reset function @ address 0

bool isOn = false;
bool wasOn = false;  

void loop() {
  
  int distance = masureDistance();

  isOn = distance > 200;

  if (wasOn != isOn)
  {
  #ifdef WithSERIALPRINT    
    Serial.print("distance: "); Serial.println(distance);
  #endif
    switchLamp(isOn);
    wasOn = isOn;
  }
  else 
  {
    if (millis() - lastTimeChange > 600000)
    {
      delay(6000);
      resetFunc();
    }
  }
  delay(500);
  
}

int masureDistance()
{
    VL53L0X_RangingMeasurementData_t measure;

  int maxValue = -1, minValue = -1, totalValue = 0, distance = 0;
  for (uint8_t index = 0; index < 5; index++)
  {
    lox.rangingTest(&measure, false);
    int value = measure.RangeMilliMeter;

  #ifdef WithSERIALPRINT    
    Serial.print("value: "); Serial.println(value);
  #endif
      
    if (measure.RangeStatus == 4)
      value = 8191;
    
    if (minValue == -1)
      minValue = maxValue = totalValue = value;
    else
      if (value < minValue)
        minValue = value;
      if (value > maxValue)
        maxValue = value;

    totalValue += value;
    delay(100);
  }
  return ((totalValue - maxValue - minValue) / 3);
}

void switchLamp(bool on)
{
  delay(500);
  if (on)
    digitalWrite(RELAYPIN, HIGH);
  else
    digitalWrite(RELAYPIN, LOW);
  delay(500);

  #ifdef WithSERIALPRINT    
    Serial.print("CHANGED TO: "); Serial.println(on);
  #endif

  lastTimeChange = millis();
}
