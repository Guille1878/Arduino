//PIN for PWM: 16, Wemos=D0
//PIN for PWM: 0, Wemos=D3
//PIN for PWM: 5
//PIN for ZeroCross: 0, Wemos=D2, A2 

#include <Wire.h>

#define SERIAL_PORT_ON

int AC_LOAD = 5;    // Output to Opto Triac pin
int dimming = 128;  // Dimming level (0-128)  0 = ON, 128 = OFF
int nextStopDimming = 128;
bool anyChanges = false;
bool recivingData = false;

void setup() {
  
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
#ifdef SERIAL_PORT_ON  
  Serial.begin(9600);           // start serial for output
#endif  
  pinMode(AC_LOAD, OUTPUT);// Set AC Load pin as output
  attachInterrupt(0, zero_crosss_int, RISING);  // Choose the zero cross interrupt # from the table above
  
  Blink(200,3);  

#ifdef SERIAL_PORT_ON  
  Serial.println("Initializated");
#endif    
}

void loop() {

  if (anyChanges)
      SetDimmer();
  
  delay(500);
}

void SetDimmer()
{
  anyChanges = false;

  int dimmingDiff = nextStopDimming - dimming;

#ifdef SERIAL_PORT_ON  
    Serial.print("SetDimmer dimmingDiff: ");  Serial.println(dimmingDiff);
#endif    
  
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

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 
{
  recivingData = true;
  
  int valueIn = Wire.read();
    
#ifdef SERIAL_PORT_ON  
    Serial.print("valueIn: ");  Serial.println(valueIn);
#endif    
    if (valueIn != nextStopDimming)
    {
      nextStopDimming = valueIn;
      anyChanges = true;
      Blink(50,4);
#ifdef SERIAL_PORT_ON  
    Serial.print("nextStopDimming: ");  Serial.println(nextStopDimming);
#endif    
    }
  recivingData = false;
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
