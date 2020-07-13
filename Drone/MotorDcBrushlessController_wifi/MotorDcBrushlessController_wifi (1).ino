/*
Wille Esteche

// In 1080 and 1090 always staqrt again after it got hang.

*/
#include <Wire.h>
#include <UnoWiFiDevEd.h>
#include <Servo.h>

Servo escs[4];  // create servo object to control a servo
/*
Servo esc2;  // create servo object to control a servo
Servo esc3;  // create servo object to control a servo
Servo esc4;  // create servo object to control a servo
*/

#define ESC_HIGH_DEFAULT 200
#define ESC_LOW_DEFAULT 20

#define PIN_RELAY 4

int currentSpeeds[4] = { ESC_LOW_DEFAULT, ESC_LOW_DEFAULT, ESC_LOW_DEFAULT, ESC_LOW_DEFAULT};

void setup() {

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW); 
  
  Wifi.begin();
  Wifi.println("REST Server is up");

  /*
  delay(10000);
  digitalWrite(PIN_RELAY, HIGH);
  
  */
  
    escs[0].attach(9);  // attaches the servo on pin 9 to the motor FrontLeft
    escs[1].attach(10);  // attaches the servo on pin 10 to the motor FrontRight
    escs[2].attach(11);  // attaches the servo on pin 11 to the motor BackRight
    escs[3].attach(12);  // attaches the servo on pin 12 to the motor BackLeft

    Wifi.println("Motors ON");

  delay(3000);

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    escs[motorIndex].write(currentSpeeds[motorIndex]);
  }

  delay(2000);
  
}


void loop() {

  while(Wifi.available()){
      process(Wifi);
    }
  delay(50);

}

void SetMortosToZero()
{
    currentSpeeds[4] = { ESC_LOW_DEFAULT, ESC_LOW_DEFAULT, ESC_LOW_DEFAULT, ESC_LOW_DEFAULT};

    for (short motorIndex = 0; motorIndex < 4; motorIndex++)
    {
      escs[motorIndex].write(currentSpeeds[motorIndex]);
    }
  
    delay(3000);
     
}

void process(WifiData client) {
  // read the command
  String command = client.readStringUntil('/');
 
  // is "digital" command?
  if (command == "digital") {

    MotorCommand(client);
  }
}

void MotorCommand(WifiData client) {
  int motorIndex, newSpeed;
 
  // Read pin number
  motorIndex = client.parseInt();

  if (motorIndex == 9)
  {
    reportBackVoltage(client);
  }
  else if (motorIndex == 5)
  {
    int OnOff = 0;    
    if (client.read() == '/') {
      OnOff = client.parseInt();
    }

    if (OnOff == 1)
    {
      digitalWrite(PIN_RELAY, HIGH);
      Wifi.println("Motors ON");
    }
    else
    {
      SetMortosToZero();
      digitalWrite(PIN_RELAY, LOW);
      Wifi.println("Motors OFF");
    }
        
    client.println("Status: 200 OK\n");
    client.println("\n");
    client.print(EOL);  
    
  }
  else if (motorIndex >= 1111 && motorIndex < 2222)
  {
   
    // If the next character is a '/' it means we have an URL
    // with a value like: "/digital/2/1050"
    if (client.read() == '/') {
      newSpeed = client.parseInt();
    }
    else {
      newSpeed = ESC_LOW_DEFAULT;
    }
    
    String = motrosStringPins = String(motorIndex);

    for (int index = 0; index < 4; index++)
    {
      if (motrosStringPins.substring(index,1) == "1")
      {
        currentSpeeds[index] = newSpeed;
        setSpeed(escs[index], newSpeed);
      }
    }
   
    // Send feedback to client
    client.println("Status: 200 OK\n");
    client.print(F("Motor: "));
    client.print(motorIndex);
    client.print(F(" set to "));
    client.print(newSpeed);
    client.println("\n");
    client.print(EOL);    //char terminator
  }
}

void setSpeed(Servo motor, int newSpeed)
{
  
  if (newSpeed > 0)
  {
    motor.write(newSpeed);
    delay(20); 
  }
}

void reportBackVoltage(WifiData client)
{

  long vcc = readVcc();
  
  client.print("Battery level: ");
  client.print(vcc);
  client.println("\n");
  client.print(EOL);    //char terminator
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
