/*
  Wille Esteche

  // In 1080 and 1090 always staqrt again after it got hang.

*/
#include <Wire.h>
#include <UnoWiFiDevEd.h>
#include <Servo.h>
#include <SPI.h>
#include "MPU6050WE.h"
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>

// GyroScope
MPU6050 mpu6050(Wire);

/*
// Altitude
#define BMP_SCK 13
#define BMP_MISO 12 // SDO
#define BMP_MOSI 11 // SDI
#define BMP_CS 10 //CSB
Adafruit_BMP280 bme(BMP_CS); // hardware SPI
*/

//Relay
#define PIN_RELAY 3

// Motors
#define ESC_HIGH_DEFAULT 200
#define ESC_LOW_DEFAULT 20

//Servo escs[4];  // create servo object to control a servo
//bool isInSettingMode = false;

bool rinding = false;
bool isRideProcessing = false;

typedef struct 
{
  float currentSpeed = ESC_LOW_DEFAULT;
  float balanceRoll = 0;
  float balancePitch = 0;
  Servo esc;
} Motor;

Motor motors[4];

typedef struct AvgMessures
{
  float maxValue = 0;
  float minValue = 0;
  float accResult = 0;
  float loopValue = 0;
  float currentValue = 0;
};

typedef struct AxisCommandos
{
  float Offset = 0;
  float WishedValue = 0;
};

bool withoutOffsets = false;

AxisCommandos RollCommandos,
              PitchCommandos;
//              YawCommandos,
//              VerticalCommandos,
//              SideCommandos;

void setup() {

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  Wifi.begin();
  Wifi.println("REST Server is up");


  motors[0].esc.attach(4);  // attaches the servo on pin 4 to the motor FrontLeft
  motors[1].esc.attach(5);  // attaches the servo on pin 5 to the motor FrontRight
  motors[2].esc.attach(6);  // attaches the servo on pin 6 to the motor BackRight
  motors[3].esc.attach(7);  // attaches the servo on pin 7 to the motor BackLeft

  delay(3000);

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].esc.write(motors[motorIndex].currentSpeed);
  }

  delay(1000);

  Wifi.println("Motors initiated");

  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  delay(1000);

  Wifi.println("Gyroscope ON");

/*
  if (!bme.begin()) {
    Wifi.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  else
    Wifi.println("Air Pressure ON");
*/

  ride(true,Wifi);
  Wifi.println("Offset done!");
  Wifi.println("Ready to start.");

}


int8_t looptimes = 0;
void loop() {

  while (Wifi.available()) {
    process(Wifi);
  }

  if (looptimes == 50)
  {
    if (rinding && !isRideProcessing)
      ride(false,Wifi);

    looptimes = 0;
  }

  looptimes++;

}

void SetMortosToZero()
{
  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    motors[motorIndex].currentSpeed = ESC_LOW_DEFAULT;
    motors[motorIndex].esc.write(motors[motorIndex].currentSpeed);
  }
}

void process(WifiData client) {
  // read the command
  String command = client.readStringUntil('/');

  // is "digital" command?
  if (command == "digital") {

    MotorCommand(client);
  }
}

int8_t MotorDifferenceMarginal = 6;

void ride(bool onlyOffset, WifiData client)
{
  isRideProcessing = true;
  
  float VerticalAcceptanceMarginal = 0.2;
  float PlanningAxisAcceptanceMarginal = 0.1;
  float MaxAngleOnAxis = 0.45;

  int8_t UpOrDownSpeed = 0;
  int8_t bestOf = 5;

  AvgMessures RollMessures,
              PitchMessures;
//              SideMessures,
//              YawMessures,
//              VerticalMessures;

  mpu6050.update();

  for (int8_t counterIndex = 0; counterIndex < bestOf; counterIndex++)
  {

    RollMessures.loopValue = mpu6050.getAccY();
    PitchMessures.loopValue = mpu6050.getAccX();
//    SideMessures.loopValue = mpu6050.getAccZ();
//    YawMessures.loopValue = mpu6050.getAngleZ();
//    VerticalMessures.loopValue = bme.readAltitude(1022);

    if (counterIndex == 0)
      RollMessures.maxValue = RollMessures.minValue = RollMessures.loopValue;
    else if (RollMessures.loopValue > RollMessures.maxValue)
      RollMessures.maxValue = RollMessures.loopValue;
    else if (RollMessures.loopValue < RollMessures.minValue)
      RollMessures.minValue = RollMessures.loopValue;

    if (counterIndex == 0)
      PitchMessures.maxValue = PitchMessures.minValue = PitchMessures.loopValue;
    else if (PitchMessures.loopValue > PitchMessures.maxValue)
      PitchMessures.maxValue = PitchMessures.loopValue;
    else if (PitchMessures.loopValue < PitchMessures.minValue)
      PitchMessures.minValue = PitchMessures.loopValue;

//    if (counterIndex == 0)
//      SideMessures.maxValue = SideMessures.minValue = SideMessures.loopValue;
//    else if (SideMessures.loopValue > SideMessures.maxValue)
//      SideMessures.maxValue = SideMessures.loopValue;
//    else if (SideMessures.loopValue < SideMessures.minValue)
//      SideMessures.minValue = SideMessures.loopValue;
//
//    if (counterIndex == 0)
//      YawMessures.maxValue = YawMessures.minValue = YawMessures.loopValue;
//    else if (YawMessures.loopValue > YawMessures.maxValue)
//      YawMessures.maxValue = YawMessures.loopValue;
//    else if (YawMessures.loopValue < YawMessures.minValue)
//      YawMessures.minValue = YawMessures.loopValue;
//
//    if (counterIndex == 0)
//      VerticalMessures.maxValue = VerticalMessures.minValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue > VerticalMessures.maxValue)
//      VerticalMessures.maxValue = VerticalMessures.loopValue;
//    else if (VerticalMessures.loopValue < VerticalMessures.minValue)
//      VerticalMessures.minValue = VerticalMessures.loopValue;

  }

  RollMessures.currentValue = ((int)(((RollMessures.accResult - RollMessures.maxValue - RollMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
  PitchMessures.currentValue = ((int)(((PitchMessures.accResult - PitchMessures.maxValue - PitchMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  SideMessures.currentValue = ((int)(((SideMessures.accResult - SideMessures.maxValue - SideMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  YawMessures.currentValue = ((int)(((YawMessures.accResult - YawMessures.maxValue - YawMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;
//  VerticalMessures.currentValue = ((int)(((VerticalMessures.accResult - VerticalMessures.maxValue - VerticalMessures.minValue) / (bestOf - 2)) * 100)) / 100.0;

  if (onlyOffset)
  {
    PitchCommandos.Offset = PitchMessures.currentValue * -1;
    RollCommandos.Offset = RollMessures.currentValue * -1;
//    YawCommandos.Offset = YawMessures.currentValue * -1;
//    VerticalCommandos.Offset = VerticalMessures.currentValue * -1;
//    SideCommandos.Offset = SideMessures.currentValue * -1;
  }
  else
  {
    if (!withoutOffsets)
    {    
      PitchMessures.currentValue += PitchCommandos.Offset;
      RollMessures.currentValue += RollCommandos.Offset;
      //    YawMessures.currentValue += YawCommandos.Offset;
      //    VerticalMessures.currentValue += VerticalCommandos.Offset;
      //    SideMessures.currentValue += SideCommandos.Offset;
    }

    if (PitchMessures.currentValue > (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
      client.print("PitchValue: "); client.println(PitchMessures.currentValue);
      Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true, client);
    }
    else if (PitchMessures.currentValue < (PitchCommandos.WishedValue + (PitchCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
      client.print("PitchValue: "); client.println(PitchMessures.currentValue);
      Motors_BalancePitch(((PitchCommandos.WishedValue - PitchMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false, client);
    }
    
    if (RollMessures.currentValue > (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal * -1 : PlanningAxisAcceptanceMarginal))
    {
      client.print("RollValue: "); client.println(RollMessures.currentValue);
      Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, true, client);
    }
    else if (RollMessures.currentValue < (RollCommandos.WishedValue + (RollCommandos.WishedValue < 0) ? PlanningAxisAcceptanceMarginal : PlanningAxisAcceptanceMarginal * -1))
    {
      client.print("RollValue: "); client.println(RollMessures.currentValue);
      Motors_BalanceRoll(((RollCommandos.WishedValue - RollMessures.currentValue) * MotorDifferenceMarginal) / MaxAngleOnAxis, false, client);
    }
  }
  isRideProcessing = false;
}


void MotorCommand(WifiData client) {

  int commandoValue, newSpeed;

  // Read pin number
  commandoValue = client.parseInt();

  if ((commandoValue >= 1111 && commandoValue < 2222))
  {
    // If the next character is a '/' it means we have an URL
    // with a value like: "/digital/2/1050"
    if (client.read() == '/') {
      newSpeed = client.parseInt();
    }
    else {
      newSpeed = ESC_LOW_DEFAULT;
    }

    String motrosStringPins = String(commandoValue);

    client.print(F("PinsString: "));
    client.print(motrosStringPins);
    client.print(F(". Value to add: "));
    client.println(newSpeed);

    for (int index = 0; index < 4; index++)
    {
      client.print(F("Loop string part: "));
      client.println(motrosStringPins.substring(index, index + 1));

      if (motrosStringPins.substring(index, index + 1) == "1")
      {
        if (motors[index].currentSpeed + newSpeed < ESC_LOW_DEFAULT)
          motors[index].currentSpeed = ESC_LOW_DEFAULT;
        else if (motors[index].currentSpeed + newSpeed > ESC_HIGH_DEFAULT)
          motors[index].currentSpeed = ESC_HIGH_DEFAULT;
        else
          motors[index].currentSpeed += newSpeed;

        setSpeed(motors[index], motors[index].currentSpeed);

        client.print(F("Motor: "));
        client.print(index);
        client.print(F(" set to "));
        client.println(motors[index].currentSpeed);
      }
    }

    // Send feedback to client
    client.println("Status: 200 OK\n");
    client.print(EOL);    //char terminator
  }
  else if (commandoValue == 6)
  {
    MotorDifferenceMarginal = client.parseInt();
    
    client.print("MotorDifferenceMarginal upset to: ");
    client.println(MotorDifferenceMarginal);
    client.println("Status: 200 OK\n");
    client.print(EOL);
  }
  else if (commandoValue == 7)
  {
    int OnOff = 0;
    if (client.read() == '/') {
      OnOff = client.parseInt();
    }
    withoutOffsets = (OnOff == 1);

    client.print("Offsets ");
    client.println(withoutOffsets ? "ON" : "OFF");
    client.println("Status: 200 OK\n");
    client.print(EOL);
  }
  else if (commandoValue == 8)
  {
    int OnOff = 0;
    if (client.read() == '/') {
      OnOff = client.parseInt();
    }
    rinding = (OnOff == 1);

    client.print("Riding ");
    client.println(rinding ? "ON" : "OFF");
    client.println("Status: 200 OK\n");
    client.print(EOL);
  }
  else if (commandoValue == 9)
  {
    reportBackVoltage(client);
  }
  else if (commandoValue == 5)
  {
    int OnOff = 0;
    if (client.read() == '/') {
      OnOff = client.parseInt();
    }

    if (OnOff == 1)
    {
      digitalWrite(PIN_RELAY, HIGH);
      client.println("Motors ON");
    }
    else
    {
      SetMortosToZero();
      digitalWrite(PIN_RELAY, LOW);
      client.println("Motors OFF");
    }

    client.println("Status: 200 OK\n");
    client.print(EOL);

  }
}

void setSpeed(Motor motor, int newSpeed)
{
  if (newSpeed > 0)
  {
    motor.esc.write(newSpeed + motor.balancePitch + motor.balanceRoll);
    delay(20);
  }
}

void SpeedUpOrDown(int commandoValue, int newSpeed,WifiData client)
{

  String motrosStringPins = String(commandoValue);
  /*
    client.print(F("PinsString: "));
    client.print(motrosStringPins);
    client.print(F(". Value to add: "));
    client.println(newSpeed);
  */
  for (int8_t index = 0; index < 4; index++)
  {
    /*
        client.print(F("Loop string part: "));
        client.println(motrosStringPins.substring(index, index + 1));
    */

    if (motrosStringPins.substring(index, index + 1) == "1")
    {
      if (motors[index].currentSpeed + newSpeed < ESC_LOW_DEFAULT)
        motors[index].currentSpeed = ESC_LOW_DEFAULT;
      else if (motors[index].currentSpeed + newSpeed > ESC_HIGH_DEFAULT)
        motors[index].currentSpeed = ESC_HIGH_DEFAULT;
      else
        motors[index].currentSpeed += newSpeed;

      setSpeed(motors[index], motors[index].currentSpeed);

        client.print(F("Motor: "));
        client.print(index);
        client.print(F(" upset to "));
        client.println(motors[index].currentSpeed);
      
    }
  }
}


void reportBackVoltage(WifiData client)
{

  long vcc = readVcc();

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
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void Motors_BalanceRoll(int8_t addingSpeed, bool toLeft, WifiData client)
{
  
  addingSpeed /= 2;

  float multiplicateToSides = toLeft ? 1 : -1;

  if ((motors[0].balanceRoll == addingSpeed * multiplicateToSides))
    return;

  motors[0].balanceRoll = addingSpeed * multiplicateToSides;
  motors[1].balanceRoll = -addingSpeed * multiplicateToSides;
  motors[2].balanceRoll = -addingSpeed * multiplicateToSides;
  motors[3].balanceRoll = addingSpeed * multiplicateToSides;

  ReloadSpeedOnMotors();

  client.print(F("BalanceRoll: "));
  client.print(addingSpeed);
  client.print(F(", isToLeft: "));
  client.println(toLeft);
 
}

void Motors_BalancePitch(int8_t addingSpeed, bool toFront, WifiData client)
{
  addingSpeed /= 2;

  float multiplicateToSides = toFront ? 1 : -1;

  if ((motors[0].balancePitch == addingSpeed * multiplicateToSides))
    return;

  motors[0].balancePitch = addingSpeed * multiplicateToSides;
  motors[1].balancePitch = addingSpeed * multiplicateToSides;
  motors[2].balancePitch = -addingSpeed * multiplicateToSides;
  motors[3].balancePitch = -addingSpeed * multiplicateToSides;

  ReloadSpeedOnMotors();

  client.print(F("BalancePitch: "));
  client.print(addingSpeed);
  client.print(F(", isToFront: "));
  client.println(toFront);
  
}

void ReloadSpeedOnMotors()
{

  for (int index = 0; index < 4; index++)
  {
    motors[index].esc.write(motors[index].currentSpeed + motors[index].balancePitch + motors[index].balanceRoll);
    delay(20);
  }  
}

