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

int currentSpeeds[4] = { 1010, 1010, 1010, 1010};

void setup() {

  Wifi.begin();
  //Wifi.println("REST Server is up");

    escs[0].attach(9);  // attaches the servo on pin 9 to the servo object
    escs[1].attach(10);  // attaches the servo on pin 9 to the servo object
    escs[2].attach(11);  // attaches the servo on pin 9 to the servo object
    escs[3].attach(12);  // attaches the servo on pin 9 to the servo object

  

  for (short motorIndex = 0; motorIndex < 4; motorIndex++)
  {
    escs[motorIndex].writeMicroseconds(1080);
  }
  //0 blink each
  escs[3].writeMicroseconds(1500);
  escs[2].writeMicroseconds(1500);
  
 
}

void loop() {

  while(Wifi.available()){
      process(Wifi);
    }
  delay(50);

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
 
  // If the next character is a '/' it means we have an URL
  // with a value like: "/digital/2/1050"
  if (client.read() == '/') {
    newSpeed = client.parseInt();
  }
  else {
    newSpeed = 1010;
  }
  
    currentSpeeds[motorIndex] = newSpeed;
    setSpeed(escs[motorIndex], newSpeed);
 
  // Send feedback to client
  client.println("Status: 200 OK\n");
  client.print(F("Motor: "));
  client.print(motorIndex);
  client.print(F(" set to "));
  client.println(newSpeed);
  client.print(EOL);    //char terminator
 
}

void setSpeed(Servo motor, int newSpeed)
{

  

/*
    esc.write(newSpeed);
    
    Serial.print("New speed: ");
    Serial.println(newSpeed);
  
    delay(100); 
  */  
  
  if (newSpeed > 0)
    {
      
    //Serial.print("New speed: ");
    //Serial.println(newSpeed);

    motor.writeMicroseconds(newSpeed);
    delay(20); 
    }
}

