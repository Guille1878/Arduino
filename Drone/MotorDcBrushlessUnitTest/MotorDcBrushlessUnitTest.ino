/*
Wille Esteche

// In 1080 and 1090 always staqrt again after it got hang.

*/

#include <Servo.h>

Servo esc1;  // create servo object to control a servo
//Servo esc2;  // create servo object to control a servo

int currentSpeed = 1010;

void setup() {
  
    Serial.begin(9600);
    Serial.setTimeout(500);
    esc1.attach(11);  // attaches the servo on pin 9 to the servo object
    //esc2.attach(10);  // attaches the servo on pin 9 to the servo object

    esc1.writeMicroseconds(1100);
    //esc2.writeMicroseconds(1100);
}

void loop() {

 if (Serial.available() > 0) {

    // Read the new throttle value
    int newSpeed = Serial.parseInt();

    delay(1000);
    
    setSpeed(newSpeed);                // sets the servo position according to the scaled value
  }
  delay(15);                           // waits for the servo to get there
}

void setSpeed(int newSpeed)
{

  

/*
    esc.write(newSpeed);
    
    Serial.print("New speed: ");
    Serial.println(newSpeed);
  
    delay(100); 
  */  
  
  if (newSpeed > 0)
    {
    Serial.print("New speed: ");
    Serial.println(newSpeed);
    //esc.write(newSpeed);
    esc1.writeMicroseconds(newSpeed);
    //esc2.writeMicroseconds(newSpeed);
    delay(500); 
    }
       
}

