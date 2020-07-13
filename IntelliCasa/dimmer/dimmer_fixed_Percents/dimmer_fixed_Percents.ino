
// https://maker.pro/projects/arduino/arduino-lamp-dimmer


int ledPin = A2;
int ledPinControl = 11;
void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(ledPinControl, OUTPUT);
  Serial.begin(9600);
  Serial.println("Serial connection started, waiting for instructions…n0 = Off n1 = 25% n2 =50% n3 = 75% n4 = 100%");

  //digitalWrite(ledPinControl, HIGH);
  
}

int brightness = 0;

void loop ()
{
  digitalWrite(ledPinControl, LOW);
  delay(200);
  analogWrite(ledPin, brightness);
  delay(200);
  digitalWrite(ledPinControl, HIGH);
  Serial.println(brightness);
  brightness += 32;

  if (brightness >  255)
  {
    brightness = 0;    
  }

  delay(2000);


/*
  // check if data has been sent from the computer:
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    brightness += Serial.read();
    
  }
  else
  {
    if (brightness)
    {
      
    Serial.print("ser: ");
      Serial.println(brightness);

      brightness = 0;
      
      // set the brightness of the LED:
      analogWrite(ledPin, brightness);
    }
  }
  */
  /*
  int ser;
  
  if (Serial.available()) {
    ser = Serial.read(); //read serial as a character

    Serial.print("ser: ");
    Serial.println(ser);
    //NOTE because the serial is read as “char” and not “int”, the read value must be compared to character numbers
    //hence the quotes around the numbers in the case statement

    switch (ser)
    {
      case '0':
        analogWrite(ledPin, 0);
        break;

      case '1':
        analogWrite(ledPin, 64);
        break;

      case '2':
        analogWrite(ledPin, 128);
        break;

      case '3':
        analogWrite(ledPin, 192);
        break;

      case '4':
        analogWrite(ledPin, 255);
        break;
      default:
        Serial.println("Invalid entry");

    }
    
  }
  */
}
