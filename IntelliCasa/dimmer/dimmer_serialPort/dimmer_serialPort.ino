int AC_pin = 3;//Pin to OptoTriac
byte dim = 0; //Initial brightness level from 0 to 255, change as you like!

void setup() {
  Serial.begin(9600);
  pinMode(AC_pin, OUTPUT);
  attachInterrupt(0, light, FALLING);//When arduino Pin 2 is FALLING from HIGH to LOW, run light procedure!
}

void light() {
  if (Serial.available()) {
    dim = Serial.read();
    if (dim < 1) {
      //Turn TRIAC completely OFF if dim is 0
      digitalWrite(AC_pin, LOW);
    }

    if (dim > 254) { //Turn TRIAC completely ON if dim is 255
      digitalWrite(AC_pin, HIGH);
    }
  }

  if (dim > 0 && dim < 255) {
    //Dimming part, if dim is not 0 and not 255
    delayMicroseconds(34*(255-dim));
    digitalWrite(AC_pin, HIGH);
    delayMicroseconds(500);
    digitalWrite(AC_pin, LOW);
  }
}
void loop() {
}
