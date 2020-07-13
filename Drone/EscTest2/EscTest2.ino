/*
Copyright (c) <2014> <Ankur Mohan>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Note: Select the mode (program mode/throttle setting mode/normal mode), run this code, 
then power on the ESC. Don't power on the ESC before this code is executing, otherwise
the ESC will see random values on the PWM pin.
*/


/*
  ++++++++++++++++++++++++++++++++++++++++++
  1 Beep                Brake 
  2 Beep-Beep           Battery Type 
  3 Beep-Beep-Beep      Cut-off mode 
  4 Beep-Beep-Beep-Beep Cut-off Threshold 
  5 Beeeep-             Start-up mode 
  6 Beeeep-Beep         Timing 
  7 Beeeep-Beep-Beep    Set all functions to factory defaults 
  8 Beeeep-Beeeep       Exit

  ++++++++++++++++++++++++++++++++++++++++++
  Brake               Off               On            N.A. 
  Battery Type        Li-ion / Li-poly  NiMh / NiCd   N.A. 
  Cut-off Mode        Soft-cut          Cut-off       N.A. 
  Cut-off Threshold   Low   Medium        High 
  Start Mode          Normal            Soft          Super Soft 
  Timing              Low               Medium        High
  ++++++++++++++++++++++++++++++++++++++++++
*/

// Need the Servo library
#include <Servo.h>

#define NUMMOTORS 1
typedef struct MotorDef
{
    Servo   Motor; 
    int     Pin;   // Indicates the Pin this motor is connected to
};

MotorDef Motors[NUMMOTORS];

// Stores the settings for all ESC. This can be made specific to each ESC, but that's not needed
// for a quadcopter project
typedef struct ESCSettingsDef
{
  int Low;
  int High;
};

ESCSettingsDef ESCSettings; 

//#define PROGRAM_MODE
#define THROTTLE_MODE
//#define NORMAL_MODE

int CurrentSpeed;
int Step = 10;


#define ESC_HIGH_DEFAULT 1000
#define ESC_LOW_DEFAULT 20

void setup()
{
	// Required for I/O from Serial monitor
	Serial.begin(9600);
	Serial.println("Setup: Serial port communication at 9600bps\n");
	// Attach motors to pins
	
  Motors[0].Pin =  12;
  /*
	Motors[1].Pin =  10;
	Motors[2].Pin =  11;
	Motors[3].Pin =  12;
*/
        for(int i = 0; i < NUMMOTORS; i++)
        {
          int pin = Motors[i].Pin;
          Motors[i].Motor.attach(pin);
        }

	// Set the ESC settings to the defaults
	ESCSettings.Low   = ESC_LOW_DEFAULT;
	ESCSettings.High  = ESC_HIGH_DEFAULT;
}

// Read low/high speed for the ESC
void ReadLHSpeed()
{
	Serial.println("Enter Low Speed");
	delay(10);
	while(!Serial.available()){}
	ESCSettings.Low = Serial.parseInt();
	Serial.println("Low Speed is");
	Serial.print(ESCSettings.Low);
	Serial.println("\nEnter High Speed\n");
	delay(10);
	while(!Serial.available()){}
	ESCSettings.High = Serial.parseInt();
	Serial.println("High Speed is");
	Serial.print(ESCSettings.High);
	Serial.println("\n");
 
}

void SetThrottleRange()
{
	Serial.println("In Set Throttle Range mode");
    
	for (int i = 0; i < NUMMOTORS; i++)
	{
    setSpeed(ESCSettings.High,Motors[i].Motor);
    //Motors[i].Motor.writeMicroseconds(ESCSettings.High);
	  //Motors[i].Motor.write(ESCSettings.High);
	}

	Serial.println("Connect the ESC now. After connecting the ESC, you should hear the ESC startup tones. Shortly afterwards, you should hear two beeps indicating that the ESC has registered the high throttle value. Immediately after hearing the two beeps, push any key. If you don't do so in 5 sec, the ESC will go into program mode");

	// Wait for user input
	while (!Serial.available())
	{
	}
	Serial.read();

	Serial.println("\nSetting the low throttle setting. If this happens successfully, you should hear several beeps indicating the input voltage supplied to the ESC followed by a long beep indicating that the low throttle has been set. After this point, push any key to proceed, your ESC is ready to be used");

	for (int i = 0; i < NUMMOTORS; i++)
	{
    setSpeed(ESCSettings.Low, Motors[i].Motor);
    //Motors[i].Motor.writeMicroseconds(ESCSettings.Low);
	  //Motors[i].Motor.write(ESCSettings.Low);
	}

	// Wait for user input
	while (!Serial.available())
	{
	}
	Serial.read();
}

void ProgramESC()
{
	Serial.println("In program mode");

	for (int i = 0; i < NUMMOTORS; i++)
	{
	  setSpeed(ESCSettings.High,Motors[i].Motor);
    //Motors[i].Motor.write(ESCSettings.High);
	}

	Serial.println("Connect the ESC now. After connecting the ESC, you should hear ");
	Serial.println("the ESC startup tones. Shortly afterwards, you should hear two beeps ");
	Serial.println("indicating that the ESC has registered the high throttle value. Now wait ");
	Serial.println("for about 5 seconds and the ESC will enter program mode. Once the ESC is in ");
	Serial.println("program mode, it will play a special tone. AFter this, it will emit a series of ");
	Serial.println("beeps indicating different settings. When you hear the beep corresponding to the ");
	Serial.println("setting you want to change, push a key. This will set the throttle to low and the ");
	Serial.println("ESC will play beeps corresponding to the various values of that setting. As soon as ");
	Serial.println("you hear the right beep, push another key and the throttle will be raised to high ");
	Serial.println("and the value for that setting will be set. The ESC will acknowledge by playing a");
	Serial.println("special tone");

	// After the user inputs a key stroke, set the throttle to low
	while (!Serial.available())
	{
	}
	Serial.read();

	for (int i = 0; i < NUMMOTORS; i++)
	{
    //setSpeed(ESCSettings.Low,Motors[i].Motor);
    Motors[i].Motor.write(ESCSettings.Low);
	}


  Serial.println("Select this setting!");
  
	// After the user inputs a key stroke, set the throttle to high. This will set the value of the setting.
	while (!Serial.available())
	{
	}
	Serial.read();

	for (int i = 0; i < NUMMOTORS; i++)
	{
    setSpeed(ESCSettings.High,Motors[i].Motor);
    Motors[i].Motor.write(ESCSettings.High);
	}

	delay(2000);

	Serial.println("Programming the ESC finished, you can power off the ESC now");

}

// Increase the speed of the motor from low to high as set by the user
void Run()
{
  // Send a low signal initially for normal mode
        for (int i = 0; i < NUMMOTORS; i++)
	{
    setSpeed(ESCSettings.Low,Motors[i].Motor);
	  //Motors[i].Motor.write(ESCSettings.Low);
	}
	Serial.println("Running ESC");
	Serial.println("Step = ");
	Serial.print(Step);
	Serial.println("\nPress 'u' to increase speed, 'd' to reduce speed");

	CurrentSpeed = ESCSettings.Low;
	while (1) {
		while (!Serial.available())
		{
		}
		char currentChar = Serial.read();
		if (currentChar == 'u')
		{
			Serial.println("\nIncreasing motor speed by step");
			if (CurrentSpeed + Step < ESCSettings.High) {
				CurrentSpeed = CurrentSpeed + Step;
				Serial.println("New speed = ");
				Serial.print(CurrentSpeed);
			}

			else
			{
				Serial.println("\nMax speed reached\n");
			}
		}
		if (currentChar == 'd')
		{
			Serial.println("\nDecreasing motor speed by step\n");
			if (CurrentSpeed - Step >= ESCSettings.Low)
			{
				CurrentSpeed = CurrentSpeed - Step;
				Serial.println("New speed = ");
				Serial.print(CurrentSpeed);
			}

			else
			{
				Serial.println("\nMin speed reached\n");
			}
		}
		if (currentChar == 'e')
		{
			Serial.println("\nStopping Motors\n");
			CurrentSpeed = ESCSettings.Low;
		}
   
		for (int i = 0; i < NUMMOTORS; i++)
		{
      setSpeed(CurrentSpeed,Motors[i].Motor);
		  //Motors[i].Motor.write(CurrentSpeed);
		}
	}
}

void loop()
{
#ifdef NORMAL_MODE
	Run();
#elif defined THROTTLE_MODE
	SetThrottleRange();
	Run();
#elif defined PROGRAM_MODE
	ProgramESC();
#endif
  while(1) { } 
}

void setSpeed(int speed, Servo motor)
{
  // speed is from 0 to 100 where 0 is off and 100 is max speed
  // the following maps speed values of 0-1000 to angles from 0-180
  
  int angle = map(speed, ESC_LOW_DEFAULT, ESC_HIGH_DEFAULT, 0, 180);
  //motor.writeMicroseconds(angle);
  motor.write(angle);

}