#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;
const int chipSelect = 4;
String dataString = "";

/*
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
*/

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 50 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

#define DECLINATION 6.34 // Declination (degrees) in Boulder, CO.

// #define WithSERIALPRINT

void setup() {

 #ifdef WithSERIALPRINT
 
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only 1
  }
  Serial.println("Initializing SD card...");
#endif

/*
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
*/

  Wire.begin();
  if (imu.begin() == false)
  {
    #ifdef WithSERIALPRINT
    Serial.println("Failed to communicate with LSM9DS1.");
    #endif
    /*
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
                  */
    while (1);
  }
  Serial.println("Initialized!");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    #ifdef WithSERIALPRINT
    Serial.println("Card failed, or not present");
    // don't do anything more:
    #endif
    while (1);
  }

  engraveGyroHeaders();
  
  #ifdef WithSERIALPRINT
   Serial.println("Headers just wroten!");
  #endif
}

void loop() {

   // make a string for assembling the data to log:
  
  updateGyroBasicValues();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    /*
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    */
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    engraveGyroMetrics();
        
    lastPrint = millis(); // Update lastPrint time
  }   
}

void engraveGyroHeaders()
{
  engrave("Pitch;Roll;Heading;Gyro-X;Gyro-Y;Gyro-Z;Acc-X;Acc-Y;Acc-Z;");
}

void engrave(String data)
{
  if (data == "")
    return;
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("gyrolog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
    // print to the serial port too:
    #ifdef WithSERIALPRINT
    Serial.println(data);
    #endif
  }
  // if the file isn't open, pop up an error:
  else {
    #ifdef WithSERIALPRINT
    Serial.println("error opening gyrolog.csv");
    #endif
  }
}

void updateGyroBasicValues()
{
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
}

void engraveGyroMetrics()
{
  dataString = "";
  
  float ax = imu.ax;
  float ay = imu.ay;
  float az = imu.az;
  float mz = imu.mz;
  float my = -imu.mx;
  float mx = -imu.my;

  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  dataString += String(pitch);
  dataString += ";";
  dataString += String(roll);
  dataString += ";";                 
  dataString += String(heading);
  dataString += ";";                 

  dataString += String(imu.calcGyro(imu.gx));
  dataString += ";"; 
  dataString += String(imu.calcGyro(imu.gy));
  dataString += ";";
  dataString += String(imu.calcGyro(imu.gz));
  dataString += ";";

  dataString += String(imu.calcGyro(imu.ax));
  dataString += ";"; 
  dataString += String(imu.calcGyro(imu.ay));
  dataString += ";";
  dataString += String(imu.calcGyro(imu.az));
  dataString += ";";
    
  if (dataString != "")
    engrave(dataString);
}

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  dataString += "G: ";
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  dataString += String(imu.calcGyro(imu.gx));
  dataString += ";"; 
  dataString += String(imu.calcGyro(imu.gy));
  dataString += ";";
  dataString += String(imu.calcGyro(imu.gz));
  
#elif defined PRINT_RAW
  dataString += String(imu.gx));
  dataString += ";";
  dataString += String(imu.gy));
  dataString += ";";
  dataString += String(imu.gz));
#endif
  dataString += ";";
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
 dataString += "A: ";
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  dataString += String(imu.calcAccel(imu.ax));
  dataString += ";";
  dataString += String(imu.calcAccel(imu.ay));
  dataString += ";";
  dataString += String(imu.calcAccel(imu.az));
  
#elif defined PRINT_RAW 
  dataString += String(imu.ax));
  dataString += ";";
  dataString += String(imu.ay));
  dataString += ";";
  dataString += String(imu.az));
#endif
  dataString += ";";

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  dataString += "M: ";
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  dataString += String(imu.calcMag(imu.mx));
  dataString += ";";
  dataString += String(imu.calcMag(imu.my));
  dataString += ";";
  dataString += String(imu.calcMag(imu.mz));
  dataString += " gauss";
#elif defined PRINT_RAW
  dataString += String(imu.mx));
  dataString += ";";
  dataString += String(imu.my));
  dataString += ";";
  dataString += String(imu.mz));
#endif
  dataString += ";";
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  dataString += "Pitch, Roll: ";
  dataString += String(pitch);
  dataString += ";";
  dataString += String(roll);
  dataString += ";Heading: "; 
  dataString += String(heading);

  dataString += ";";
}
