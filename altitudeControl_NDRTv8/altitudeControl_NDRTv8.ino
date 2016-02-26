/*

Shane Ryan   --   v1.8   --   2/26/2016
University of Notre Dame Rocketry Team
Altitude Control Software

// TO DO:  average velocity, and implement Chris's new apogee
// calculations and drag coefficients.


Adafruit 10DOF Board Wiring
Arduino | 10DOF
----------------
GND | GND
5V  | Vin
SDA | SDA
SCL | SCL

*/


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <SD.h>
#include <RTClib.h>

// Set chip select pin for SPI.
const int chipSelect = 10;

// Create RTC instance
RTC_DS1307 rtc;

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// Define modes for the writeData function that writes to SD Card.
#define WRITE_GLOBAL 2
#define WRITE_INPUT 1

// function to write to SD Card.
void writeData(float valueToWrite, int mode);

float roll;
float pitch;
float altitude;
float temperature;
float heading;
float currentTime;


void setup()
{
  
  Serial.begin(115200); 
  Serial.print("Initializing SD card...");

  // attempt card initialization.
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");   // our card is now open for data logging.

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // set RTC to the date & time this sketch was compiled.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  accel.begin();
  mag.begin();
  bmp.begin();

  
}




void loop()
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));

    roll = orientation.roll;
    pitch = orientation.pitch;
    heading = orientation.heading;
  }

  // Calculate the altitude using the barometric pressure sensor
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude */
    Serial.print(F("Alt: "));
    altitude = bmp.pressureToAltitude(seaLevelPressure,
                                          bmp_event.pressure,
                                          temperature);
    Serial.print(altitude); 
    Serial.println(F(""));
    /* Display the temperature */
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.println(F("\n"));
  }
  
  delay(100);
  
  writeData(0, WRITE_GLOBAL);
  
}


void writeData(float valueToWrite, int mode)
{
  // make a string for assembling the data to log:
  String dataString = "";

  switch (mode) {
    case WRITE_INPUT:
       // Append data passed to function to the string:
       dataString += String(valueToWrite);
       break;
     case WRITE_GLOBAL:
       // Update global variables onto SD Card.
       dataString += String(roll);
       dataString += String(", ");
       dataString += String(pitch);
       dataString += String(", ");
       dataString += String(heading);
       dataString += String(", ");
       dataString += String(temperature);
       dataString += String(", ");
       dataString += String(altitude);
       dataString += String(", ");
       dataString += String(currentTime);  
       break;     
  }
 
    
  // open file, remember to close:
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

