/*

Nathan Vahrenberg
University of Notre Dame Rocketry Team
Altitude Control Software


LIS331 Accelerometer Wiring
Arduino | LIS331
----------------
3V3	| VCC
GND	| GND
10	| CS
11	| SDA
12	| SA0
13	| SPC

MPL3115A2 Altimeter Wiring
Arduino | MPL311
----------------
3V3	| VCC
GND	| GND
A4	| SDA (through 330 Ohm resistor)
A5	| SCL (through 330 Ohm resistor)

*/

#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

// define connections for LIS331
#define SS 10	// Serial Select -> CS
#define MOSI 11	// Master Out - Slave In -> SDA
#define MISO 12	// Master In - Slave Out -> SA0
#define SCK 13	// Serial Clock -> SPC
#define SCALE 0.0007324 // Scale for values in g's

// define control signals for MPL3115A2
#define STATUS     0x00
#define OUT_P_MSB  0x01
#define WHO_AM_I   0x0C
#define PT_DATA_CFG 0x13
#define CTRL_REG1  0x26
#define MPL3115A2_ADDRESS 0x60 // 7-bit I2C address

// general Arduino definitions
#define SAMPLES_PER_SECOND 10 // *max* samples per second

#define SD_PIN 8
#define SD_ENABLE 1 // disabled SD logging because it screwed with SPI bus

#define OVERRIDE_PIN 9

#define SOLENOID_PIN 3
#define ELECTROMAGNET_PIN 6

#define DUTY_CYCLE 25
#define EM_DUTY_CYCLE 100

#define DESIRED_APOGEE 3000*0.3048

#define PRE_LAUNCH 0
#define BURN 1
#define BURNOUT 2
#define DESCENT 3

#define DEBUG 1 // enables debug terminal, but slows down program execution

double xAcc, yAcc, zAcc;
double xVel, yVel, zVel;
unsigned long startTime;
unsigned long lastTime;
unsigned long altitudeTime;
unsigned long altitudeTimeLast;
float altitude, startingAltitude;

int state;
int stateNext;

void setup(){

	Serial.begin(9600);

        pinMode(OVERRIDE_PIN, INPUT);
        pinMode(SOLENOID_PIN, OUTPUT);
        pinMode(ELECTROMAGNET_PIN, OUTPUT);
	
        // initialize accelerometer
	      SPI_Setup();
	      Accelerometer_Setup();

        // initialize altimeter
        Wire.begin();
        if(IIC_Read(0x0C) != 196){
          Serial.println("**\nError: Altimeter Initialization failed\n**");
        }
        setModeAltimeter();
        setOversampleRate(5);
        enableEventFlags();
        
        
        // initialize SD for data logging
        if(SD_ENABLE){
          pinMode(SD_PIN, OUTPUT);
          if(!SD.begin(SD_PIN)){
            Serial.println("** Warning: SD Initialization failed");
          }
          File fd = SD.open("datalog.txt", FILE_WRITE);
          fd.println();
          fd.println("-----------------------------------------");
          fd.println();
          fd.close();
        }
        
        // get start time (for reference)
	      startTime = micros();
        altitudeTime = micros();

        xVel = 0;
        yVel = 0;
        zVel = 0;
        
        digitalWrite(SOLENOID_PIN, LOW);
        digitalWrite(ELECTROMAGNET_PIN, LOW);
        
        state = 0;
        stateNext = 0;
        
        startingAltitude = readAltitude();

}

// main program loop
void loop(){      
  
  // update global acceleration variables
	readAccelVal();

        // update altitude
        float altitude_last = altitude;
        altitude = readAltitude();
        altitudeTimeLast = altitudeTime;
        altitudeTime = micros();
	
        // update elapsed time
        unsigned long elapsedTime = micros();
        elapsedTime -= startTime;
        
        // calculate velocity
        float zVel = (altitude - altitude_last) / ((altitudeTime-altitudeTimeLast) / 1000000);
        
        // predict apogee
        // double apogee = calculateApogee( altitude, zVel );
        
	// print to terminal
        if(DEBUG){
  	      Serial.print("ax: ");
          Serial.print(xAcc);
	        Serial.print("\tay: ");
	        Serial.print(yAcc);
	        Serial.print("\taz: ");
	        Serial.print(zAcc);
          //Serial.print("\tvx: ");
          //Serial.print(xVel);
          //Serial.print("\tvy: ");
          //Serial.print(yVel);
          Serial.print("\tvz: ");
          Serial.print(zVel);
          Serial.print("\talt: ");
          Serial.print(altitude);
	        Serial.print("\tt (us): ");
	        Serial.print(elapsedTime);
        }
        
        // Log data to SD card
        if(SD_ENABLE){
          File fd = SD.open("datalog.txt", FILE_WRITE);
          fd.print("ax: ");
          fd.print(xAcc);
	        fd.print("\tay: ");
	        fd.print(yAcc);
	        fd.print("\taz: ");
	        fd.print(zAcc);
          fd.print("\talt: ");
          fd.print(altitude);
	        fd.print("\tt (us): ");
	        fd.print(elapsedTime);
          fd.print("\n");
	        fd.close();
        }
        
        /* override disabled for flight
        
        if( digitalRead(OVERRIDE_PIN) == HIGH ){
          analogWrite(SOLENOID_PIN, (DUTY_CYCLE*255/100));
          //digitalWrite(SOLENOID_PIN, HIGH);
        }else{
          digitalWrite(SOLENOID_PIN, LOW); 
        }
        Serial.print("\tOverride: ");
        Serial.println(digitalRead(OVERRIDE_PIN));
        */

        /* controls disabled for logging
        
        switch (state){
          case PRE_LAUNCH:
            digitalWrite(SOLENOID_PIN, LOW);
            digitalWrite(ELECTROMAGNET_PIN, LOW);
            if(abs(zAcc) >= 2.0){
              stateNext = BURN;
            }else{
              stateNext = PRE_LAUNCH;
            }
            break;
          case BURN:
            digitalWrite(SOLENOID_PIN, LOW);
            analogWrite(ELECTROMAGNET_PIN, (EM_DUTY_CYCLE*255/100));
            if(abs(zAcc) <= 2.0){
              stateNext = BURNOUT;
            }else{
              stateNext = BURN;
            }
            break;
          case BURNOUT:
            if(apogee > (startingAltitude + DESIRED_APOGEE)){
              digitalWrite(ELECTROMAGNET_PIN, LOW);
              analogWrite(SOLENOID_PIN, (DUTY_CYCLE*255/100));
            }else{
              digitalWrite(SOLENOID_PIN, LOW);
              analogWrite(ELECTROMAGNET_PIN, (EM_DUTY_CYCLE*255/100));
            }
            if(zVel < 0){
              stateNext = DESCENT;
            }else{
              stateNext = BURNOUT; 
            }
            break;
          case DESCENT:
            digitalWrite(SOLENOID_PIN, LOW);
            analogWrite(ELECTROMAGNET_PIN, (EM_DUTY_CYCLE*255/100));
            stateNext = DESCENT;
            break;
        }
        
        state = stateNext;
        
        */

        // wait and attempt to maintain desired sample speed
        while( (micros()-lastTime) < 1000000/(SAMPLES_PER_SECOND) ){
          
        }
        
        lastTime = elapsedTime;
        

}

// function to calculate and return the estimated maximum altitude
double calculateApogee( float altitude, float zVel ){ 
  
  double apogee;
  
  // rocket design parameters
  double mass = 12.19; // mass in kg
  double D_in = 5.5; // diameter in inches
  double D = D_in*0.0254; // diameter in meters
  double fuselageArea = (3.1415*(D*D)/4); // CSA in m^2
  double cd_0 = 0.46; // drag coefficient
  double rho = 1.225;
  double g = 9.8;
  
  apogee = altitude + (1/2*mass*zVel*zVel)/(cd_0/2*rho*(zVel/2)*(zVel/2) + mass*g);
  
  
  
}

void readAccelVal(){

	byte xAddressByteL = 0x28; // low byte of X acceleration value
	byte readBit = B10000000; // set register read
	byte incrementBit = B01000000; // set register increment
	byte dataByte = xAddressByteL | readBit | incrementBit;
	byte b0 = 0x0;
	
	digitalWrite(SS, LOW); // Set signal low to initialize communication
	delay(1);
	SPI.transfer(dataByte); // request read
	byte xL = SPI.transfer(b0); // get low byte of X accel
	byte xH = SPI.transfer(b0); // get high byte of X accel
	byte yL = SPI.transfer(b0); // get low byte of Y accel
	byte yH = SPI.transfer(b0); // get high byte of Y accel
	byte zL = SPI.transfer(b0); // get low byte of Z accel
	byte zH = SPI.transfer(b0); // get high byte of Z accel
	delay(1);
	digitalWrite(SS, HIGH); // end communication
	
	// merge high and low components of accelerations
	int xVal = (xL | (xH << 8) );
	int yVal = (yL | (yH << 8) );
	int zVal = (zL | (zH << 8) );
	
	// scale and set global vars
	xAcc = xVal * SCALE;
	yAcc = yVal * SCALE;
	zAcc = zVal * SCALE;

}

float readAltitude(){
  
  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  //Wait for PDR bit, indicates we have new pressure data
  int counter = 0;
  while( (IIC_Read(STATUS) & (1<<1)) == 0)
  {
      if(++counter > 100) return(-999); //Error out
      delay(1);
  }
  
  // Read pressure registers
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(OUT_P_MSB);  // Address of data to get
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 3); // Request three bytes

  //Wait for data to become available
  counter = 0;
  while(Wire.available() < 3)
  {
    if(counter++ > 100) return(-999); //Error out
    delay(1);
  }

  byte msb, csb, lsb;
  msb = Wire.read();
  csb = Wire.read();
  lsb = Wire.read();

  toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

  // The least significant bytes l_altitude and l_temp are 4-bit,
  // fractional values, so you must cast the calulation in (float),
  // shift the value over 4 spots to the right and divide by 16 (since 
  // there are 16 values in 4-bits). 
  float tempcsb = (lsb>>4)/16.0;

  float altitude = (float)( (msb << 8) | csb) + tempcsb;

  return(altitude);
}

void SPI_Setup(){

	pinMode(SS, OUTPUT);

	// intialize SPI bus
	SPI.begin();
	
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	
	// Set SPI clock rate to 16MHz / x
	SPI.setClockDivider(SPI_CLOCK_DIV16); // Currently 1MHz
	
}

void Accelerometer_Setup(){

	byte addressByte = 0x20;
	
	// PM2 PM1 PM0 DR1 DR0 Zen Yen Xen
	byte ctrlRegByte = 0x37; // 00111111 : normal mode, 1000Hz, xyz enabled

	// send data for Control Register 1
	digitalWrite(SS, LOW);
	delay(1);
	SPI.transfer(addressByte);
	SPI.transfer(ctrlRegByte);
	delay(1);
	digitalWrite(SS, HIGH);
	
	delay(100);
	
	// write to Control Register 2
	addressByte = 0x21;
	ctrlRegByte = 0x00; // High pass filter off
	
	// Send data for Control Register 2
	digitalWrite(SS, LOW);
	delay(1);
	SPI.transfer(addressByte);
	SPI.transfer(ctrlRegByte);
	delay(1);
	digitalWrite(SS, HIGH);
	
	delay(100);
	
	// write to Control Register 4
	addressByte = 0x23;
	
	// BDU BLE FS1 FS0 STsign 0 ST SIM
	ctrlRegByte = 0x30; // 00110000 : 24G (full scale)
	
	// Send data for Control Register 4
	digitalWrite(SS, LOW);
	delay(1);
	SPI.transfer(addressByte);
	SPI.transfer(ctrlRegByte);
	delay(1);
	digitalWrite(SS, HIGH);

}

void enableEventFlags()
{
  IIC_Write(PT_DATA_CFG, 0x07); // Enable all three pressure and temp event flags 
}

void setModeAltimeter()
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting |= (1<<7); //Set ALT bit
  IIC_Write(CTRL_REG1, tempSetting);
}

void setOversampleRate(byte sampleRate)
{
  if(sampleRate > 7) sampleRate = 7; //OS cannot be larger than 0b.0111
  sampleRate <<= 3; //Align it for the CTRL_REG1 register

  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= 0b11000111; //Clear out old OS bits
  tempSetting |= sampleRate; //Mask in new OS bits
  IIC_Write(CTRL_REG1, tempSetting);
}

void toggleOneShot(void)
{
  byte tempSetting = IIC_Read(CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<1); //Clear OST bit
  IIC_Write(CTRL_REG1, tempSetting);

  tempSetting = IIC_Read(CTRL_REG1); //Read current settings to be safe
  tempSetting |= (1<<1); //Set OST bit
  IIC_Write(CTRL_REG1, tempSetting);
}

byte IIC_Read(byte regAddr)
{
  // This function reads one byte over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);  // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
  Wire.requestFrom(MPL3115A2_ADDRESS, 1); // Request the data...
  return Wire.read();
}

void IIC_Write(byte regAddr, byte value)
{
  // This function writes one byto over IIC
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}
