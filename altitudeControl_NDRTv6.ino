/*

Shane Ryan   --   v1.6   --   2/19/2016M
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
#include <SparkFunMPL3115A2.h>

// define connections for LIS331
#define SS 10	// Serial Select -> CS
#define MOSI 11	// Master Out - Slave In -> SDA
#define MISO 12	// Master In - Slave Out -> SA0
#define SCK 13	// Serial Clock -> SPC
#define SCALE 0.0007324 // Scale for values in g's   <-- we have bad accel values.  culprit?

// general Arduino definitions
#define SAMPLES_PER_SECOND 10 // *max* samples per second   -- hmm, I may change this.

#define SD_PIN 8
#define SD_ENABLE 0 // disabled SD logging because it screwed with SPI bus  <-- duh, you used
                    // pin 10 as chip select for the spi bus.  Needed for sd libs to work.

#define OVERRIDE_PIN 9  // I believe this is the black SPST switch on the shield

#define SOLENOID_PIN 3   // this is the only output we need, i think.  pin 6 has an output too,
                         // but it's not declared for use anywhere in the code :/

#define DUTY_CYCLE 25    // not sure why he's mucking with this low-level stuff, we have libs.

#define DESIRED_APOGEE 1634           // defined in meters now... much better
#define DESCENT_DEPLOY_ALT 1238

#define PRE_LAUNCH 0
#define BURN 1
#define BURNOUT 2
#define DESCENT 3
#define PASSIVE_DESCENT 4

#define DEBUG 1 // enables debug terminal, but slows down program execution

// WHY SO MANY GLOBAL VARIABLES?!   bad practice.

double xAcc, yAcc, zAcc;              // Only zAcc seems to be relevant.  xAcc and yAcc calculated later, but never used.
                                      
double xVel, yVel, zVel;              // what?  Only zVel is ever used.  Do we need the other values?

unsigned long currentAltitudeTime;
unsigned long lastAltitudeTime;
unsigned long deltaAltitudeTime;             // must be a floating pt, to manipulate w/o integer division
unsigned long elapsedTime;

float startingAltitude;
float currentAltitude;
float lastAltitude;
float deltaAltitude;

int state;
int stateNext;

MPL3115A2 myAltimeter;            // instance of the MPL3115A2 object for altimeter usage.

void setup()
{ 
      Serial.begin(9600);             // serial port baud rate
  
      pinMode(OVERRIDE_PIN, INPUT);   // pin declarations
      pinMode(SOLENOID_PIN, OUTPUT);
  
      SPI_Setup();
      Accelerometer_Setup();

      //altimeterSetup();               // Efficient altimeter setup        
      
      // disable SD data logging for test flight       <-- we're gonna need to work on this
      // initialize SD for data logging
      if(SD_ENABLE){
        pinMode(SD_PIN, OUTPUT);
        if(!SD.begin(SD_PIN)){
          Serial.println("** Warning: SD Initialization failed");
        }
        File fd = SD.open("datalog.txt", FILE_WRITE);
        fd.println();
        fd.println("-----------------------------------------");
        fd.print("Starting Altitude:  ");
        fd.print(startingAltitude);
        fd.println();
        fd.close();
      }
      
      xVel = 0;
      yVel = 0;
      zVel = 0;
  
      state = 0;
      stateNext = 0;
      
      // test tab deployment upon startup 
      digitalWrite(SOLENOID_PIN, HIGH);
      
      // tabs retracted 
      delay(1000); 
//      
//      // tabs deploy
//      digitalWrite(SOLENOID_PIN, LOW);
//      delay(2000); 
//      
//      //repeat retract, deploy, retract
//      digitalWrite(SOLENOID_PIN, HIGH);
//      delay(2000); 
//   
//      digitalWrite(SOLENOID_PIN, LOW);
//      delay(2000); 
//      digitalWrite(SOLENOID_PIN, HIGH);
//  
//      currentAltitudeTime = micros();        
//      Serial.println("hey I got here - 1");

      for (int i = 0; i < 15; i++) {
        startingAltitude = myAltimeter.readAltitude();
        currentAltitude = startingAltitude;
        delay(100);
      }
}


// main program loop
void loop()
{      
      // update global altitude variables and return zVelocity
//      zVel = readAltVal();

      // update global acceleration variables
      readAccelVal();
    
      // predict apogee */
      double apogee;
      apogee = calculateApogee(currentAltitude, zVel);
      // ^we're using altimeter and timers to calculate zvel, no x and y accel or vel??

      elapsedTime = micros();
    
      // print to terminal
      if(DEBUG){
        Serial.print("\nax: ");
        Serial.print(xAcc);
        Serial.print("\tay: ");
        Serial.print(yAcc);
        Serial.print("\taz: ");
        Serial.print(zAcc);
//        Serial.print("\nvx: ");
//        Serial.print(xVel);
//        Serial.print("\tvy: ");
//        Serial.print(yVel);
//        Serial.print("\tvz: ");
//        Serial.print(zVel);
          Serial.print("\talt: ");
          Serial.print(currentAltitude);
          Serial.print("\tapogee: ");
          Serial.print(apogee);
          Serial.print("\tzVel: ");
          Serial.print(zVel);
//        Serial.print("\nt (us): ");
//        Serial.print(elapsedTime);
      }
      
      // disable data logging for test flight
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
        fd.print(currentAltitude);
        fd.print("\tt (us): ");
        fd.print(elapsedTime);
        fd.print("\n");
        fd.close();
      }
    
    
      //controls NOT disabled for logging    <-- what does this mean?
      
      switch (state){
        case PRE_LAUNCH:
          digitalWrite(SOLENOID_PIN, HIGH);
          if(abs(zAcc) >= 2.0){
            stateNext = BURN;
          }else{
            stateNext = PRE_LAUNCH;
          }
          break;
        case BURN:
          digitalWrite(SOLENOID_PIN, HIGH);
          if(abs(zAcc) <= 2.0){
            stateNext = BURNOUT;
          }else{
            stateNext = BURN;
          }
          break;
        case BURNOUT:
          if(apogee > (startingAltitude + DESIRED_APOGEE + (5*0.3094))){
            digitalWrite(SOLENOID_PIN, HIGH); //keep pin HIGH to close tabs when w/in desired apogee 
          }else{
            digitalWrite(SOLENOID_PIN, LOW ); // deploy tabs while descresed vel 
          }
          if(zVel < 0){
            stateNext = DESCENT;
          }else{
            stateNext = BURNOUT; 
          }
          break;
        case DESCENT:
          digitalWrite(SOLENOID_PIN, HIGH); // retract tabs during initial descent 
          if(currentAltitude < DESCENT_DEPLOY_ALT){
            stateNext = PASSIVE_DESCENT; 
          }else{
            stateNext = DESCENT;
          }
          break;
        case PASSIVE_DESCENT:
          digitalWrite(SOLENOID_PIN, LOW); // allow tabs to passively deploy
          stateNext = PASSIVE_DESCENT;         
      }
            
      state = stateNext;
}



// new function (grounded in science this time) to calculate apogee.
double calculateApogee( double altitude, double zVel )
{
  double apogee;
  double delta_x;
  double drag_accel;
  
  // rocket design parameters
  double mass = 12.19;      // mass in kg - this must have changed since last year
  double D_in = 5.525;      // diameter in inches - may be 5.525in this year
  double D = D_in*0.0254;   // diameter in meters
  double fuselageArea = (PI*(D/2.00)*(D/2.00)); // CSA in m^2
  double cd_0 = 0.46;       // Drag coefficient, which I found is unitless.  where'd it come from?
  double rho = 1.225;       // Looks like this is the mass density of air @ 15deg Celsius
  double g = 9.8;           // Gravity, duh!

  // total acceleration rocket experiences at this small dt.  Calculate drag force, divide by mass.
  drag_accel = -(rho*zVel*zVel*cd_0*fuselageArea/2/mass);// *cd_0*fuselageArea) / (2.00*mass);

//  Serial.println();
//  Serial.print("drag_accel:  ");
//  Serial.print(drag_accel);  

  // predicted delta_x before stopping rocket = -(V^2)/(2a).
  delta_x = -(zVel*zVel) / (2.00*(-g+drag_accel));  
  
//  Serial.println();
//  Serial.print(delta_x);

  apogee = altitude + delta_x;

  return apogee;
}



// The below process seems to be the only way to get viable data from the accelerometer, but
// the sd card logging feature interferes with it...  May have to transmit to comms payload
// directly and log it there.

void readAccelVal()
{
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




double readAltVal()
{ 
  double timeSeconds;
  
  // update altitude
  lastAltitude = currentAltitude;
  currentAltitude = abs(myAltimeter.readAltitude() - startingAltitude);   // read altitude in meters
  deltaAltitude = abs(currentAltitude - lastAltitude);
  
  // update time between measurements
  lastAltitudeTime = currentAltitudeTime;
  currentAltitudeTime = micros();
  deltaAltitudeTime = currentAltitudeTime - lastAltitudeTime;

  Serial.print("\ttime: ");
  Serial.print(deltaAltitudeTime);

  Serial.print("\tdeltaAlt: ");
  Serial.print(deltaAltitude);
  
  // calculate velocity
  timeSeconds = ((deltaAltitudeTime) / 1000000.0);
  float zVel = (deltaAltitude) / (timeSeconds);
  
  return zVel;
}



void SPI_Setup()
{
	pinMode(SS, OUTPUT);

	// intialize SPI bus
	SPI.begin();
	
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	
	// Set SPI clock rate to 16MHz / x
	SPI.setClockDivider(SPI_CLOCK_DIV16); // Currently 1MHz
}




// This function can be much simpler, bro!!
void Accelerometer_Setup()
{
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



// simple altimeter config
void altimeterSetup()
{
  myAltimeter.begin();              // brings device online
  myAltimeter.setModeAltimeter();   // sets correct mode 
  myAltimeter.setOversampleRate(2); // recommended ratio, gives 512ms b/w samples.  low noise.
  myAltimeter.enableEventFlags();   // enable all event flags.  why not?
}    // holy cow.  wasn't that easy?

