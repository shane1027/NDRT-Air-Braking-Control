/*

Shane Ryan   --   v1.4   --   2/19/2016M
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
#include <SparkFunMPL3115A2.h>    // this makes life much, much easier for altimeter control.

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

double xAcc, yAcc, zAcc;              // Only zAcc seems to be relevant.  xAcc and yAcc calculated later,
                                      // but never used.
double xVel, yVel, zVel;              // what?  Only zVel is ever used.  Do we need the other values?
unsigned long currentAltitudeTime;
unsigned long lastAltitudeTime;
unsigned long deltaAltitudeTime;
float startingAltitude;
float currentAltitude;
float lastAltitude;
float deltaAltitude;

int state;
int stateNext;

MPL3115A2 myAltimeter;            // instance of the MPL3115A2 object for altimeter usage.

void setup(){

      Serial.begin(9600);             // serial port baud rate  
}

// main program loop
void loop()
{      
      // update global acceleration variables
      // readAccelVal();
    
      // update global altitude variables and return zVelocity
      // zVel = readAltVal();
    
      // predict apogee */
      while (Serial.available()) {
           Serial.println("Enter current altitude:  ");
           currentAltitude = Serial.parseInt();
           Serial.println("Enter current zVelocity:  ");
           zVel = 58;
           Serial.println();
           Serial.print("\nAltitude:  ");
           Serial.print(currentAltitude);
           Serial.print("\nVelocity:  ");
           Serial.print(zVel);
           double apogee = calculateApogee(currentAltitude, zVel);
           Serial.print("\nCalculated Apogee:  ");
           Serial.print(apogee);
      // ^we're using altimeter and timers to calculate zvel, no x and y accel or vel??
      }
}



// new function grounded in science to calculate apogee.
double calculateApogee( double altitude, double zVel ){ 
  
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

//  Serial.println();
//  Serial.print("D:  ");
//  Serial.print(D);
//  Serial.println();
//  Serial.print("fuselageArea:  ");
//  Serial.print(fuselageArea);
//
//  Serial.println();
//  Serial.print("zVel:   ");
//  Serial.print(zVel);
//
//  Serial.println();
//  Serial.print("rho:  ");
//  Serial.print(rho);

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

