/*

Shane Ryan   --   v1.7   --   2/19/2016M
University of Notre Dame Rocketry Team
Altitude Control Software

// TO DO:  average velocity, and 


LIS331 Accelerometer Wiring
Arduino | LIS331
----------------
3V3  | VCC
GND | GND
10  | CS
11  | SDA
12  | SA0
13  | SPC

MPL3115A2 Altimeter Wiring
Arduino | MPL311
----------------
3V3 | VCC
GND | GND
A4  | SDA (through 330 Ohm resistor)
A5  | SCL (through 330 Ohm resistor)

*/

#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

#define SD_PIN 8
#define SD_ENABLE 0 // disabled SD logging because it screwed with SPI bus  <-- duh, you used
                    // pin 10 as chip select for the spi bus.  Needed for sd libs to work.

#define OVERRIDE_PIN 9  // I believe this is the black SPST switch on the shield

#define SOLENOID_PIN 3   // this is the only output we need, i think.  pin 6 has an output too,
                         // but it's not declared for use anywhere in the code :/

#define DESIRED_APOGEE 1634           // defined in meters now... much better
#define DESCENT_DEPLOY_ALT 1238

#define PRE_LAUNCH 0
#define BURN 1
#define BURNOUT 2
#define DESCENT 3
#define PASSIVE_DESCENT 4

#define ALTMODE;
#define ALTBASIS 18 //start altitude to calculate mean sea level pressure in meters
//this altitude must be known (or provided by GPS etc.)

#define DEBUG 1 // enables debug terminal, but slows down program execution

double zlastVel;
double zVel;              // what?  Only zVel is ever used.  Do we need the other values?
double zAccel;

unsigned long currentAltitudeTime;
unsigned long lastAltitudeTime;
unsigned long deltaAltitudeTime; 
unsigned long elapsedTime;

float startingAltitude;
float currentAltitude;
float lastAltitude;
float deltaAltitude;

int state;
int stateNext;
 
const int SENSORADDRESS = 0x60; // address specific to the MPL3115A1, value found in datasheet
 
float altsmooth = 0; //for exponential smoothing
byte IICdata[5] = {0,0,0,0,0}; //buffer for sensor data
 
void setup(){
      Wire.begin(); // join i2c bus
      Serial.begin(9600); // start serial for output
      Serial.println("Setup");
      if(IIC_Read(0x0C) == 196); //checks whether sensor is readable (who_am_i bit)
      else Serial.println("i2c bad");
     
      IIC_Write(0x2D,0); //write altitude offset=0 (because calculation below is based on offset=0)
      //calculate sea level pressure by averaging a few readings
      Serial.println("Pressure calibration...");
      float buff[4];
      for (byte i=0;i<4;i++){
        IIC_Write(0x26, 0b00111011); //bit 2 is one shot mode, bits 4-6 are 128x oversampling
        IIC_Write(0x26, 0b00111001); //must clear oversampling (OST) bit, otherwise update will be once per second
        delay(550); //wait for sensor to read pressure (512ms in datasheet)
        IIC_ReadData(); //read sensor data
        buff[i] = Baro_Read(); //read pressure
        Serial.println(buff[i]);
      }
      float currpress=(buff[0]+buff[1]+buff[2]+buff[3])/4; //average over two seconds
     
      Serial.print("Current pressure: "); Serial.print(currpress); Serial.println(" Pa");
      //calculate pressure at mean sea level based on a given altitude
      float seapress = currpress/pow(1-ALTBASIS*0.0000225577,5.255877);
      Serial.print("Sea level pressure: "); Serial.print(seapress); Serial.println(" Pa");
      Serial.print("Temperature: ");
      Serial.print(IICdata[3]+(float)(IICdata[4]>>4)/16); Serial.println(" C");
     
      // This configuration option calibrates the sensor according to
      // the sea level pressure for the measurement location (2 Pa per LSB)
      IIC_Write(0x14, (unsigned int)(seapress / 2)>>8);//IIC_Write(0x14, 0xC3); // BAR_IN_MSB (register 0x14):
      IIC_Write(0x15, (unsigned int)(seapress / 2)&0xFF);//IIC_Write(0x15, 0xF3); // BAR_IN_LSB (register 0x15):
     
      //one reading seems to take 4ms (datasheet p.33);
      //oversampling 32x=130ms interval between readings seems to be best for 10Hz; slightly too slow
      //first bit is altitude mode (vs. barometer mode)
     
      //Altitude mode
      IIC_Write(0x26, 0b10111011); //bit 2 is one shot mode //0xB9 = 0b10111001
      IIC_Write(0x26, 0b10111001); //must clear oversampling (OST) bit, otherwise update will be once per second
      delay(550); //wait for measurement
      IIC_ReadData(); //
      altsmooth=Alt_Read();
      Serial.print("Altitude now: "); Serial.println(altsmooth);
      Serial.println("Done.");

      pinMode(OVERRIDE_PIN, INPUT);   // pin declarations
      pinMode(SOLENOID_PIN, OUTPUT);
  

   
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

      state = 0;
      stateNext = 0;
      
      // test tab deployment upon startup 
      digitalWrite(SOLENOID_PIN, HIGH);
      
      // tabs retracted 
      delay(1000); 
         
      // tabs deploy
      digitalWrite(SOLENOID_PIN, LOW);
      delay(2000); 
      
      //repeat retract, deploy, retract
      digitalWrite(SOLENOID_PIN, HIGH);
      delay(2000); 
   
      digitalWrite(SOLENOID_PIN, LOW);
      delay(2000); 
      digitalWrite(SOLENOID_PIN, HIGH);

      startingAltitude = sensor_read_data();
}
 
void loop(){
      // update global altitude variables and return zVelocity
      currentAltitude = sensor_read_data();
    
      // predict apogee */
      double apogee;
      apogee = calculateApogee(currentAltitude, zVel);
      // ^we're using altimeter and timers to calculate zvel, no x and y accel or vel??

      elapsedTime = micros();
    
      // print to terminal
      if(DEBUG){
        Serial.print("\nalt: ");
        Serial.print(currentAltitude);
        Serial.print("\tapogee: ");
        Serial.print(apogee);
        Serial.print("\tzVel: ");
        Serial.print(zVel);
        Serial.print("\tzAcc: ");
        Serial.print(zAccel);
      }
      
      // disable data logging for test flight
      // Log data to SD card
      if(SD_ENABLE){
        File fd = SD.open("datalog.txt", FILE_WRITE);
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
          if(abs(zAccel) >= 12.0){
            stateNext = BURN;
          }else{
            stateNext = PRE_LAUNCH;
          }
          break;
        case BURN:
          digitalWrite(SOLENOID_PIN, HIGH);
          if(abs(zAccel) <= 20.0){
            stateNext = BURNOUT;
          }else{
            stateNext = BURN;
          }
          break;
        case BURNOUT:
          if(apogee < (startingAltitude + DESIRED_APOGEE)){
            digitalWrite(SOLENOID_PIN, HIGH); //keep pin HIGH to close tabs when w/in desired apogee 
          }else{
            digitalWrite(SOLENOID_PIN, LOW ); // deploy tabs while descresed vel 
          }
          if(zAccel < 0){
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
  // check open rocket for coefficient
  double rho = 0.065;       // will be updated
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

 
double sensor_read_data(){
  // This function reads the altitude (or barometer) and temperature registers, then prints their values
  // variables for the calculations
  double timeSeconds;
  int m_temp;
  float l_temp;
  float altbaro, temperature;
 
  //One shot mode at 0b10101011 is slightly too fast, but better than wasting sensor cycles that increase precision
  //one reading seems to take 4ms (datasheet p.33);
  //oversampling at 32x=130ms interval between readings seems to be optimal for 10Hz
  #ifdef ALTMODE //Altitude mode
    IIC_Write(0x26, 0b10111011); //bit 2 is one shot mode //0xB9 = 0b10111001
    IIC_Write(0x26, 0b10111001); //must clear oversampling (OST) bit, otherwise update will be once per second
  #else //Barometer mode
    IIC_Write(0x26, 0b00111011); //bit 2 is one shot mode //0xB9 = 0b10111001
    IIC_Write(0x26, 0b00111001); //must clear oversampling (OST) bit, otherwise update will be once per second
  #endif
  delay(100); //read with 10Hz; drop this if calling from an outer loop
 
  IIC_ReadData(); //reads registers from the sensor
  m_temp = IICdata[3]; //temperature, degrees
  l_temp = (float)(IICdata[4]>>4)/16.0; //temperature, fraction of a degree
  temperature = (float)(m_temp + l_temp);
 
  #ifdef ALTMODE //converts byte data into float; change function to Alt_Read() or Baro_Read()
    lastAltitude = currentAltitude;
    altbaro = Alt_Read();
    currentAltitude = altbaro;
    deltaAltitude = currentAltitude - lastAltitude;
  #else
    altbaro = Baro_Read();
  #endif
 
  altsmooth=(altsmooth*3+altbaro)/4; //exponential smoothing to get a smooth time series

  
  lastAltitudeTime = currentAltitudeTime;
  currentAltitudeTime = micros();
  deltaAltitudeTime = currentAltitudeTime - lastAltitudeTime;
  timeSeconds = ((deltaAltitudeTime) / 1000000.0);
  zlastVel = zVel;
  zVel = abs((deltaAltitude) / (timeSeconds));  
  zAccel = zVel - zlastVel;

  return altsmooth;
}
 
float Baro_Read(){
  //this function takes values from the read buffer and converts them to pressure units
  unsigned long m_altitude = IICdata[0];
  unsigned long c_altitude = IICdata[1];
  float l_altitude = (float)(IICdata[2]>>4)/4; //dividing by 4, since two lowest bits are fractional value
  return((float)(m_altitude<<10 | c_altitude<<2)+l_altitude); //shifting 2 to the left to make room for LSB
}
 
float Alt_Read(){
  //Reads altitude data (if CTRL_REG1 is set to altitude mode)
  int m_altitude = IICdata[0];
  int c_altitude = IICdata[1];
  float l_altitude = (float)(IICdata[2]>>4)/16;
  return((float)((m_altitude << 8)|c_altitude) + l_altitude);
}
 
byte IIC_Read(byte regAddr){
  // This function reads one byte over I2C
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(regAddr); // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. Works in Arduino V1.0.1
  Wire.requestFrom(SENSORADDRESS, 1);
  return Wire.read();
}
 
void IIC_ReadData(){  //Read Altitude/Barometer and Temperature data (5 bytes)
  //This is faster than reading individual register, as the sensor automatically increments the register address,
  //so we just keep reading...
  byte i=0;
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(0x01); // Address of CTRL_REG1
  Wire.endTransmission(false);
  Wire.requestFrom(SENSORADDRESS,5); //read 5 bytes: 3 for altitude or pressure, 2 for temperature
  while(Wire.available()) IICdata[i++] = Wire.read();
}
 
void IIC_Write(byte regAddr, byte value){
  // This function writes one byto over I2C
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}

