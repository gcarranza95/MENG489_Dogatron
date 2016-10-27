#include <Servo.h>
#include <Wire.h> //I2C Arduino Library for HMC5883 COMPASS
#define addr 0x1E //I2C Address for The HMC5883
#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
#endif

Servo wheel;
const int DEFAULT_POS = 90;
int servo_pos = DEFAULT_POS;

//US SENSOR RELEVANT PINS 
const int TRIG_PIN = -1;
const int ECHO_PIN = -1;
const int SERVO_PIN = 7;

void setup() {
  Serial.begin(9600); //baud rate

  //DIGITAL COMPASS SETUP
  Wire.begin();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
  
  //SERVO SETUP
  wheel.attach(SERVO_PIN);
}

void loop() {
// GETTING HMC5883 X,Y AND Z VALUES
  int x,y,z; //triple axis data

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  
 
 //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //MSB  x for reference on bitshift see: https://www.arduino.cc/en/Reference/Bitshift
    x |= Wire.read(); //LSB  x for reference on bitwise OR see: https://www.arduino.cc/en/Reference/BitwiseAnd
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
  }
  
  float heading = atan2(y,x);
  if(heading<0)
  heading += 2*M_PI;
  float headingDegrees = heading * 180/M_PI; //range of headingDegrees = 0-360
  //set servo_pos
  if(headingDegrees >= 0 && headingDegrees <=180){
    //servo_pos = DEFAULT_POS + headingDegrees; //compass follower
    servo_pos = DEFAULT_POS - headingDegrees; //direction compensator
  }
  else{
    //servo_pos = DEFAULT_POS - (360-headingDegrees); //compass follower
    servo_pos = DEFAULT_POS + (360-headingDegrees); //direction compensator
  }
  #ifdef DEBUG
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  #endif

  wheel.write(servo_pos);
}
