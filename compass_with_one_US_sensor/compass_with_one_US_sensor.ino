#include <Servo.h>
#include <Wire.h> //I2C Arduino Library for HMC5883 COMPASS
#define addr 0x1E //I2C Address for The HMC5883

#define DEBUG
#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.print(x)
#else
 #define DEBUG_PRINT(x)
#endif

//#define FOLLOW //Compass follower, vs. heading compensator

//SERVO VARIABLES
Servo wheel;
const int SERVO_PIN = 7;
const int DEFAULT_POS = 90;
int servo_pos = DEFAULT_POS;

//US SENSOR RELEVANT VARIABLES 
  const int TRIG_PIN = 10;
  const int ECHO_PIN = 9;
  const unsigned int MAX_TIME = 23200;  // Anything over 400 cm (10000 us pulse) is "out of range"
  const float PW_TO_CM = 58.0;
  const unsigned int MAX_DIST = MAX_TIME/PW_TO_CM; 
  const float CM_TO_DEG = MAX_DIST/90;
  const int DOGATRON_RADIUS = 5; // cm

void setup() {
  Serial.begin(9600);

  //DIGITAL COMPASS SETUP
  Wire.begin();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
  
  //SERVO SETUP
  wheel.attach(SERVO_PIN);

  //US Sensor
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void loop() {
  // US SENSOR: distance to nearest obstacle in cm (from datasheet and sample code)
  int pulse_width = ping(TRIG_PIN, ECHO_PIN);
  int obstacle_dist = (pulse_width < MAX_TIME) ? (pulse_width/PW_TO_CM) : (MAX_TIME/PW_TO_CM);

  // If impending collision (based on US sensor data), move to the right for 5 seconds
  if(obstacle_dist < DOGATRON_RADIUS) { 
    servo_pos = DEFAULT_POS + 90; // TURN RIGHT
    wheel.write(servo_pos);  
    DEBUG_PRINT("Obstacle distance: "); DEBUG_PRINT(obstacle_dist); DEBUG_PRINT("\n");
    DEBUG_PRINT("Servo position: "); DEBUG_PRINT(servo_pos); DEBUG_PRINT("\n");
    delay(2000); 
  }
  // If no impending collision, move north based on compass data
  else 
  {  
    float heading = getHeading();
    
    //set servo position
    if(heading >= 0 && heading <= 180){
      #ifdef FOLLOW 
      servo_pos = DEFAULT_POS + heading;
      #else
      servo_pos = DEFAULT_POS - heading;
      #endif
    }
    else{
      #ifdef FOLLOW
      servo_pos = DEFAULT_POS - (360-heading);
      #else
      servo_pos = DEFAULT_POS + (360-heading);
      #endif
    }
    
    DEBUG_PRINT("Compass heading: "); DEBUG_PRINT(heading); DEBUG_PRINT("\n");
    DEBUG_PRINT("Servo position: "); DEBUG_PRINT(servo_pos); DEBUG_PRINT("\n");
  
    wheel.write(servo_pos);
  }
}


int ping(int trig, int echo) {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;

  // Hold the trigger pin high for 10 us
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Wait for pulse on echo pin, then measure how long the echo pin was held high (pulse width)
  while (digitalRead(echo) == 0);
  t1 = micros();
  while (digitalRead(echo) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  return pulse_width;
}


float getHeading() {    // GETTING HMC5883 X,Y AND Z VALUES
  int x,y,z; //triple axis data

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  
 //Read the compass data: 2 bytes for each axis, 6 total bytes
  Wire.requestFrom(addr, 6);
  if(6 <= Wire.available()) {
    x = Wire.read()<<8; //MSB  x for reference on bitshift see: https://www.arduino.cc/en/Reference/Bitshift
    x |= Wire.read(); //LSB  x for reference on bitwise OR see: https://www.arduino.cc/en/Reference/BitwiseAnd
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
  }

  //Calculate heading, with range 0-360
  float heading = atan2(y,x);
  heading = (heading<0) ? heading + 2*M_PI : heading;
  heading = heading * 180/M_PI;

  return heading;
}
