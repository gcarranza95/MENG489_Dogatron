#include <Servo.h>
#include <Wire.h> //I2C Arduino Library for HMC5883 COMPASS
#define addr 0x1E //I2C Address for The HMC5883
#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
#endif
//SERVO VARIABLES
Servo wheel;
const int DEFAULT_POS = 90;
int servo_pos = DEFAULT_POS;

//US SENSOR RELEVANT VARIABLES 
  const int TRIG_PIN = 10;
  const int ECHO_PIN = 9;
  const int SERVO_PIN = 7;
  const unsigned int MAX_TIME = 23200;  // Anything over 400 cm (10000 us pulse) is "out of range"
  const float PW_TO_CM = 58.0;
  const unsigned int MAX_DIST = MAX_TIME/PW_TO_CM; 
  const float CM_TO_DEG = MAX_DIST/90;
  const int DOGATRON_RADIUS = 5;

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

  //US Sensor
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void loop() {
// US SENSOR
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float obstacle_dist;
  // Hold the trigger pin high for 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Wait for pulse on echo pin, then measure how long the echo pin was held high (pulse width)
  while ( digitalRead(ECHO_PIN) == 0 );
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1 );
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate and print distance to nearest obstacle in cm (from datasheet and sample code)
  obstacle_dist = (pulse_width < MAX_TIME) ? (pulse_width/PW_TO_CM) : (MAX_TIME/PW_TO_CM);
if(obstacle_dist < DOGATRON_RADIUS){ //if impending collision (based on US sensor data), move to the right for 5 seconds
  servo_pos = DEFAULT_POS + 90; // TURN RIGHT
  wheel.write(servo_pos);  
  #ifdef DEBUG
  Serial.print("Obstacle distance: "); Serial.println(obstacle_dist);
  Serial.print("Servo position: "); Serial.println(servo_pos);
  #endif
  delay(5000); 
}
else //if no impending collision, move north based on compass data
{  
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
    Serial.print("Servo position: "); Serial.println(servo_pos);
    #endif
  
    wheel.write(servo_pos);
}
}
