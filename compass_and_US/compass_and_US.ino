/* Final combined Dogatron code, incorporating compass and three ultrasonic sensors.
 * Based on HC-SR04 Demo: Demonstration of the HC-SR04 Ultrasonic Sensor (August 3, 2016; license: Public Domain)
 */

#define DEBUG
#ifdef DEBUG
 #define PRINT(x)  Serial.print(x)
#else
 #define PRINT(x)
#endif

#define ULTRASONIC_IN_USE
#define SERVO_IN_USE
#define COMPASS_IN_USE
#define SPEAKER_IN_USE

#ifdef SERVO_IN_USE 
#include <Servo.h>
Servo wheel;
const int SERVO_PIN = 7;
const int DEFAULT_POS = 90;
const int MAX_POS_CHANGE = 30;
int servo_pos = DEFAULT_POS;
#endif

#ifdef ULTRASONIC_IN_USE
const int NUM_SENSORS = 3;
const int TRIG_PINS(NUM_SENSORS) = {8,10,12};
const int ECHO_PINS(NUM_SENSORS) = {9,11,13};
const int LEFT, CENTER, RIGHT = 0, 1, 2;

const unsigned int MAX_DIST = 200;  // (cm); anything over this value is "out of range"
const float PW_TO_CM = 58.0;        // (us/cm)
const unsigned int MAX_TIME = MAX_DIST*PW_TO_CM; // (us)
const float CM_TO_DEG = MAX_DIST/90;
#endif

#ifdef COMPASS_IN_USE
#define addr 0x1E  //I2C Address for The HMC5883
#include <Wire.h>  //I2C Arduino Library for HMC5883 COMPASS
#endif

#ifdef SPEAKER_IN_USE
const int SPEAKER_PIN = 3;
#endif

/*------------------------------------------SETUP----------------------------------------------
 */
void setup(){

Serial.begin(9600);

#ifdef SPEAKER_IN_USE
pinMode(SPEAKER_PIN, OUTPUT);
#endif

#ifdef SERVO_IN_USE
wheel.attach(SERVO_PIN);
#endif
  
#ifdef COMPASS_IN_USE
Wire.begin();
Wire.beginTransmission(addr); // Start talking
Wire.write(0x02);             // Set register
Wire.write(0x00);             // Tell the HMC5883 to continuously measure
Wire.endTransmission();  
#endif

#ifdef ULTRASONIC_IN_USE
pinMode(TRIG_PIN, OUTPUT);
digitalWrite(TRIG_PIN, LOW);
#endif
}


/*------------------------------------------LOOP----------------------------------------------
 */
void loop() {

  #ifdef ULTRASONIC_IN_USE
  float obstacle_dist = ping(TRIG_PIN, ECHO_PIN);
  PRINT(obstacle_dist); PRINT(" cm\n");

  #ifdef LEFT, RIGHT
  //Choose direction: 0 = forward, 1 = right, -1 = left
  turn_dir = obst_bool[CTR] ? ((obst_dist[LFT] < obst_dist[RGT]) ? 1 : -1) : 0;
  #endif
  #endif

  int pw;
  float obst_dist[NUM_SENSORS];
  bool obst_bool[NUM_SENSORS];
  float servo_pos;

  
  // Get current obstacle distances (Note that this takes roughly 3*(10 + pw) us)
  // May want to change to doing center ping first, then both side pings at once?
  for(int i = 0; i < NUM_SENSORS; i++) {
    pw = ping(TRIG_PINS[i], ECHO_PINS[i]);
    obst_dist[i] = (pw < MAX_TIME) ? (pw/PW_TO_CM) : (MAX_TIME/PW_TO_CM);
    obst_bool[i] = (pw < MAX_TIME) ? true : false;
  }



  // Calculate and set dogatron position based on obstacle distance
  servo_pos = TARGET_POS + turn_dir*(MAX_DIST - obst_dist[CTR])/CM_TO_DEG;
  dogatron.write(servo_pos);

  // Get compass heading
  #ifdef COMPASS_IN_USE
  float heading = getHeading();  
  #endif

  #ifdef SERVO_IN_USE
  // Calculate and set wheel position based on obstacle distance
  servo_pos = DEFAULT_POS + (MAX_DIST - obstacle_dist)/CM_TO_DEG;
  wheel.write(servo_pos);
  #endif

  // Wait indicated sampling time before next measurement
  delay(SAMPLE_DELAY);
}


/*------------------------------------------PING----------------------------------------------
 */
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

/*------------------------------------------GET HEADING-----------------------------------------
 */
float getHeading() {    // GET HMC5883 COMPASS X, Y AND Z VALUES
  int x,y,z; //triple axis data

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  
  //Read the compass data: 2 bytes for each axis, 6 total bytes
  Wire.requestFrom(addr, 6);
  if(6 <= Wire.available()) {
    x = Wire.read()<<8;  //MSB x for reference on bitshift see: https://www.arduino.cc/en/Reference/Bitshift
    x |= Wire.read();    //LSB x for reference on bitwise OR see: https://www.arduino.cc/en/Reference/BitwiseAnd
    z = Wire.read()<<8;  //MSB z
    z |= Wire.read();    //LSB z
    y = Wire.read()<<8;  //MSB y
    y |= Wire.read();    //LSB y
  }

  //Calculate compass heading (range = 0-360 degrees)
  float heading = atan2(y,x) * 180/M_PI;
  return heading;
}

/*------------------------------------------BUZZ-----------------------------------------
 */
void buzz(int pin, long frequency, long len) {
  long delayVal = 1000000/frequency/2; // Delay value between transitions; unitless (1 second in us)/(1/s)
  long numCycles = frequency*len/1000; // Number of cycles; unitless (1/s)/(s)
  
  for (long i=0; i < numCycles; i++) {
    digitalWrite(pin, HIGH);      // high (push out the diaphram)
    delayMicroseconds(delayVal);  // wait  
    digitalWrite(pin, LOW);       // low (pull back the diaphram)
    delayMicroseconds(delayVal);  // wait  
  }
}
