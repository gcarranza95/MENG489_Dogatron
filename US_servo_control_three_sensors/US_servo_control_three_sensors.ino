/*
 * Control a servo with three ultrasonic proximity sensors to model an obstacle-avoiding device. 
 * The closer an obstacle is to the sensor, the greater the angle of the servo from 90 degrees, 
 * ping function based on HC-SR04 Demo: Demonstration of the HC-SR04 Ultrasonic Sensor (August 3, 2016; license: Public Domain)
 */

#include <Servo.h>

// Servo setup
Servo dogatron;
const int TARGET_POS = 90;
const int SERVO_PIN = 7;
int servo_pos = TARGET_POS;

// US sensor setup: 0=left, 1=center, 2=right
const unsigned int NUM_SENSORS = 3;
const int TRIG_PINS[NUM_SENSORS] = {8,10,12};
const int ECHO_PINS[NUM_SENSORS] = {9,11,13};
const int LFT = 0;
const int CTR = 1;
const int RGT = 2; 

int obst_dist[3]; // Obstacle distance from given sensor
int obst_bool[3]; // Is obstacle present at given sensor?
int turn_dir = 0;

const unsigned int MAX_TIME = 5800;  // (us) Anything over this value is "out of range"
const float PW_TO_CM = 58.0;
const unsigned int MAX_DIST = MAX_TIME/PW_TO_CM; 
const float CM_TO_DEG = MAX_DIST/90;

void setup() {

  dogatron.attach(SERVO_PIN);

  for(int i = 0; i < NUM_SENSORS; i++) {
    int pin = TRIG_PINS[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  Serial.begin(9600);
}

void loop() {

  int pw;
  float obst_dist[NUM_SENSORS];
  bool obst_bool[NUM_SENSORS];
  float servo_pos;

  // Get current obstacle distances (Note that this takes 3*(10 + wait + pw) us)
  // May want to change to doing center ping first, then both side pings at once?
  for(int i = 0; i < NUM_SENSORS; i++) {
    pw = ping(TRIG_PINS[i], ECHO_PINS[i]);
    obst_dist[i] = (pw < MAX_TIME) ? (pw/PW_TO_CM) : (MAX_TIME/PW_TO_CM);
    obst_bool[i] = (pw < MAX_TIME) ? true : false;
  }

  //Choose direction: 0 = forward, 1 = right, -1 = left
  turn_dir = obst_bool[CTR] ? ((obst_dist[LFT] < obst_dist[RGT]) ? 1 : -1) : 0;

  // Calculate and set dogatron position based on obstacle distance
  servo_pos = TARGET_POS + turn_dir*(MAX_DIST - obst_dist[CTR])/CM_TO_DEG;
  dogatron.write(servo_pos);

  // Display on serial monitor for troubleshooting
  for(int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(obst_dist[i]);
    Serial.print("\t");    
  }
  Serial.print("\t\t");
  Serial.print(servo_pos);
  Serial.print("\n");  
  
  // Wait before next measurement
  delay(40);
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
  while ( digitalRead(echo) == 0 );
  t1 = micros();
  while ( digitalRead(echo) == 1 );
  t2 = micros();
  pulse_width = t2 - t1;

  return pulse_width;
}
