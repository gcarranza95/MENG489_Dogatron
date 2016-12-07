/*
 * Control a servo with an ultrasonic sensor: the closer an obstacle is to the sensor,
 * the greater the angle of the servo from 90 degrees. 
 * Based on HC-SR04 Demo: Demonstration of the HC-SR04 Ultrasonic Sensor (August 3, 2016; license: Public Domain)
 */

//#define SERVO

#ifdef SERVO
#include <Servo.h>
Servo wheel;
const int SERVO_PIN = 7;
const int DEFAULT_POS = 90;
int servo_pos = DEFAULT_POS;
#endif

// One ultrasound sensor only - for testing
const int TRIG_PIN = 10;
const int ECHO_PIN = 11;
const int SAMPLE_DELAY = 60; // (ms)

const unsigned int MAX_DIST = 400;  // (cm); anything over this value is "out of range"
const float PW_TO_CM = 58.0;        // (us/cm)
const unsigned int MAX_TIME = MAX_DIST*PW_TO_CM; // (us)

#ifdef SERVO
const float CM_TO_DEG = MAX_DIST/90;
#endif


/*------------------------------------------SETUP---------------------------------------------
 */
 int reading = 1; 
 int count = 0;
 float sampling_time;
void setup() {

  #ifdef SERVO
  wheel.attach(SERVO_PIN);
  #endif

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(9600);
}

/*------------------------------------------MAIN LOOP-----------------------------------------
 */
void loop() {
  if(reading == 1){
  Serial.print("Time (ms)\t"); Serial.print("Distance (cm)\t\n");
  }
  while (count < 100 && reading == 1){
  sampling_time = millis();
  unsigned long pulse_width = ping(TRIG_PIN, ECHO_PIN);
  float obstacle_dist = (pulse_width < MAX_TIME) ? (pulse_width/PW_TO_CM) : (MAX_TIME/PW_TO_CM);

  Serial.print(sampling_time); Serial.print("\t"); Serial.println(obstacle_dist); 
  //Serial.print(" cm\n");

  #ifdef SERVO
  // Calculate and set wheel position based on obstacle distance
  servo_pos = DEFAULT_POS + (MAX_DIST - obstacle_dist)/CM_TO_DEG;
  wheel.write(servo_pos);
  #endif
  
  // Wait indicated sampling time before next measurement
  delay(SAMPLE_DELAY);
  count++;
  }
  reading = 0;
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
