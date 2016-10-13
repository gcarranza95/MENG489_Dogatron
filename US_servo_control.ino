/*
 * Control a servo with an ultrasonic sensor: the closer an obstacle is to the sensor,
 * the greater the angle of the servo from 90 degrees. 
 * Based on HC-SR04 Demo: Demonstration of the HC-SR04 Ultrasonic Sensor (August 3, 2016; license: Public Domain)
 */

#include <Servo.h>

Servo wheel;
const int DEFAULT_POS = 90;
int servo_pos = DEFAULT_POS;

const int TRIG_PIN = 7;
const int ECHO_PIN = 8;
const int SERVO_PIN = 9;

const unsigned int MAX_TIME = 23200;  // Anything over 400 cm (10000 us pulse) is "out of range"
const float PW_TO_CM = 58.0;
const unsigned int MAX_DIST = MAX_TIME/PW_TO_CM; 
const float CM_TO_DEG = MAX_DIST/90;

void setup() {

  wheel.attach(SERVO_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(9600);
}

void loop() {

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
  Serial.print(obstacle_dist);
  Serial.print(" cm\n");
  
  // Calculate and set wheel position based on obstacle distance
  servo_pos = DEFAULT_POS + (MAX_DIST - obstacle_dist)/CM_TO_DEG;
  wheel.write(servo_pos);
  
  // Wait at least 60ms before next measurement
  delay(60);
}
