#include <Servo.h>

// Servo setup
Servo dogatron;
const int TARGET_POS = 90;
const int SERVO_PIN = 7;

int servo_pos = 90;

void setup() {
  // put your setup code here, to run once:
  dogatron.attach(SERVO_PIN);
  Serial.begin(9600);
}

void loop() {
  int i;
  // put your main code here, to run repeatedly:
  for (i=0; i<=180; i++){
  dogatron.write(i);
  delay(20);
  }
  for (i=180; i>=0; i--){
  dogatron.write(i);
  delay(20);
  }
}
