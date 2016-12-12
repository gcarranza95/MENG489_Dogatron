
#define addr 0x1E //I2C Address for The HMC5883
#define all_in_use
#ifdef all_in_use
#define ultrasonic_in_use
#define servo_in_use
#define compass_in_use
#define speaker_in_use
#define compass_in_use
#define hap1_in_use
#define hap2_in_use
#endif

//#define ultrasonic_in_use
//#define servo_in_use
//#define compass_in_use
//#define speaker_in_use
//#define compass_in_use
//#define hap1_in_use
//#define hap2_in_use


#define DEBUG
#ifdef DEBUG
 #define DEBUG_PRINT(fx)  Serial.print(x)
#else
 #define DEBUG_PRINT(x)
#endif
//#define FOLLOW //Compass follower, vs. heading compensator


#ifdef servo_in_use 
#include <Servo.h>
Servo wheel;
const int SERVO_PIN = 7;
const int DEFAULT_POS = 90;
int servo_pos = DEFAULT_POS;
int SERVO_LEFT_LIMIT = 60;
int SERVO_RIGHT_LIMIT = 120;
#endif

#ifdef ultrasonic_in_use
// US sensor setup: 0=left, 1=center, 2=right
  const int TRIG_PIN;
  const unsigned int NUM_SENSORS = 3;
  const int TRIG_PINS[NUM_SENSORS] = {8,10,12};
  const int ECHO_PINS[NUM_SENSORS] = {9,11,13};
  const int LFT = 0;
  const int CTR = 1;
  const int RGT = 2; 
  int obst_dist[3]; // Obstacle distance from given sensor
  int obst_bool[3]; // Is obstacle present at given sensor?
  int turn_dir = 0;
  const unsigned int MAX_TIME = 23200;  // Anything over 400 cm (10000 us pulse) is "out of range"
  const float PW_TO_CM = 58.0;
  const unsigned int MAX_DIST = MAX_TIME/PW_TO_CM; 
  const float CM_TO_DEG = MAX_DIST/90;
  const int DOGATRON_RADIUS = 5; // cm
#endif

#ifdef compass_in_use
#include <Wire.h> //I2C Arduino Library for HMC5883 COMPASS
#endif

#ifdef speaker_in_use //
const int SPEAKER_PIN = 6;
#endif

#ifdef hap1_in_use
const int HAP1_PIN = 4;
#endif

#ifdef hap2_in_use
const int HAP2_PIN = 5;
#endif

//used for testing of different sensors. If test_type == 0, then no tests are being conducted
int test_type = 0;

//used for digital compass testing
int count = 0; 
unsigned long time;
float angle;


//variable to alternate between different implemented steering methods
int steering_method = 1; 

void setup(){

Serial.begin(9600);


#ifdef speaker_in_use
pinMode(SPEAKER_PIN, OUTPUT);
#endif

  if (test_type == 4)
  {
    tone(SPEAKER_PIN,262,250);
    noTone(SPEAKER_PIN);
    test_type = 0;
  }


#ifdef servo_in_use
wheel.attach(SERVO_PIN);
//wheel.write(DEFAULT_POS); //is this necessary?
#endif


#ifdef compass_in_use
Wire.begin();
Wire.beginTransmission(addr); //start talking
Wire.write(0x02); // Set the Register
Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
Wire.endTransmission();  
#endif

#ifdef ultrasonic_in_use
pinMode(TRIG_PIN, OUTPUT);
digitalWrite(TRIG_PIN, LOW);
#endif

#ifdef hap1_in_use
pinMode(HAP1_PIN,OUTPUT);
#endif

#ifdef hap2_in_use
pinMode(HAP2_PIN,OUTPUT);
#endif
}

void loop() { 
  //LIST OF FUNCTIONS
  //void buzz(int pin, long frequency, long len)
  //int ping(int trig, int echo)
  //flo at convertPulseWidthToDistance(int trig, int echo)
  //float getHeading()
  
  //first possible steering method
  #ifdef all_in_use
  
  if(steering_method == 1){
    int intended_wheel_angle;
    int current_wheel_angle = wheel.read(); // reads servo's current angle
    int wheel_response = collisionResponse();
    if(wheel_response == 0)// if no impending collision, then try to follow north
    {
     float current_heading = getHeading(); 
     //Serial.print("Heading "); Serial.print(current_heading);
     intended_wheel_angle = 180- constrain(DEFAULT_POS-current_heading,SERVO_LEFT_LIMIT,SERVO_RIGHT_LIMIT);
     wheelTurn(current_wheel_angle,intended_wheel_angle,50);
    }
    else { //otherwise, collisionResponse's output would determine the wheel.write value
      intended_wheel_angle = wheel_response;
      wheelTurn(current_wheel_angle,intended_wheel_angle,50);
    }
    
    //Serial.print("Intended Wheel Angle: "); Serial.print(intended_wheel_angle);
    //Serial.print(" Servo reading: "); Serial.println(wheel.read()); 
  }
  
  #endif


  //TESTS SIMPLE WHEEL TURNING BACK AND FORTH
  if (test_type == 6){
  wheel.write(120);
  for (int i = 1; i<60; i+=3){
  wheel.write(120-i);
  delay(50);
  }
  for (int i = 1; i<60; i+=3){
  wheel.write(60+i);
  delay(50);
  }
  }
  //tests speaker   
  if(test_type == 5)
  {
    Serial.println("On");
    tone(SPEAKER_PIN,600);
    delay(2000);
    noTone(SPEAKER_PIN);
    delay(2000);
    
  }  

  #ifdef servo_in_use
  #ifdef compass_in_use
  if (test_type == 3) //testing micro-servo with compass, micro-servo points "north"
  {
  angle = getHeading();
  servo_pos = DEFAULT_POS - angle ;
  Serial.print(angle); Serial.print(" "); Serial.println(servo_pos);
  wheel.write(servo_pos);
  delay(100); 
  }
  #endif
  #endif
  
#ifdef compass_in_use  
  if (test_type == 2) //testing compass, does not stop reading values
  {
    time = millis();
    angle = getHeading();
    Serial.println(angle);
    delay(100); 
    count ++;
    }
#endif

#ifdef compass_in_use  
  if (test_type == 1) //testing compass, stops reading after "count" values 
  { 
    count = 0; 
    while (count < 100) {
    angle = getHeading();
    Serial.println(angle);
    delay(100); 
    count ++;
    }
  test_type = 0;
  }
#endif
}

#ifdef ultrasonic_in_use
float convertPulseWidthToDistance(int trig, int echo){
  int pulse_width = ping(trig, echo);
  float obstacle_dist = (pulse_width < MAX_TIME) ? (pulse_width/PW_TO_CM) : (MAX_TIME/PW_TO_CM);
}
#endif
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

#ifdef compass_in_use
//output format: 0 represents north 
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

  //Calculate heading, with range -180,180 in radians
  float heading = atan2(y,x);
  //heading = (heading<0) ? heading + 2*M_PI : heading;
  heading = heading * 180/M_PI;

  return heading;
}
#endif

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

void vibrator(int switcher, int pin, int type) {
  if (switcher == 1)    //function is on if value "1" is passed to it
  {
   if (type == 1) { 
    delay(1000);          //set pin to intermittent buzz 
    digitalWrite(pin,HIGH);   
    delay(1000);           
    digitalWrite(pin,LOW);   
  }
  else{
   digitalWrite(pin,HIGH);   //set pin to continuous buzz 
  }
 } 
 else{
  digitalWrite(pin,LOW); //set pin to off
 }
}
#ifdef ultrasonic_in_use
float collisionResponse(){
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
  servo_pos = DEFAULT_POS + turn_dir*(MAX_DIST - obst_dist[CTR])/CM_TO_DEG;

  return servo_pos;

}
float collisionResponse(){
  return 0;
  }
#endif
#ifdef servo_in_use
void wheelTurn(int start_angle,int end_angle,int ms_speed)
{
   int direction = (end_angle - start_angle < 0) ? -1 : 1; // -1 is left , 1 is right 
     for (int i = 1; i<=abs(end_angle-start_angle); i++)
     {
      wheel.write(start_angle+i*direction);
      delay(ms_speed); // this would determine how fast the servo would go
     }
}
#endif
