#include <Average.h>

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
 #define DEBUG_PRINT(x)  Serial.print(x)
#else
 #define DEBUG_PRINT(x)
#endif

#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
 #define DEBUG_PRINTLN(x)
#endif


#ifdef servo_in_use 
#include <Servo.h>
Servo wheel;
const int SERVO_PIN = 7;
const int DEFAULT_POS = 90;
int SERVO_LEFT_LIMIT = 60; 
int SERVO_RIGHT_LIMIT = 120;
#endif

#ifdef ultrasonic_in_use
// US sensor setup: 0=left, 1=center, 2=right
  const int NUM_SENSORS = 3;
  const int TRIG_PINS[NUM_SENSORS] = {8,10,12};
  const int ECHO_PINS[NUM_SENSORS] = {9,11,13};
  const int LFT = 0;
  const int CTR = 1;
  const int RGT = 2; 
  int obst_dist[3]; // Obstacle distance from given sensor
  int obst_bool[3]; // Is obstacle present at given sensor?
  int turn_dir = 0;
  //const int MAX_TIME = 23200;  // Anything over 400 cm (10000 us pulse) is "out of range"
//  const int MAX_TIME = 5800;
//  const float PW_TO_CM = 58.0;
//  const float MAX_DIST = MAX_TIME/PW_TO_CM; 
//  const float CM_TO_DEG = MAX_DIST/90;
const unsigned int MAX_DIST = 100;  // (cm); anything over this value is "out of range"
const float PW_TO_CM = 58.0;        // (us/cm)
const unsigned int MAX_TIME = MAX_DIST*PW_TO_CM; // (us)
const float SCALAR = 3;
const float CM_TO_DEG = SCALAR*MAX_DIST/90;  
  //for filtering 
  const int HIST = 3;
  int hist_pos=0;
//  struct hist(hist_size){
//    int pos = 0;
//    int count;
//    int hist_array[hist_size];
//
//    int get_count(){
//      return count;
//    }
//
//    void pw_push(int pw_value){
//    hist_array(pos) = pw_value;
//
//    if (pos>= hist_sze)
//    } 
//    
//    }
//  int pw_hist_LFT[HIST];
//  int pw_hist_CTR[HIST];  
//  int pw_hist_RGT[HIST];
  Average<int> pw_hist_LFT(HIST);
  Average<int> pw_hist_CTR(HIST);
  Average<int> pw_hist_RGT(HIST);
#endif

#ifdef compass_in_use
#include <Wire.h> //I2C Arduino Library for HMC5883 COMPASS
#endif

#ifdef speaker_in_use //
const int SPEAKER_PIN = 6; //SPECIFIC PIN HAS NOT BEEN SET YET
#endif

#ifdef hap1_in_use
const int HAP1_PIN = 4;
#endif

#ifdef hap2_in_use
const int HAP2_PIN = 5;
#endif

//variable to alternate between different implemented steering methods
                                      int steering_method = 1;
                                      int test_type = 0;  

void setup()
{
Serial.begin(9600);


#ifdef speaker_in_use
pinMode(SPEAKER_PIN, OUTPUT);
#endif

#ifdef servo_in_use
wheel.attach(SERVO_PIN);
#endif

#ifdef compass_in_use
Wire.begin();
Wire.beginTransmission(addr); //start talking
Wire.write(0x02); // Set the Register
Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
Wire.endTransmission();  
#endif

#ifdef ultrasonic_in_use
 for(int i = 0; i < NUM_SENSORS; i++) {
    int pin = TRIG_PINS[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
#endif

#ifdef hap1_in_use
pinMode(HAP1_PIN,OUTPUT);
#endif

#ifdef hap2_in_use
pinMode(HAP2_PIN,OUTPUT);
#endif
}

void loop(){

  //first possible steering method
  #ifdef all_in_use
  
  if(steering_method == 1){ //ultrasound sensors only
    int intended_wheel_angle;
    int current_wheel_angle = wheel.read(); // reads servo's current angle    
    int wheel_response = collisionResponse();
    if(wheel_response> 95 || wheel_response < 85)
    {
     vibrator(1,HAP1_PIN,0);
     vibrator(1,HAP2_PIN,0);
    }
    
     
   //collisionResponse's output would determine the wheel.write value
    intended_wheel_angle = constrain(wheel_response,SERVO_LEFT_LIMIT,SERVO_RIGHT_LIMIT);
    DEBUG_PRINT(wheel_response); DEBUG_PRINT(" "); DEBUG_PRINTLN(intended_wheel_angle);
    
    wheel.write(intended_wheel_angle);
    //wheelTurn(current_wheel_angle,intended_wheel_angle,50);
    }
    
  if(steering_method == 2){ //compass only
    int intended_wheel_angle;
    int current_wheel_angle = wheel.read(); // reads servo's current angle    
    float current_heading = getHeading(); 
    DEBUG_PRINTLN(current_heading);
    intended_wheel_angle = 180- constrain(DEFAULT_POS-current_heading,SERVO_LEFT_LIMIT,SERVO_RIGHT_LIMIT);
    //wheelTurn(current_wheel_angle,intended_wheel_angle,50);
    wheel.write(intended_wheel_angle);
    }

    if(steering_method == 3){ //both
    
    int intended_wheel_angle;
    int current_wheel_angle = wheel.read(); // reads servo's current angle    
    int wheel_response = collisionResponse();
      float current_heading = getHeading(); 
      DEBUG_PRINT("Heading: "); DEBUG_PRINTLN(current_heading);
      if(wheel_response == 90)// if no impending collision, then try to follow north
      {
       
       
       intended_wheel_angle = 180- constrain(DEFAULT_POS-current_heading,SERVO_LEFT_LIMIT,SERVO_RIGHT_LIMIT);
       wheelTurn(current_wheel_angle,intended_wheel_angle,50);
      }
      else { //otherwise, collisionResponse's output would determine the wheel.write value
        intended_wheel_angle = constrain(wheel_response,SERVO_LEFT_LIMIT,SERVO_RIGHT_LIMIT);
        //wheelTurn(current_wheel_angle,intended_wheel_angle,50);
        wheel.write(intended_wheel_angle);
      }
//      if (wheel_response == 0){
//      DEBUG_PRINT("Heading north: ");
//      }
       // DEBUG_PRINT("Wheel Response: "); DEBUG_PRINT(wheel_response); DEBUG_PRINT(" ");
      // DEBUG_PRINT("Intended Wheel Angle: "); DEBUG_PRINTLN(intended_wheel_angle);
      
    }
    
    if(test_type == 1){
    for(int i = 0; i < NUM_SENSORS; i++) {
    int pw_val = ping(TRIG_PINS[i], ECHO_PINS[i]);
    Serial.print(pw_val); Serial.print("\t");
    }
    Serial.println();
    }
  #endif

}

#ifdef ultrasonic_in_use
float convertPulseWidthToDistance(int trig, int echo){
  int pulse_width = ping(trig, echo);
  float obstacle_dist = (pulse_width < MAX_TIME) ? (float) (pulse_width/PW_TO_CM) : (float) (MAX_TIME/PW_TO_CM); 
}
#endif

int ping(int trig, int echo) {
  unsigned long t1;
  unsigned long t2;
  unsigned long t0;
  unsigned long wait_time;
  int pulse_width;
  // Hold the trigger pin high for 10 us
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Wait for pulse on echo pin, then measure how long the echo pin was held high (pulse width)
  t0 = micros();
  while (digitalRead(echo) == 0 && (wait_time = micros()-t0) < MAX_TIME);
  
  if(wait_time < MAX_TIME){ 
  t1 = micros();
  while (digitalRead(echo) == 1);
  t2 = micros();
  pulse_width = (int) (t2 - t1);
  }
  else
  { //max time exceeded
    pulse_width = MAX_TIME;
  }
  return pulse_width;
}

#ifdef compass_in_use
//output format: 0 represents north 
float getHeading() {    // GETTING HMC5883 X,Y AND Z VALUES
  unsigned int x,y,z; //triple axis data

  //Tell the HMC what regist to begin writing data unsigned into
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

void buzz(unsigned int pin, long frequency, long len) {
  long delayVal = 1000000/frequency/2; // Delay value between transitions; unitless (1 second in us)/(1/s)
  long numCycles = frequency*len/1000; // Number of cycles; unitless (1/s)/(s)
  for (long i=0; i < numCycles; i++) {
    digitalWrite(pin, HIGH);      // high (push out the diaphram)
    delayMicroseconds(delayVal);  // wait  
    digitalWrite(pin, LOW);       // low (pull back the diaphram)
    delayMicroseconds(delayVal);  // wait  
  }
}

void vibrator(unsigned int switcher, unsigned int pin, unsigned int type) {
  if (switcher == 1)    //function is on if value "1" is passed to it
  {
   if (type == 1) { 
    delay(1000);          //set pin to unsigned intermittent buzz 
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
int filteredResponse(){
  int pw[NUM_SENSORS];
  int pw_filt[NUM_SENSORS];
  float obst_dist[NUM_SENSORS];
  bool obst_bool[NUM_SENSORS];
  float servo_pos;
  //get raw pulse width values based on history, then update history w/ raw value
  
  for(int i = 0; i < NUM_SENSORS; i++) {
    int pw_val = ping(TRIG_PINS[i], ECHO_PINS[i]);
    pw[i] = (pw_val < MAX_TIME) ? pw_val : MAX_TIME;
  }
  //DEBUG_PRINT(pw[LFT]); DEBUG_PRINT(" "); DEBUG_PRINT(pw[CTR]); DEBUG_PRINT(" "); DEBUG_PRINTLN(pw[RGT]); 
  //DEBUG_PRINT(pw_hist_LFT.getCount()); DEBUG_PRINT(" "); DEBUG_PRINT(pw_hist_CTR.getCount()); DEBUG_PRINT(" "); DEBUG_PRINTLN(pw_hist_RGT.getCount()); 
  //DEBUG_PRINTLN(pw_hist_CTR.minimum());
  //DEBUG_PRINT("pw[CTR] = "); DEBUG_PRINTLN(pw[CTR]);
  // Adjust (filter) current values based on history, then update history w/raw value
  pw_filt[LFT] = (pw_hist_LFT.getCount() == HIST) ? filter(pw[LFT], pw_hist_LFT) : pw[LFT];
  pw_filt[CTR] = (pw_hist_CTR.getCount() == HIST) ? filter(pw[CTR], pw_hist_CTR) : pw[CTR]; //line of bug
  pw_filt[RGT] = (pw_hist_RGT.getCount() == HIST) ? filter(pw[RGT], pw_hist_RGT) : pw[RGT];
  pw_hist_LFT.push(pw[LFT]);
  pw_hist_CTR.push(pw[CTR]);
  pw_hist_RGT.push(pw[RGT]);
  // Get obstacle distance values from all three sensors    
  for(unsigned int i=0; i<NUM_SENSORS; i++) {
    obst_dist[i] = pw_filt[i] / PW_TO_CM;
    obst_bool[i] = (pw_filt[i] < MAX_TIME) ? true : false;
  }

  //Choose turning direction based on proximity of obstacles: 0 = forward, 1 = right, -1 = left
  turn_dir = obst_bool[CTR] ? ((obst_dist[LFT] < obst_dist[RGT]) ? 1 : -1) : 0;
  

  // Calculate and set dogatron position based on obstacle distance
  servo_pos = (DEFAULT_POS + turn_dir*(int)((MAX_DIST - obst_dist[CTR])/CM_TO_DEG));

  return servo_pos;

}
#endif

#ifdef ultrasonic_in_use
int collisionResponse(){
  int pw;
  float obst_dist[NUM_SENSORS];
  bool obst_bool[NUM_SENSORS];
  int servo_pos;
  for(int i = 0; i < NUM_SENSORS; i++) {
    pw = ping(TRIG_PINS[i], ECHO_PINS[i]);
    obst_dist[i] = (pw < MAX_TIME) ? (pw/PW_TO_CM) : (MAX_TIME/PW_TO_CM);
    obst_bool[i] = (pw < MAX_TIME) ? true : false;
  }

  //Choose turning direction based on proximity of obstacles: 0 = forward, 1 = right, -1 = left
  turn_dir = obst_bool[CTR] ? ((obst_dist[LFT] < obst_dist[RGT]) ? -1 : 1) : 0;
  

  // Calculate and set dogatron position based on obstacle distance
  servo_pos = (DEFAULT_POS + turn_dir*(int)((MAX_DIST - obst_dist[CTR])/CM_TO_DEG));
  return servo_pos;
}
#endif



#ifdef ultrasonic_in_use
int filter(int current_pw, Average<int> pw_hist) {
  if (pw_hist.mode() == MAX_TIME) {
    current_pw = MAX_TIME;
  }
  else if (current_pw == MAX_TIME) {
    current_pw = pw_hist.minimum();
  }
  //DEBUG_PRINTLN(current_pw);
  return current_pw; 
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




