#include <ros.h>
#include "BTS7960.h"
#include <PID_v1.h>
#include <ArduinoHardware.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#define LOOPTIME 100
unsigned long int noCommLoopMax = 100000;          //number of main loops the robot will execute without communication before stopping
unsigned long int noCommLoops = 0;                 //main loop without communication counter

// PINOUT for LEFT Motor
// L_EN -> 7
// R_EN -> 8
// L_PWM -> 9
// R_PWM -> 10

const uint8_t R_EN_Left = 25;
const uint8_t L_EN_Left = 24;
const uint8_t L_PWM_Left = 6;
const uint8_t R_PWM_Left = 7;
BTS7960 motorController_Left(L_EN_Left,R_EN_Left, L_PWM_Left, R_PWM_Left);

// PINOUT for RIGHT Motor
// L_EN -> 3
// R_EN -> 4
// L_PWM -> 5
// R_PWM -> 6

const uint8_t R_EN_Right = 23;
const uint8_t L_EN_Right = 22;
const uint8_t L_PWM_Right = 8;
const uint8_t R_PWM_Right = 9;
BTS7960 motorController_Right(L_EN_Right,R_EN_Right, L_PWM_Right, R_PWM_Right);

//PINOUT for Left Motor Encoder

const uint8_t left_ena = 20;
const uint8_t left_enb = 21;

//PINOUT for Right Motor Encoder

const uint8_t right_ena = 18;
const uint8_t right_enb = 19;

volatile float pos_left = 0;    // encoder left
volatile float pos_right = 0;   // encoder right

unsigned long currentMillis;
unsigned long prevMillis;

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s

//PID
double left_kp = 0.4480 , left_ki = 0.5 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 0.4 , right_ki = 0.5 , right_kd = 0.0;

PID rightPID(&speed_act_right, &speed_cmd_right, &speed_req_right, right_kp, right_ki, right_kd, DIRECT);  
PID leftPID(&speed_act_left, &speed_cmd_left, &speed_req_left, left_kp, left_ki, left_kd, DIRECT);  

double demandx = 0, demandz = 0;

const double radius = 0.04;               //Wheel radius, in m
const double wheelbase = 0.380;            //Wheelbase, in m
const double encoder_cpr = 4480;           //Encoder ticks or counts per rotation
const double max_speed = 0.3;              //Max speed in m/s
const double speed_to_pwm_ratio = 0.00235; //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor

ros::NodeHandle nh;

void cmd_vel_cb(const geometry_msgs::Twist& velocity){
  noCommLoops = 0;  
  demandx = velocity.linear.x;
  demandz = velocity.angular.z;
  speed_req_left  = demandx - demandz * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = demandx + demandz * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds   
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&cmd_vel_cb);
geometry_msgs::Vector3Stamped speed_msg;                         
ros::Publisher speed_pub("speed", &speed_msg);  

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(sub);
  nh.advertise(speed_pub);

  //Setting PID parameters
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(95);
  rightPID.SetOutputLimits(-max_speed, max_speed);
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(95);
  leftPID.SetOutputLimits(-max_speed, max_speed);
  
  //Serial.begin(9600);
  //Serial.println("Basic Encoder Test:");
  //Setting Encoder left and right
  pinMode(left_ena, INPUT_PULLUP);
  pinMode(left_enb, INPUT_PULLUP);
  pinMode(right_ena, INPUT_PULLUP);
  pinMode(right_enb, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left_ena), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_enb), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_ena), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_enb), change_right_b, CHANGE);
  
  //Initializing Motors and Setting Speed to Zero
  motorController_Left.Enable();
  motorController_Right.Enable();
  motorController_Left.Stop();
  motorController_Right.Stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;
   
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    
    pos_left = 0;
    pos_right = 0;
    
    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    leftPID.Compute();  
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255);

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    rightPID.Compute();
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); 
    
    if(speed_req_left > 0.0){
      motorController_Left.GoStraight(PWM_leftMotor);
    }
    else if(speed_req_left < 0.0){
      motorController_Left.GoInverse(abs(PWM_leftMotor));
    }
    else{
      motorController_Left.Stop();
    }
    if(speed_req_right > 0.0){
      motorController_Right.GoInverse(abs(PWM_rightMotor));
    }
    else if(speed_req_right < 0.0){
      motorController_Right.GoStraight(abs(PWM_rightMotor));
    }
    else{
      motorController_Right.Stop();
    }
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      motorController_Left.Stop();
      motorController_Right.Stop();
      motorController_Left.Disable();
      motorController_Right.Disable();
      nh.loginfo("No Command for a long while! Shutting down the motors.");
    }
    
    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

    if((millis()-prevMillis) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      nh.loginfo(" TOO LONG ");
    }
    Serial.println(noCommLoops);
    Serial.println(noCommLoopMax);
  }
  publishSpeed(LOOPTIME);
  nh.spinOnce();  
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
}

void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(left_ena) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(left_enb) == LOW) {  
      pos_left = pos_left + 1;         // CW
    } 
    else {
      pos_left = pos_left - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left_enb) == HIGH) {   
      pos_left = pos_left + 1;          // CW
    } 
    else {
      pos_left = pos_left - 1;          // CCW
    }
  }
 
}

void change_left_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(left_enb) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(left_ena) == HIGH) {  
      pos_left = pos_left + 1;         // CW
    } 
    else {
      pos_left = pos_left - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left_ena) == LOW) {   
      pos_left = pos_left + 1;          // CW
    } 
    else {
      pos_left = pos_left - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void change_right_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(right_ena) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(right_enb) == LOW) {  
      pos_right = pos_right - 1;         // CW
    } 
    else {
      pos_right = pos_right + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right_enb) == HIGH) {   
      pos_right = pos_right - 1;          // CW
    } 
    else {
      pos_right = pos_right + 1;          // CCW
    }
  }
 
}

void change_right_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(right_enb) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(right_ena) == HIGH) {  
      pos_right = pos_right - 1;         // CW
    } 
    else {
      pos_right = pos_right + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right_ena) == LOW) {   
      pos_right = pos_right - 1;          // CW
    } 
    else {
      pos_right = pos_right + 1;          // CCW
    }
  }
}
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
    }
