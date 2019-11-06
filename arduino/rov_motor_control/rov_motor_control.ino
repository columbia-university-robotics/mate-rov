/*
 * Columbia University Robotics Club. 
 * MATE-ROS Competition. 
 * 
 * Overview: 
 * The Arduino code handles the motor movement on the ROV. 
 * The Arduino Due receives user joystick data via serial from ROS. 
 * Then, it processes that data and computes the motor movements. 
 * 
 * For the definition of different axis, please checkout the readme. 
 * 
 * (c) Columbia University Robotics Club, 2019 - 2020.
 * All Rights Reserved. Development by the Software & Arduino Team. 
 * Contact: Neil, yn2376@columbia.edu
 *
https://github.com/ros-drivers/rosserial/tree/melodic-devel/rosserial_arduino


https://answers.ros.org/question/264764/rosserial-arduino-due-sync-issues/

http://docs.ros.org/jade/api/rosserial_arduino/html/ArduinoHardware_8h_source.html

 http://docs.ros.org/jade/api/rosserial_arduino/html/ArduinoHardware_8h.html
 */
//#define USE_USBCON


#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include "
ros::NodeHandle nh;

// define all constants
int MOTOR_PORT = 3;
int POTEN_LOW = -1;
int POTEN_HIGH = 1; 
int PULSE_WIDTH_LOW = 1100;
int PULSE_WIDTH_HIGH = 1900; 
int PULSE_OFF = PULSE_WIDTH_LOW + (PULSE_WIDTH_HIGH - PULSE_WIDTH_LOW)/2;
int BITS_PER_SEC = 57600;  
float MIN_STICK_THRESHOLD = 0.01;

Servo motor_fl, motor_fr, motor_rr, motor_rl, motor_fu, motor_rlu, motor_rru;
float left_hori, left_vert, right_hori, right_vert;

// ===================================================
// =============== ROS callback methods ==============
// ===================================================

void left_hori_cb( const std_msgs::Float32& msg){
  left_hori = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
}

void left_vert_cb( const std_msgs::Float32& msg){
  left_vert = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
}

void right_hori_cb( const std_msgs::Float32& msg){
  right_hori = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
}

void right_vert_cb( const std_msgs::Float32& msg){
  right_vert = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
}

// declare all ROS pub inst. vars. 
// ros::Subscriber<std_msgs::Float32> lh_sub("/controller/left_hori", &left_hori_cb );
ros::Subscriber<std_msgs::Float32> lv_sub("/controller/left_vert", left_vert_cb );
// ros::Subscriber<std_msgs::Float32> rh_sub("/controller/right_hori", &right_hori_cb );
// ros::Subscriber<std_msgs::Float32> rv_sub("/controller/right_vert", &right_vert_cb );

// ===================================================
// ====================== setup ======================
// ===================================================
// the setup routine runs once when you press reset:
void setup() {

  // init node handler
  nh.initNode();
  // nh.subscribe(lh_sub);
  nh.subscribe(lv_sub);
  // nh.subscribe(rh_sub);
  // nh.subscribe(rv_sub);
  
  // initialize serial communication 
  motor_fl.attach(MOTOR_PORT);
  motor_fl.writeMicroseconds(PULSE_OFF);
  delay(2000);
}

// ===================================================
// ====================== loop =======================
// ===================================================
// the loop routine runs over and over again forever:
void loop() {

  motor_fl.writeMicroseconds(left_vert);
  // ---------------

  int delay_sec = 3;
  nh.spinOnce();
  
  delay(delay_sec);        // delay in between reads for stability
}

// ===================================================
// ====================== moving =====================
// ===================================================
// methods for moving the robot
void move_rov(float left_hori, float left_vert, float right_hori, float right_vert){

  if (abs(right_vert) > MIN_STICK_THRESHOLD) {
    move_x(right_vert);
  } else if (abs(right_hori) > MIN_STICK_THRESHOLD) {
    move_y(right_hori);
  }
  
}

void move_x(float right_vert) {
  // TODO
}

void move_y(float right_hori) {
  // TODO
}

void move_z() {
  // TODO
}

void yaw() {
  
}

void pitch() {
  
}

void roll() {
  
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
