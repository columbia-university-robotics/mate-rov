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


#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;

// define all constants
int MOTOR_PORT_1 = 2;
int MOTOR_PORT_2 = 3;
int POTEN_LOW = -1;
int POTEN_HIGH = 1; 
int PULSE_WIDTH_LOW = 1100;
int PULSE_WIDTH_HIGH = 1900; 
int PULSE_SLOW_OFFSET = 200; // subtract the high by the offset to get a smaller high
int PULSE_OFF = PULSE_WIDTH_LOW + (PULSE_WIDTH_HIGH - PULSE_WIDTH_LOW)/2;
int BITS_PER_SEC = 57600;  
float MIN_STICK_THRESHOLD = 0.01;
float rub = 0;
float rbb = 0;
float rb = 0;

// ==========================================================================
//    motor 1   motor 2   motor  3  motor 4   motor 7   motor 5    motor  6
Servo motor_fl, motor_fr, motor_rr, motor_rl, motor_ru, motor_flu, motor_fru;
float left_hori, left_vert, right_hori, right_vert;
// ==========================================================================

// ===================================================
// =============== ROS callback methods ==============
// ===================================================

void left_hori_cb( const std_msgs::Float32& msg){
  left_hori = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  move_x(left_hori);
}

void left_vert_cb( const std_msgs::Float32& msg){
  left_vert = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  move_y(left_vert); // forward backward
}

void right_hori_cb( const std_msgs::Float32& msg){
  right_hori = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  yaw(right_hori);
}

void right_vert_cb( const std_msgs::Float32& msg){
  right_vert = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  pitch(right_vert);
}

void right_upper_bumper_cb(const std_msgs::Float32& msg) {
  rub = mapf(msg.data, 0, 1, PULSE_OFF, PULSE_WIDTH_HIGH - PULSE_SLOW_OFFSET);
  if (rbb == 0)
    move_z(rub);
}

void right_bottom_bumper_cb(const std_msgs::Float32& msg) {
  rub = mapf(-1.0*msg.data, -1, 0, PULSE_WIDTH_LOW + PULSE_SLOW_OFFSET, PULSE_OFF);
  if (rub == 0)
    move_z(rub);
}


// declare all ROS pub inst. vars. 
ros::Subscriber<std_msgs::Float32> lh_sub("/controller/left_hori", &left_hori_cb );
ros::Subscriber<std_msgs::Float32> lv_sub("/controller/left_vert", left_vert_cb );
ros::Subscriber<std_msgs::Float32> rh_sub("/controller/right_hori", &right_hori_cb );
ros::Subscriber<std_msgs::Float32> rv_sub("/controller/right_vert", &right_vert_cb );
ros::Subscriber<std_msgs::Float32> up_button_sub("/controller/right_topbumper", &right_upper_bumper_cb);
ros::Subscriber<std_msgs::Float32> down_button_sub("/controller/right_bottombumper", &right_bottom_bumper_cb);

// ===================================================
// ====================== setup ======================
// ===================================================
// the setup routine runs once when you press reset:
void setup() {

  // init node handler
  nh.initNode();
  nh.subscribe(lh_sub);
  nh.subscribe(lv_sub);
  nh.subscribe(rh_sub);
  nh.subscribe(rv_sub);
  nh.subscribe(up_button_sub);
  nh.subscribe(down_button_sub);
  
  // initialize serial communication
  motor_fr.attach(MOTOR_PORT_1);
  motor_fl.attach(MOTOR_PORT_2);
  motor_fl.writeMicroseconds(PULSE_OFF);
  motor_fr.writeMicroseconds(PULSE_OFF);
  delay(2000);
}

// ===================================================
// ====================== loop =======================
// ===================================================
// the loop routine runs over and over again forever:
void loop() {

  // ---------------

  int delay_sec = 3;
  nh.spinOnce();
  
  delay(delay_sec);        // delay in between reads for stability
}

// ===================================================
// ====================== moving =====================
// ===================================================

// int powervalue;
// controller by the verticle left joystick movement
void move_y(float left_vert) {
  // forward / backwards (positive)
  // they are all spinning in the same direction
  motor_fl.writeMicroseconds(left_vert);
  motor_fr.writeMicroseconds(left_vert);
  motor_rr.writeMicroseconds(left_vert);
  motor_rl.writeMicroseconds(left_vert);
}

// horizontal left
void move_x(float right_hori) {
  // motor 2 and 4 need to be reversed --> positive x
  motor_fl.writeMicroseconds(right_hori);
  motor_fr.writeMicroseconds(reverse_motor(right_hori));
  motor_rr.writeMicroseconds(right_hori);
  motor_rl.writeMicroseconds(reverse_motor(right_hori));
}

void move_z(float value) {

  // turn 5, 6, 7 in the same direction
  motor_ru.writeMicroseconds(value);
  motor_fru.writeMicroseconds(value);
  motor_flu.writeMicroseconds(value);
}

void yaw(float yaw) {
  // for counterclockwise viewed from positive z
  // same thing as forward
  // except 1 and 4 are reversed
  motor_fl.writeMicroseconds(reverse_motor(left_vert));
  motor_fr.writeMicroseconds(left_vert);
  motor_rr.writeMicroseconds(left_vert);
  motor_rl.writeMicroseconds(reverse_motor(left_vert));
  
}

void pitch(float pitch) {
  // 7 is the opposite of 5, 6. 
  // we define positive pitch as raising the head of the robot
  motor_ru.writeMicroseconds(reverse_motor(pitch));
  motor_fru.writeMicroseconds(pitch);
  motor_flu.writeMicroseconds(pitch);
  
}

void roll(float roll) {
  // positive roll is counterclockwise viewed from -y. 
  // opposite 5 and 6. 
  motor_fru.writeMicroseconds(roll);
  motor_flu.writeMicroseconds(reverse_motor(roll));
}

int reverse_motor(int in){
  return 1500 - (in - 1500);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
