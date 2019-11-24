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
int MOTOR_PORT_1 = 12;
int MOTOR_PORT_2 = 11;
int MOTOR_PORT_3 = 10;
int MOTOR_PORT_4 = 9;
int MOTOR_PORT_5 = 8;
int MOTOR_PORT_6 = 7;
int MOTOR_PORT_7 = 6;

int POTEN_LOW = -1;
int POTEN_HIGH = 1; 
int PULSE_WIDTH_LOW = 1100;
int PULSE_WIDTH_HIGH = 1900; 
int PULSE_SLOW_OFFSET = 100; // subtract the high by the offset to get a smaller high
int PULSE_OFF = PULSE_WIDTH_LOW + (PULSE_WIDTH_HIGH - PULSE_WIDTH_LOW)/2;
int BITS_PER_SEC = 57600;  
float MIN_STICK_THRESHOLD = 0.01;
float rub = 0;
float rbb = 0;
float rb = 0;

double pos = 0.0;  

bool x, y, yaw_on , vert , hori ;
double diag ;



// ==========================================================================
//    motor 1   motor 2   motor  3  motor 4   motor 7   motor 5    motor  6
Servo motor_fl, motor_fr, motor_rr, motor_rl, motor_ru, motor_flu, motor_fru;
float left_hori, left_vert, right_hori, right_vert , unmapped_x , unmapped_y ;
// ==========================================================================

std_msgs::Float32 pos_msg;
ros::Publisher pos_pub("/sensor/vehicle/steering/actuator_position", &pos_msg);

// ===================================================
// =============== ROS callback methods ==============
// ===================================================

float average_1_4(  ){  
    diag = mapf( (unmapped_x + unmapped_y )/2 , POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
    return diag ;
}

void left_hori_cb( const std_msgs::Float32& msg){
  left_hori = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  // move_x(left_hori);
  hori = true ;
  if (abs(msg.data) >= 0.01) {
    x = true;
  } else {
    x = false;
  }
/*
  if (x == true && y == false && yaw_on == false) {
    move_x(left_hori);
  }
  */
 
  unmapped_x = msg.data ;  
  if (hori && vert && yaw_on == false) {
    move_fl_rr( average_1_4() );
  }   
  hori = true ;
  if ( hori && vert ) {
    move_fr_rl( average_1_4( 1 ) );
    hori = false ;
    vert = false ;
  }
  
}

void left_vert_cb( const std_msgs::Float32& msg){
  left_vert = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
/*

  if (abs(msg.data) >= 0.01) {
    y = true;
  } else {
    y = false;
    
  }
  if (x == false && y == true && yaw_on == false) {
    move_y(left_vert);
  }
  */

  
  unmapped_y = msg.data ; 
  if (hori && vert && yaw_on == false) {
    move_fl_rr( average_1_4() );
  }    
  vert = true ;
  unmapped_y *= -1.0 ; 
  if ( hori && vert ) {
    move_fr_rl( average_1_4() );
    hori = false ;
    vert = false ;
  }
}

void right_hori_cb( const std_msgs::Float32& msg){
  right_hori = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  
  if (abs(msg.data) >= 0.01) {
    yaw_on = true;
  } else {
    yaw_on = false;
  }

  if (x == false && y == false && yaw_on == true) {
    yaw(right_hori);
  }

}

void right_vert_cb( const std_msgs::Float32& msg){
  right_vert = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  // pitch(right_vert);
}


void right_upper_bumper_cb(const std_msgs::Float32& msg) {
  rub = msg.data;
}

void right_bottom_bumper_cb(const std_msgs::Float32& msg) {
  rbb = msg.data;
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
  
  nh.advertise(pos_pub);

  
  // initialize serial communication
  motor_fr.attach(MOTOR_PORT_1);
  motor_fl.attach(MOTOR_PORT_2);
  motor_rr.attach(MOTOR_PORT_3);
  motor_rl.attach(MOTOR_PORT_4);
  motor_ru.attach(MOTOR_PORT_5);
  motor_flu.attach(MOTOR_PORT_6);
  motor_fru.attach(MOTOR_PORT_7);

  
  motor_fl.writeMicroseconds(PULSE_OFF);
  motor_fr.writeMicroseconds(PULSE_OFF);
  motor_rr.writeMicroseconds(PULSE_OFF);
  motor_rl.writeMicroseconds(PULSE_OFF);
  motor_ru.writeMicroseconds(PULSE_OFF);
  motor_flu.writeMicroseconds(PULSE_OFF);
  motor_fru.writeMicroseconds(PULSE_OFF);
  
  delay(2000);
}

// ===================================================
// ====================== loop =======================
// ===================================================
// the loop routine runs over and over again forever:
void loop() {

  // -----debugging only -----
  pos = left_vert;
  pos_msg.data = pos;
  pos_pub.publish(&pos_msg);
  // -----debugging only -----

  int delay_sec = 3;
  nh.spinOnce();

  if (rub == 1 && rbb == 0){
    move_z(1750);
  } else if (rub == 0 && rbb == 1){
    move_z(1250);
  } else {
    move_z(PULSE_OFF);
  }

  if (!x && !y && !yaw_on) {
    move_y(PULSE_OFF);
  }

  delay(delay_sec);        // delay in between reads for stability
}

// ===================================================
// ====================== moving =====================
// ===================================================
// int powervalue;
// controller by the verticle left joystick movement
void move_y(int left_vert) {
  // forward / backwards (positive)
  // they are all spinning in the same direction
  
  motor_fl.writeMicroseconds(left_vert);
  motor_fr.writeMicroseconds(left_vert);
  motor_rr.writeMicroseconds(left_vert);
  motor_rl.writeMicroseconds(left_vert);
}

// horizontal left
void move_x(float left_hori) {
  // motor 2 and 4 need to be reversed --> positive x
  
  motor_fl.writeMicroseconds(left_hori);
  motor_fr.writeMicroseconds(reverse_motor(left_hori));
  motor_rr.writeMicroseconds(left_hori);
  motor_rl.writeMicroseconds(reverse_motor(left_hori));
}
//////////// SPLIT /////////////////////////////////
//////////// SPLIT /////////////////////////////////

void move_fl_rr(int left_vert) {
  // forward / backwards (positive)
  // they are all spinning in the same direction
  
  motor_fl.writeMicroseconds(left_vert);
  motor_rr.writeMicroseconds(left_vert);
}

// horizontal left
void move_fr_rl(float left_hori) {
  // motor 2 and 4 need to be reversed --> positive x
  
  motor_fr.writeMicroseconds(left_hori);
  motor_rl.writeMicroseconds(left_hori);
    
  //motor_fr.writeMicroseconds(reverse_motor(left_hori));
  //motor_rl.writeMicroseconds(reverse_motor(left_hori));
}
//////////// SPLIT /////////////////////////////////
//////////// SPLIT /////////////////////////////////
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

float reverse_motor(float in){
  return 1500.0 - (in - 1500.0);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
