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

// need to implement a warning LED. 

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
#include <Wire.h>

// ================= MPU Vars =================
float angle_pitch, angle_roll;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;
// ============================================

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

bool x, y, yaw_on;

// ==========================================================================
//    motor 1   motor 2   motor  3  motor 4   motor 7   motor 5    motor  6
Servo motor_fl, motor_fr, motor_rr, motor_rl, motor_ru, motor_flu, motor_fru;
float left_hori, left_vert, right_hori, right_vert;
// ==========================================================================

std_msgs::Float32 pos_msg;
ros::Publisher pos_pub("/sensor/vehicle/steering/actuator_position", &pos_msg);

// ===================================================
// =============== ROS callback methods ==============
// ===================================================

void left_hori_cb( const std_msgs::Float32& msg){

  left_hori = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);

  if (abs(msg.data) >= 0.01)
    x = true;
  else
    x = false;

  if (x == true && y == false && yaw_on == false)
    move_x(left_hori);
}

void left_vert_cb( const std_msgs::Float32& msg){

  // flipped the sign during pool testing
  left_vert = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);

  if (abs(msg.data) >= 0.01)
    y = true;
  else 
    y = false;
  
  if (x == false && y == true && yaw_on == false)
    move_y(left_vert);
}

void right_hori_cb( const std_msgs::Float32& msg){
  right_hori = mapf(-1.0*msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  
  if (abs(msg.data) >= 0.01) {
    yaw_on = true;
  } else {
    yaw_on = false;
  }

  if (x == false && y == false && yaw_on == true)
    yaw(right_hori);
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
ros::Subscriber<std_msgs::Float32> up_button_sub("/controller/right_topbumper", &right_bottom_bumper_cb);
ros::Subscriber<std_msgs::Float32> down_button_sub("/controller/right_bottombumper", &right_upper_bumper_cb);

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

// ===========================================================
// ========================== loop ===========================
// ===========================================================
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
    move_z(1800);
  } else if (rub == 0 && rbb == 1){
    move_z(1200);
  } else {
    move_z(PULSE_OFF);
  }

  if (!x && !y && !yaw_on) {
    move_y(PULSE_OFF);
  }

  delay(delay_sec);        // delay in between reads for stability
}

// ===========================================================
// ========================== moving =========================
// ===========================================================

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

void move_z(float v) {
  // turn 5, 6, 7 in the same direction
  motor_ru.writeMicroseconds(v);
  motor_fru.writeMicroseconds(v);
  motor_flu.writeMicroseconds(v);
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

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ===========================================================
// =========================== MPU ===========================
// ===========================================================

void calculate_angle() {

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.00095419847;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.00095419847;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.00000719542);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.00000719542);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

}

void print_mpu_data(){

//  Serial.print("Pitch: ");
//  Serial.println(angle_pitch_output);
  Serial.print("Roll: ");
  Serial.println(angle_roll_output);
}

void read_mpu_data() {

    //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  gyro_y = gyro_y * -1;
  gyro_z = gyro_z * -1;
}

void setup_mpu_6050() {

  // We want to write to the PWR_MGMT_1 register (6B hex)
  // Set the register bits as 00000000 to activate the gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);                                                          
  Wire.write(0x00);                                                          
  Wire.endTransmission();

  // We want to write to the GYRO_CONFIG register (1B hex)
  // Set the register bits as 00001000 (500dps full scale)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          
  Wire.write(0x08);                                                          
  Wire.endTransmission();                                                   

  // We want to write to the ACCEL_CONFIG register (1A hex)
  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                                                          
  Wire.write(0x10);                                                          
  Wire.endTransmission();

  // Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(0x68, 1);                                                 //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(12, HIGH);                                                  //Turn on the warning led
    while (1) delay(10);                                                     //Stay in this loop for ever
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();
}
