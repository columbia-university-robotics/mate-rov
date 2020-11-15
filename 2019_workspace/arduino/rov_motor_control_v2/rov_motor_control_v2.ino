/*
   Columbia University Robotics Club.
   MATE-ROS Competition.

   Overview:
   The Arduino code handles the motor movement on the ROV.
   The Arduino Due receives user joystick data via serial from ROS.
   Then, it processes that data and computes the motor movements.

   For the definition of different axis, please checkout the readme.

   (c) Columbia University Robotics Club, 2019 - 2020.
   All Rights Reserved. Development by the Software & Arduino Team.
   Contact: Neil, yn2376@columbia.edu

  https://github.com/ros-drivers/rosserial/tree/melodic-devel/rosserial_arduino

  https://answers.ros.org/question/264764/rosserial-arduino-due-sync-issues/

  http://docs.ros.org/jade/api/rosserial_arduino/html/ArduinoHardware_8h_source.html

  http://docs.ros.org/jade/api/rosserial_arduino/html/ArduinoHardware_8h.html
*/

// need to implement a warning LED.
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include "pid_vars.h"
#include "motor_def.h"
#include "helper_methods.h"

ros::NodeHandle nh;
extern TwoWire Wire1; // use SCL1 & SDA1
LiquidCrystal_I2C lcd(0x27, 16, 2);

// time related vars and constants
int time_diff;
float freq;
long loop_timer;
int delay_sec = 1;
int lcd_loop_counter;

// ================= MPU Vars =================
// class default I2C address is 0x68

int angle_pitch_buffer, angle_roll_buffer;

float angle_pitch, angle_roll;
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;
int temperature;
float pid_error_temp;

const float angle_pitch_accel_cal = 0.0; // 0.628178439;
const float angle_roll_accel_cal = 0.0; // -1.425186567;
// ============================================

float rub = 0;
float rbb = 0;
float rb = 0;
int throttle = 1700;
int n_throttle = 1300 ;

double pos = 0.0;
bool x, y, yaw_on;

// ==========================================================================
//    motor 1   motor 2   motor  3  motor 4   motor 5    motor  6  motor 7   motor  8
Servo motor_fl, motor_fr, motor_rr, motor_rl, motor_lu, motor_fu , motor_ru, motor_bu;
float left_hori, left_vert, right_hori, right_vert;
// ==========================================================================

std_msgs::Float64 debug_msg_1;
std_msgs::Float64 yaw_msg;
std_msgs::Float64 pitch_msg;
std_msgs::Float64 roll_msg;
ros::Publisher debug_pub_1("/rov/debugging/stream_1", &debug_msg_1);
ros::Publisher yaw_pub("/rov/sensor/yaw", &yaw_msg);
ros::Publisher pitch_pub("/rov/sensor/pitch", &pitch_msg);
ros::Publisher roll_pub("/rov/sensor/roll", &roll_msg);

// ===================================================
// =============== ROS callback methods ==============
// ===================================================

void left_hori_cb(const std_msgs::Float32& msg) {

  left_hori = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);

  if (abs(msg.data) >= 0.01)
    x = true;
  else
    x = false;

  if (x == true && y == false && yaw_on == false)
    move_x(left_hori);
}

void left_vert_cb( const std_msgs::Float32& msg) {

  // flipped the sign during pool testing
  left_vert = mapf(msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);

  if (abs(msg.data) >= 0.01)
    y = true;
  else
    y = false;

  if (x == false && y == true && yaw_on == false)
    move_y(left_vert);
}

void right_hori_cb( const std_msgs::Float32& msg) {
  right_hori = mapf(-1.0 * msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);

  if (abs(msg.data) >= 0.01) {
    yaw_on = true;
  } else {
    yaw_on = false;
  }

  if (x == false && y == false && yaw_on == true)
    yaw(right_hori);
}

void right_vert_cb( const std_msgs::Float32& msg) {
  right_vert = mapf(-1.0 * msg.data, POTEN_LOW, POTEN_HIGH, PULSE_WIDTH_LOW, PULSE_WIDTH_HIGH);
  // pitch(right_vert);
}


void right_upper_bumper_cb(const std_msgs::Float32& msg) {
  rub = msg.data;
}

void right_bottom_bumper_cb(const std_msgs::Float32& msg) {
  rbb = msg.data;
}

ros::Subscriber<std_msgs::Float32> lh_sub("/controller/left_hori", &left_hori_cb );
ros::Subscriber<std_msgs::Float32> lv_sub("/controller/left_vert", &left_vert_cb );
ros::Subscriber<std_msgs::Float32> rh_sub("/controller/right_hori", &right_hori_cb );
ros::Subscriber<std_msgs::Float32> rv_sub("/controller/right_vert", &right_vert_cb );
ros::Subscriber<std_msgs::Float32> up_button_sub("/controller/right_topbumper", &right_bottom_bumper_cb);
ros::Subscriber<std_msgs::Float32> down_button_sub("/controller/right_bottombumper", &right_upper_bumper_cb);

// ===================================================
// ====================== setup ======================
// ===================================================
// the setup routine runs once when you press reset:

void setup() {

  delay(1000);

  lcd.begin();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("CURC Controller");
  lcd.setCursor(0, 1);
  lcd.print("V1.0");

  delay(500);

  lcd.clear();

  // init node handler
//  nh.initNode();
//  nh.subscribe(lh_sub);
//  nh.subscribe(lv_sub);
//  nh.subscribe(rh_sub);
//  nh.subscribe(rv_sub);
//  nh.subscribe(up_button_sub);
//  nh.subscribe(down_button_sub);
//
//  nh.advertise(debug_pub_1);

  // initialize serial communication
  motor_fr.attach(MOTOR_PORT_1);
  motor_fl.attach(MOTOR_PORT_2);
  motor_rr.attach(MOTOR_PORT_3);
  motor_rl.attach(MOTOR_PORT_4);
  motor_lu.attach(MOTOR_PORT_5);
  motor_fu.attach(MOTOR_PORT_6);
  motor_ru.attach(MOTOR_PORT_7);
  motor_bu.attach(MOTOR_PORT_8);

  motor_fl.writeMicroseconds(PULSE_OFF);
  motor_fr.writeMicroseconds(PULSE_OFF);
  motor_rr.writeMicroseconds(PULSE_OFF);
  motor_rl.writeMicroseconds(PULSE_OFF);
  motor_lu.writeMicroseconds(PULSE_OFF);
  motor_fu.writeMicroseconds(PULSE_OFF);
  motor_ru.writeMicroseconds(PULSE_OFF);
  motor_bu.writeMicroseconds(PULSE_OFF);

  Wire1.begin();
  setup_mpu_6050();
  delay(100);
  calibrate_mpu();

  lcd.setCursor(0, 0);                                                 //Set the LCD cursor to position to position 0,0
  lcd.print("Left :");                                                 //Print text to screen
  lcd.setCursor(0, 1);                                                 //Set the LCD cursor to position to position 0,1
  lcd.print("Right:");                                                 //Print text to screen

  delay(500);

  resetPID();
  
  loop_timer = micros();
}

// ===========================================================
// ========================== loop ===========================
// ===========================================================
// the loop routine runs over and over again forever:
void loop() {

  int tbefore = micros();
  
  //---------------------------------------------------------------------------------------
  read_mpu_data();
  //---------------------------------------------------------------------------------------
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_x / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_y / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_z / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  //---------------------------------------------------------------------------------------
  calculate_angle();
  //---------------------------------------------------------------------------------------

  pid_roll_setpoint = 0;
//  if (receiver_input_channel_4 > 1510) pid_roll_setpoint = receiver_input_channel_4 - 1510;
//  else if (receiver_input_channel_4 < 1490) pid_roll_setpoint = receiver_input_channel_4 - 1490;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  pid_pitch_setpoint = 0;
//  if (receiver_input_channel_3 > 1510) pid_pitch_setpoint = receiver_input_channel_3 - 1510;
//  else if (receiver_input_channel_3 < 1490) pid_pitch_setpoint = receiver_input_channel_3 - 1490;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  pid_yaw_setpoint = 0;
  
  //---------------------------------------------------------------------------------------
  calculate_pid();
  //---------------------------------------------------------------------------------------

  write_LCD();

  // -----debugging only -----
//  freq = calculate_freq(time_diff);
//  debug_msg_1.data = freq;
//  debug_pub_1.publish(&debug_msg_1);
  // -----debugging only -----

  // motor_lu.writeMicroseconds(throttle - pid_output_roll); //( throttle - pid_output_pitch + pid_output_roll - pid_output_yaw ); // (front-right - CCW)
  // motor_fu.writeMicroseconds( throttle + pid_output_pitch + pid_output_roll + pid_output_yaw ); // (rear-right - CW)
  // motor_ru.writeMicroseconds(throttle + pid_output_roll); //( throttle + pid_output_pitch - pid_output_roll - pid_output_yaw ); // (rear-left - CCW)
  // motor_bu.writeMicroseconds( throttle - pid_output_pitch - pid_output_roll + pid_output_yaw ); // (front-left - CW)

  if (!x && !y && !yaw_on) {
    move_y(PULSE_OFF);
  }

  // nh.spinOnce();
  // delay(delay_sec);        // delay in between reads for stability

  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  time_diff = micros() - tbefore;
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
  motor_bu.writeMicroseconds(v);
  motor_fu.writeMicroseconds(v);
  motor_ru.writeMicroseconds(v);
  motor_lu.writeMicroseconds(v);
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
  motor_bu.writeMicroseconds(reverse_motor(pitch));
  motor_fu.writeMicroseconds(pitch);
}

void roll(float roll) {
  // positive roll is counterclockwise viewed from -y.
  // opposite 5 and 6.
  motor_ru.writeMicroseconds(roll);
  motor_lu.writeMicroseconds(reverse_motor(roll));
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
  float k = 0.0000611;
  angle_pitch += gyro_x * k;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * k;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  float c = k * (3.1415926 / 180.0);
  angle_pitch += angle_roll * sin(gyro_z * c);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * c);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  if (abs(acc_y) < acc_total_vector) angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  if (abs(acc_x) < acc_total_vector) angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= angle_pitch_accel_cal;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= angle_roll_accel_cal;                                               //Accelerometer calibration value for roll

  // set angle values
  angle_pitch = angle_pitch * 0.9900 + angle_pitch_acc * 0.0100;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angle_roll = angle_roll * 0.9990 + angle_roll_acc * 0.0100;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    
  pitch_level_adjust = angle_pitch * 10;
  roll_level_adjust = angle_roll * 10;
}

//Subroutine for reading the raw gyro and accelerometer data
void read_mpu_data() {

  Wire1.beginTransmission(0x68);                                        
  Wire1.write(0x3B);                                                    //Send the requested starting register
  Wire1.endTransmission();                                              //End the transmission
  Wire1.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire1.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire1.read() << 8 | Wire1.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire1.read() << 8 | Wire1.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire1.read() << 8 | Wire1.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire1.read() << 8 | Wire1.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire1.read() << 8 | Wire1.read();                               //Add the low and high byte to the gyro_x variable
  gyro_y = Wire1.read() << 8 | Wire1.read();                               //Add the low and high byte to the gyro_y variable
  gyro_z = Wire1.read() << 8 | Wire1.read();                               //Add the low and high byte to the gyro_z variable

  gyro_y = gyro_y * -1;
  gyro_z = gyro_z * -1;

}

void calibrate_mpu() {

  lcd.clear();
  lcd.print("Calibrating");
  lcd.setCursor(0, 1);
  for (int cal_int = 0; cal_int < 2000; cal_int ++) {                 //Run this code 2000 times
    if (cal_int % 125 == 0) lcd.print(".");                          //Print a dot on the LCD every 125 readings
    read_mpu_data();                                                   //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(1);
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;
  lcd.clear();
}

void setup_mpu_6050() {

  Wire1.beginTransmission(0x68);
  Wire1.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire1.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire1.endTransmission();

  Wire1.beginTransmission(0x68);
  Wire1.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire1.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire1.endTransmission();                                                    //End the transmission wit

  Wire1.beginTransmission(0x68);
  Wire1.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire1.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire1.endTransmission();

  //Let's perform a random register check to see if the values are written correct
  Wire1.beginTransmission(0x68);
  Wire1.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire1.endTransmission();                                                    //End the transmission
  Wire1.requestFrom(0x68, 1);                                                 //Request 1 bytes from the gyro
  while (Wire1.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire1.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(12, HIGH);                                                       //Turn on the warning led
    lcd.setCursor(0, 0);                                                          //Set the LCD cursor to position to position 0,0
    lcd.print("Init Sensors Failed");                                             //Print text to screen
    while (1) digitalWrite(12, HIGH);                                             //Stay in this loop for ever
  }

  Wire1.beginTransmission(0x68);
  Wire1.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire1.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire1.endTransmission();
}
// ===========================================================
// =========================== LCD ===========================
// ===========================================================
void write_LCD() {

  //Subroutine for writing the LCD
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if (lcd_loop_counter == 14)lcd_loop_counter = 0;                     //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if (lcd_loop_counter == 1) {
    angle_pitch_buffer = throttle - pid_output_pitch;// throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;                      //Buffer the pitch angle because it will change
    lcd.setCursor(6, 0);                                               //Set the LCD cursor to position to position 0,0
  }
  if (lcd_loop_counter == 2) {
    if (angle_pitch_buffer < 0)lcd.print("-");                         //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if (lcd_loop_counter == 3)lcd.print(abs(angle_pitch_buffer) / 1000); //Print first number
  if (lcd_loop_counter == 4)lcd.print((abs(angle_pitch_buffer) / 100) % 10); //Print second number
  if (lcd_loop_counter == 5)lcd.print((abs(angle_pitch_buffer) / 10) % 10); //Print third number
  // if (lcd_loop_counter == 6)lcd.print(".");                            //Print decimal point
  if (lcd_loop_counter == 7)lcd.print(abs(angle_pitch_buffer) % 10);   //Print decimal number

  if (lcd_loop_counter == 8) {
    angle_roll_buffer = throttle + pid_output_pitch; // right
    lcd.setCursor(6, 1);
  }
  if (lcd_loop_counter == 9) {
    if (angle_roll_buffer < 0)lcd.print("-");                          //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if (lcd_loop_counter == 10)lcd.print(abs(angle_roll_buffer) / 1000); //Print first number
  if (lcd_loop_counter == 11)lcd.print((abs(angle_roll_buffer) / 100) % 10); //Print second number
  if (lcd_loop_counter == 12)lcd.print((abs(angle_roll_buffer) / 10) % 10); //Print third number
  // if (lcd_loop_counter == 13)lcd.print(".");                           //Print decimal point
  if (lcd_loop_counter == 14)lcd.print(abs(angle_roll_buffer) % 10);   //Print decimal number
}

// ===========================================================
// =========================== PID ===========================
// ===========================================================
void calculate_pid() {
  
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void resetPID() {

  angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
  set_gyro_angles = true;
  
  //reset all PID controllers
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;

}
