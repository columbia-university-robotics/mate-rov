#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68

float angle_pitch, angle_roll;
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;
long loop_timer;
int temperature;

const float angle_pitch_accel_cal = 0.628178439;
const float angle_roll_accel_cal = -1.425186567;

void setup() {

  Wire.begin();

  Serial.begin(9600);

  setup_mpu_6050();

  delay(1000);

  // put your setup code here, to run once:
  pinMode(9,OUTPUT);

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0) Serial.print(".");                           //Print a dot on the LCD every 125 readings
    read_mpu_data();                                                   //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(1);                                                          
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  Serial.println("Finished");
  loop_timer = micros();                                               //Reset the loop timer
}

void loop() {

  digitalWrite(9,HIGH);
  int x = millis();
  read_mpu_data();
  calculate_angle();
  print_mpu_data();
  delay(1);
  while(micros() - loop_timer < 33333);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                                //Reset the loop timer
  digitalWrite(9, LOW);
  // put your main code here, to run repeatedly:
}

void calculate_angle() {

  float hz = 30;

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  
  //0.0000611 = 1 / (250Hz / 65.5)
  float k = 1.0 / hz / 65.5;
  angle_pitch += gyro_x * k;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * k;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  float c = k * (3.1415926 / 180.0);
  angle_pitch += angle_roll * sin(gyro_z * c);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * c);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= angle_pitch_accel_cal;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= angle_roll_accel_cal;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9990 + angle_pitch_acc * 0.0010;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9990 + angle_roll_acc * 0.0010;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
}

void print_mpu_data(){

  Serial.print("Pitch: ");
  Serial.println(angle_pitch);
  Serial.print("Roll: ");
  Serial.println(angle_roll);
}

//Subroutine for reading the raw gyro and accelerometer data
void read_mpu_data() {

  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

  gyro_y = gyro_y * -1;
  gyro_z = gyro_z * -1;
  
}

void setup_mpu_6050() {

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission wit

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(0x68, 1);                                                 //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(12, HIGH);                                                  //Turn on the warning led
    Serial.println("Failed");  
    while (1) digitalWrite(12, HIGH);                                        //Stay in this loop for ever
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();
  
}
