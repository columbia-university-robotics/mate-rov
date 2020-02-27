// define all constants
int MOTOR_PORT_1 = 12;
int MOTOR_PORT_2 = 11;
int MOTOR_PORT_3 = 10;
int MOTOR_PORT_4 = 9;
int MOTOR_PORT_5 = 8;
int MOTOR_PORT_6 = 7;
int MOTOR_PORT_7 = 6;
int MOTOR_PORT_8 = 5;

int POTEN_LOW = -1;
int POTEN_HIGH = 1;
int PULSE_WIDTH_LOW = 1100;
int PULSE_WIDTH_HIGH = 1900;
int PULSE_SLOW_OFFSET = 100; // subtract the high by the offset to get a smaller high
int PULSE_OFF = PULSE_WIDTH_LOW + (PULSE_WIDTH_HIGH - PULSE_WIDTH_LOW) / 2;
int BITS_PER_SEC = 57600;
float MIN_STICK_THRESHOLD = 0.01;
