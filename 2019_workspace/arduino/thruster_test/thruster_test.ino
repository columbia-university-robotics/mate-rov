//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, timer_channel_6, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

//Setup routine
void setup() {
  Serial.begin(9600);
  DDRD |= B11111100;                                 //Configure digital poort 2, 3, 4, 5, 6 and 7 as output

  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change

  PORTD |= B11111100;                              //Set digital poort 4, 5, 6 and 7 high.
  delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
  PORTD &= B00111111;                              //Set digital poort 4, 5, 6 and 7 low.
  delay(3);
  Serial.println("Starting");
  digitalWrite(12, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.

  receiver_input_channel_3 = 1600;
}

//Main program loop
void loop() {

//  if (receiver_input_channel_3 < 1300) {
//    receiver_input_channel_3++;
//  }


  while (zero_timer + 4000 > micros());                      //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.

  PORTD |= B11111100; //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 4 is set low.
  timer_channel_2 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 5 is set low.
  timer_channel_3 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 6 is set low.
  timer_channel_4 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 7 is set low.
  timer_channel_5 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 7 is set low.
  timer_channel_6 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 7 is set low.


  while (PORTD >= 16) {                                      //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B11111011; //When the delay time is expired, digital port 4 is set low.
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B11110111;//Check the current time.
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low.
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low.
    if (timer_channel_5 <= esc_loop_timer) PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low.
    if (timer_channel_6 <= esc_loop_timer) PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low.
  }
}

//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect) {

  current_time = micros();

  //Channel 1=========================================
  //pulse raise
  if (PINB & B00000001) {                                      //Is input 8 high?
    if (last_channel_1 == 0) {                                 //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  //pulse falls
  else if (last_channel_1 == 1) {                              //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }

  //Channel 2=========================================
  if (PINB & B00000010 ) {                                     //Is input 9 high?
    if (last_channel_2 == 0) {                                 //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1) {                              //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2 ;   //Channel 2 is current_time - timer_2 ||During testing, we are using the red rc ||
  }
  //Channel 3=========================================
  if (PINB & B00000100 ) {                                     //Is input 10 high?
    if (last_channel_3 == 0) {                                 //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1) {                              //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3
  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                     //Is input 11 high?
    if (last_channel_4 == 0) {                                 //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1) {                              //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

//Subroutine for displaying the receiver signals
void print_signals() {
  Serial.print("Roll:");
  if (receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Nick:");
  if (receiver_input_channel_2 - 1520 > 0)Serial.print("^^^");
  else if (receiver_input_channel_2 - 1480 < 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Gas:");
  if (receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
  else if (receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if (receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}
