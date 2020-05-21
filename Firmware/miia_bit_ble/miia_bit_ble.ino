/* MiiCode Firmware for MiiA.bit Robot
 *  Version 1.2
 *  Date: May 2020
 *  Author: Tyrone van Balla
 *  Company: RD9 Solutions
 */
 
// included libraries
#include <SoftwareSerial.h>
#include <SparkFun_TB6612.h>

// pin assignments

// bluetooth module communication
#define rx_pin 7
#define tx_pin 8

// servo motor
#define servo_arm_pin 17

// ultrasonic sensor
#define uss_trigger_pin 19
#define uss_echo_pin 18

// RBG LED
#define led_r_pin 14
#define led_g_pin 15 
#define led_b_pin 16 

// DC motors
#define motor_a_forward_pin 4
#define motor_a_reverse_pin 2
#define motor_a_pwm 3
#define motor_b_forward_pin 6
#define motor_b_reverse_pin 9
#define motor_b_pwm 10
#define motor_stand_by MOSI

// buzzer
#define buzzer_pin 5

// push button
#define push_button_pin A7

// constants

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int motor_offset_a = 1;
const int motor_offset_b = 1;

#define baud_rate 57600

#define servo_calibration_pos 90

int lenMicroSecondsOfPeriod = 25 * 1000; // 25 milliseconds (ms)
int lenMicroSecondsOfPulse = 1 * 1000; // 1 ms is 0 degrees
int min_pos = 1 * 1000; //0.5ms is 0 degrees in HS-422 servo
int max_pos = 2 * 1000;
int increment = 0.01 * 1000;
int current_pos = 0;
int servo_pos = servo_calibration_pos;

int current_byte = 0;
int prev_byte = 0;
int input_button = 0;
int cntr = 0;
int motor_direction = 0;
int dist;

// variables for creating delay in sending data
unsigned long previous_timestamp = 0;
unsigned long current_timestamp;
const long time_delay = 1250; // milliseconds

// create objects

SoftwareSerial mySerial (rx_pin, tx_pin); // RX, TX from Arduino's Perspective

// Initializing motors
Motor motor_a = Motor(motor_a_forward_pin, motor_a_reverse_pin, motor_a_pwm, motor_offset_a, motor_stand_by);
Motor motor_b = Motor(motor_b_forward_pin, motor_b_reverse_pin, motor_b_pwm, motor_offset_b, motor_stand_by);

// function prototypes
void configure_pins(void);
void initialize_pins(void);
void motor_control(int motor_forward_pin, int motor_reverse_pin, int speed);
void initialization_ok(void);
long read_distance_uss(void);
void receive_data_from_miicode(void);
void outputs_set(void);
void send_data_to_miicode(void);
void servo_off(int servo_pin);
void set_servo_position(int servo_pin, int pos);

void setup()
{

  // configure all pin modes
  configure_pins();

  // initialize all pins
  initialize_pins();

  // board initialization - ok!
  initialization_ok();

  // set calibration pin and drive pin to calibrated position
  set_servo_position(servo_arm_pin, servo_calibration_pos);

  // initialize serial communications at specified baud rate
  mySerial.begin(baud_rate);
  
  // setup complete
}

void loop()
{
    // receive data from the app
    receive_data_from_miicode();
    
    // hold servo
    set_servo_position(servo_arm_pin, servo_pos);

    // check time passed
    current_timestamp = millis();
    if (current_timestamp - previous_timestamp >= time_delay)
    {
      // reset the previous timestamp
      previous_timestamp = current_timestamp;
      
      // send data
      send_data_to_miicode();
    }

    // hold servo
    set_servo_position(servo_arm_pin, servo_pos);
}

// board initialization - ok!: led red flash, then blue, then blue, then off
void initialization_ok()
{
  analogWrite(led_r_pin, 0);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 0);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 0);
  delay(500);
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);
}

// ultrasonic sensor function
long read_distance_uss()
{
  
  long duration, distance;
  digitalWrite(uss_trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(uss_trigger_pin, HIGH);

  delayMicroseconds(5);
  digitalWrite(uss_trigger_pin, LOW);
  duration = pulseIn(uss_echo_pin, HIGH, 5000);
  distance = (duration/2) / 29.1;
  
  dist = distance;
}

// receives data from the miicode application via Bluetooth
void receive_data_from_miicode(void)
{
  if (mySerial.available() > 0)
  {
    // catch any motor related commands as these commands consist of three parts
    if ((prev_byte == 202 or prev_byte == 203 or prev_byte == 235) and cntr == 0)
    {      
      // current byte will be the direction
      motor_direction = mySerial.read();
      // keep prev_byte
      cntr = 1;
    }
    else
    {
      current_byte = mySerial.read();
      outputs_set();
      prev_byte = current_byte;
    }
  } 
}

// function to perform output based on data received from MiiCode
void outputs_set()
{   
  // buzzer on/off set tone
  if (prev_byte == 201)
  {
    // turn the buzzer on
    if (current_byte == 1)
    {
      tone(buzzer_pin, 2400);
    }

    // turn the buzzer off
    if (current_byte == 0)
    {
      noTone(buzzer_pin);
    }
  }

  // Motor A
  if (prev_byte == 202)  
  {
    // prev_byte is the motor
    // current_byte is the speed
    // motor_direction is the direction
    if (current_byte == 0)
    {
      // stop
      motor_a.brake();
      cntr = 0;
    }
    
    if (motor_direction == 0)
    {
      // reverse direction
      current_byte = current_byte * -1 *2.55;
    }
        else
    {
      current_byte = current_byte * 2.55;
    }
    
    motor_a.drive(current_byte);
    cntr = 0;
  }

  // Motor B
  if (prev_byte == 203) 
  {
    if (current_byte == 0)
    {
      // stop
      motor_b.brake();
      cntr = 0;
    }
    if (motor_direction == 0)
    {
      // reverse direction
      current_byte = current_byte * -1 * 2.55;
    }
    else
    {
      current_byte = current_byte * 2.55;
    }
    
    motor_b.drive(current_byte);
    cntr = 0;
  }

  // Motor A and B (simultaneous control)
  if (prev_byte == 235) 
  {
    if (current_byte == 0)
    {
      // stop
      motor_a.brake();
      motor_b.brake();
      cntr = 0;
    }
    
    if (motor_direction == 0)
    {
      // reverse direction
      current_byte = current_byte * -1 * 2.55;
    }
    else
    {
      current_byte = current_byte * 2.55;
    }

    motor_a.drive(current_byte);
    motor_b.drive(current_byte);
    cntr = 0;
  }

  // RGB Output
  if (prev_byte == 204)
  {
      analogWrite(led_r_pin, 255-(current_byte*2.55));
  }
  // rGb output
  if (prev_byte == 205)
  {
      analogWrite(led_g_pin, 255-(current_byte*2.55));
  }
  // rgB output
  if (prev_byte == 206)
  {
      analogWrite(led_b_pin, 255-(current_byte*2.55));
  }

  // Servo Motor Output
  if (prev_byte == 208)
  {
      // get new servo pos
      servo_pos = current_byte;
      // set new servo pos
      set_servo_position(servo_arm_pin, current_byte);
  }
}

// sends data to the miicode application via Bluetooth
void send_data_to_miicode(void)
{
  // push button input
  mySerial.write('abc');
  
  if (analogRead(push_button_pin) > 512){
    input_button = 0;
  }
  else{
    input_button = 1;
  }
  
  mySerial.write(input_button);
  mySerial.write('abc');
  
  // ultra sonic sensor output
  read_distance_uss();
  mySerial.write('xyz');
  mySerial.write(dist);
  mySerial.write('xyz');
}

// function to configure pins and pin modes
void configure_pins(void)
{
  
  // miia.bit servo motor signal pins
  pinMode(servo_arm_pin, OUTPUT);

  // ultrasonic sensor trigger and echo pins
  pinMode(uss_trigger_pin, OUTPUT);
  pinMode(uss_echo_pin, INPUT);

  // dc motor pins
  // configured by motor library  

  // RGB LED pins
  pinMode(led_r_pin, OUTPUT);
  pinMode(led_g_pin, OUTPUT);
  pinMode(led_b_pin, OUTPUT);

  // buzzer
  pinMode(buzzer_pin, OUTPUT);

  // button
  pinMode(push_button_pin, INPUT);
}

// function to set the default pin states
void initialize_pins()
{
  // turn all LEDs off
  analogWrite(led_r_pin, 255);
  analogWrite(led_g_pin, 255);
  analogWrite(led_b_pin, 255);

  // turn buzzer off
  noTone(buzzer_pin);

  // reset servo to calibrated position
  set_servo_position(servo_arm_pin, servo_calibration_pos);

}

// function to set position of a servo motor
void set_servo_position(int servo_pin, int pos)
{
  int pos_setting = ((pos*1.8) * 5.50) + 1000;
  if (pos_setting >= current_pos)
  {
    for(int pwm = current_pos; pwm <= pos_setting; pwm += increment)
    {
      digitalWrite(servo_pin, HIGH);
      delayMicroseconds(pwm);
      digitalWrite(servo_pin, LOW);
      delay(15);
      
    }
  }

  if (pos_setting <= current_pos)
  {
    for(int pwm = current_pos; pwm >= pos_setting; pwm -= increment)
    {
      digitalWrite(servo_pin, HIGH);
      delayMicroseconds(pwm);
      digitalWrite(servo_pin, LOW);
      delay(15);
      
    }
  }
  current_pos = pos_setting;
}

//end
