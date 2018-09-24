/*
 * limbitless_v3
 *
 * Title: Limbitless v3
 * Author: Tyler Petresky <tylerpetresky.com>
 * Date: 7-14
 *
 * Desc: Basic code to allow the actuation a servo (closing of
 *       a prosthetic hand) based on bicep movement.
 *       The hand should open and close using a toggle
 *       method.
 */

// Digital pin for servo motor
const int pin_servo = 9;

// Analog pin for the muscle sensor
const int pin_muscle = A0;

// define pinout for rgb led
const int pin_rgb_r = 7;
const int pin_rgb_g = 5;
const int pin_rgb_b = 6;

// define pin for mode button
const int pin_btn = 3;

// scale value for led dimming (value between 0 and 1) (eg: .5 is half powa!!)
float led_scalar = .2;

// Threshold value that allows the hand to open and close.
// The scaled value of the muscle sensor's value is compared
// to this threshold value.
// Sample 6-year old was 20.0, Adult was 100.0
float toggle_threshold = 40.0;

// this value will be added to the toggle_threshold when the sensitivity_state increases
float sensitivity_steps = 10.0;


/******************************************************************************
* FOR NORMAL WORKING, DON'T CHANGE THE VARIABLES BENEATH THIS LINE OF COMMENT *
******************************************************************************/

// Library for button press duration detection
// library can be downloaded at https://www.arduinolibraries.info/libraries/bounce2
// current working version when writing this code is: 2.52.0
#include <Bounce2.h>

// Library for servo use
#include <Servo.h>

int current_servo_pos = 0;

// buffer to store initial toggle threshold. Toggle treshold is a dynamic var
float toggle_threshold_buffer = 0;

// current sensitivity state (can range between 1 and 4)
unsigned int sensitivity_state = 1;

// log var for button press.
unsigned long button_pressed_millis = 0;

// Maximum value of the timer value. Prevents overflow errors.
//int timer_threshold = 500;
const int timer_threshold = 25;

// Current state of the hand's position
boolean hand_opened = true;
/** hand modus
* Mode 1: toggle trigger, when the hand is triggered, the hand closes, when it is triggered again, the hand wil open
* Mode 2: when the hand is triggered the hand will close, when the muscle is relaxed, the hand wil open (This mode is disabled for now, sensor data is not relayable for this kind of usage, Mode 2 is now the mute mode)
*/
int hand_mode = 1;

// The angle of the servo in the opened and closed states
int opened_angle = 45, closed_angle = 0;

// Servo object
Servo servo;

// Timer used to allow the muscle to relax before toggling the
// hand. Prevents toggling too quickly.
int servo_timer = 0;

// Instantiate a Bounce object :
Bounce debouncer = Bounce();

unsigned long buttonPressTimeStamp;

// Sets up the system. Runs once on startup.
void setup()
{
  // Uncomment this for calibration purposes.
  Serial.begin(9600);

  // Setup the button with an internal pull-up :
  pinMode(pin_btn,INPUT_PULLUP);

  pinMode(pin_rgb_r, OUTPUT);
  pinMode(pin_rgb_g, OUTPUT);
  pinMode(pin_rgb_b, OUTPUT);

  // Startup light sequence
  writeLedData(255,0,0);
  delay(200);
  writeLedData(0,255,0);
  delay(200);
  writeLedData(0,0,255);
  delay(200);
  writeLedData(0,0,0);
  delay(500);
  writeLedData(255,0,0);

  // set init value for toggle_threshold_buffer
  toggle_threshold_buffer = toggle_threshold;

  // After setting up the button, setup the Bounce instance :
  debouncer.attach(pin_btn);
  debouncer.interval(5);

  pinMode(pin_muscle, INPUT);

  // Assign the servo to it's respective pin
  servo.attach(pin_servo);

  // Set default angle of servo as opened
  servo.write(opened_angle);
}

void loop()
{
  // Update the Bounce instance :
  debouncer.update();

  // register the time we start pussing the button
  if(debouncer.rose()) {
    button_pressed_millis = millis();
  }

  // actions when the button is pressed
  if ( debouncer.fell() ) {
    // check if we need to invert the input
    if(millis() - button_pressed_millis > 2000) {
      invertInput();
      button_pressed_millis = 0;
    } else {
      sensitivity_state++;
      toggle_threshold = toggle_threshold + sensitivity_steps;
    }

    if(sensitivity_state > 4) {
      sensitivity_state = 1;
      toggle_threshold = toggle_threshold_buffer;
    }

    if(sensitivity_state == 1) {
      writeLedData(255,0,0);
    } else if(sensitivity_state == 2) {
      writeLedData(0,255,0);
    } else if(sensitivity_state == 3) {
      writeLedData(0,0,255);
    } else if(sensitivity_state == 4) {
      writeLedData(255,125,0);
    }
  }

  // Raw value of the muscle sensor reading
  float muscle_sensor_value = analogRead(pin_muscle),
  // Muscle sensor value scaled down just for easier working
        muscle_sensor_scaled = muscle_sensor_value * (180.0 / 1023.0);
        muscle_sensor_scaled = muscle_sensor_value * (180.0 / 1023.0);

  // Conditions to toggle the position of the hand.
  // 1. We are above the threshold for movement.
  // 2. The timer is at it's max value.
  if (muscle_sensor_scaled >= toggle_threshold && servo_timer == timer_threshold && hand_mode == 1)
  {
      // Change position of hand
      hand_opened = !hand_opened;

      if (hand_opened) {
        for(int pos = opened_angle * 2; pos > closed_angle * 2; pos-=2)
        { // Closes the hand by gradually adjusting the written angle.
          servo.write(pos);
          current_servo_pos = pos;
          delay(2);
        }
      } else {
        for(int pos = closed_angle * 2; pos < opened_angle * 2; pos+=2)
        { // Opens the hand by gradually adjusting the written angle.
          servo.write(pos);
          current_servo_pos = pos;
          delay(2);
        }
      }

      // Reset the timer
      servo_timer = 0;
  }


  // Remove these before deployment.
  // Uncomment for calibration purposes.
  // To calibrate, have the person of interest
  // flex the desired muscle and watch where the
  // value of muscle_sensor_scaled peaks. Afterwards,
  // change the value of toggle_threshold accordingly.
  /*Serial.print("Toggle Threshold: ");
  Serial.print(toggle_threshold);
  Serial.print("\tSensor raw: ");
  Serial.print(pin_muscle);
  Serial.print("\tSensor: ");
  Serial.print(muscle_sensor_value);
  Serial.print("\tScaled:  ");
  Serial.print(muscle_sensor_scaled);
  Serial.print("\tServoTimer: ");
  Serial.print(servo_timer);
  Serial.print("\tHandOpen: ");
  Serial.print((hand_opened ? "OPEN  " : "CLOSED"));
  Serial.print("\tServo position: ");
  Serial.print(current_servo_pos);
  Serial.println("");*/

  Serial.print("Sensor Data: ");
  Serial.print(muscle_sensor_scaled);
  Serial.print("\t Treshold: ");
  Serial.println(toggle_threshold);

  // Don't allow the servo_timer to get too big. Overflow errors
  // crash the Arduino.
  if (servo_timer < timer_threshold)
    servo_timer++;

  // Delay for the servo. Don't want to overload it.
  delay(1);
}

/**
*  Change led color
*/
void writeLedData(int red, int green, int blue) {
  led_scalar = led_scalar > 1 ? 1 : led_scalar;

  red = red * led_scalar;
  green = green * led_scalar;
  blue = blue * led_scalar;

  analogWrite(pin_rgb_r, red);
  analogWrite(pin_rgb_g, green);
  analogWrite(pin_rgb_b, blue);
}


/**
* This function changes the working mode for the hand
*/

void invertInput() {
  hand_mode = hand_mode == 1 ? 2 : 1;

  // Startup light sequence
  writeLedData(0,0,0);
  delay(200);
  writeLedData(255,255,255);
  delay(200);
  writeLedData(0,0,0);
  delay(200);
  writeLedData(255,255,255);
  delay(200);
  writeLedData(0,0,0);
  delay(200);

  writeLedData(255,0,0);
}
