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

const int pin_battery = A2;
const int pin_battery_deadswitch = 12
const float battery_lowest_allowed_voltage = 3.5;

// define pinout for rgb led
// keep in mind, this pins need to support PWM signal
const int pin_rgb_r = 11;
const int pin_rgb_g = 5;
const int pin_rgb_b = 6;

// define pin for mode button
const int pin_btn = 3;

// scale value for led dimming (value between 0 and 1) (eg: .5 is half powa!!)
float led_scalar = .3;

// Threshold value that allows the hand to open and close.
// The scaled value of the muscle sensor's value is compared
// to this threshold value.
// Sample 6-year old was 20.0, Adult was 100.0
float toggle_threshold = 40.0;

// this value will be added to the toggle_threshold when the sensitivity_state increases
float sensitivity_steps = 7.5;

// this value will be used to cut the power to the servo when the battery voltage will drop below a certain level.
float battery_voltage_threshold = 6.0;

// this determines smoothness  - .0001 is low, realtime data, 1 is max smoothness
float filterVal = .85;


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
* Mode 2: when the hand is triggered the hand will close, when the muscle is relaxed, the hand wil open
* Mode 3: Mute/Idle mode.  (get your popcorn and watch a movie mode)
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

float smoothedVal;

// variables for voltage devider
float battery_value = 0;
float battery_voltage = 0.0;            // calculated voltage

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

  pinMode(pin_battery_deadswitch, OUTPUT);

  // Assign the servo to it's respective pin
  servo.attach(pin_servo);

  // Set default angle of servo as opened
  servo.write(opened_angle);
}

void loop()
{
  // Update the Bounce instance :
  debouncer.update();

  // read battery voltage
  battery_value = smooth(analogRead(pin_battery), filterVal, battery_value);
  battery_voltage = ((battery_value * 5.015) / 1024) * 11.132;

  // check if battery needs to be disconnected
  if(pin_battery_deadswitch == HIGH) {
    // if battery is switched off, check if voltage is high enough to switch it back on
    if(battery_voltage >= (battery_lowest_allowed_voltage+0.2)) {
      digitalWrite(pin_battery_deadswitch, LOW);
    }
  } else {
    // if battery is switched on, check if voltage is low enough to switch it off
    if(battery_voltage < battery_lowest_allowed_voltage) {
      digitalWrite(pin_battery_deadswitch, HIGH);
    }
  }

  // register the time we start pussing the button
  if(debouncer.rose()) {
    button_pressed_millis = millis();
  }

  // actions when the button is pressed
  if ( debouncer.fell() ) {
    // check if we need to invert the input
    if(millis() - button_pressed_millis > 2000) {
      modeSelect();
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

  smoothedVal =  smooth(analogRead(pin_muscle), filterVal, smoothedVal);
  // Raw value of the muscle sensor reading
  float muscle_sensor_value = smoothedVal,
  // Muscle sensor value scaled down just for easier working
        muscle_sensor_scaled = muscle_sensor_value * (180.0 / 1023.0);
        muscle_sensor_scaled = muscle_sensor_value * (180.0 / 1023.0);

  // Mode 1: toggle trigger, when the hand is triggered, the hand closes, when it is triggered again, the hand wil open
  if (muscle_sensor_scaled >= toggle_threshold && servo_timer == timer_threshold && hand_mode == 1)
  {
    // Conditions to toggle the position of the hand.
    // 1. We are above the threshold for movement.
    // 2. The timer is at it's max value.
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

  // Mode 2: when the hand is triggered the hand will close, when the muscle is relaxed, the hand wil open
  if(hand_mode == 2) {
    if(muscle_sensor_scaled >= toggle_threshold && servo_timer == timer_threshold) {
      // Close hand
      current_servo_pos = closed_angle * 2;
      servo.write(current_servo_pos);
      delay(50);
    } else if(muscle_sensor_scaled < toggle_threshold && servo_timer == timer_threshold) {
      // Open hand
      current_servo_pos = opened_angle * 2;
      servo.write(current_servo_pos);
      delay(50);
    }
  }

  // Mode 3: Mute/Idle mode.  (get your popcorn and watch a movie mode)
  if(hand_mode == 3) {
    // idle mode (get your popcorn and watch a movie mode)
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
  Serial.print(toggle_threshold);
  Serial.print("\t Voltage: ");
  Serial.println(battery_voltage);

  // Don't allow the servo_timer to get too big. Overflow errors
  // crash the Arduino.
  if (servo_timer < timer_threshold)
    servo_timer++;

  // Delay for the servo. Don't want to overload it.
  delay(10);
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

void blinkLed(int red, int green, int blue, int count) {
  for(int i = 0; i < count; i++) {
    writeLedData(red, green, blue);
    delay(200);
    writeLedData(0,0,0);
    delay(200);
  }
}


/**
* This function changes the working mode for the hand
*/

void modeSelect() {
  hand_mode++;

  if(hand_mode > 3) {
    hand_mode = 1;
  }

  if(hand_mode == 1) {
    blinkLed(0,0,255,3);
  } else if(hand_mode == 2) {
    blinkLed(0,255,0,3);
  } else {
    blinkLed(255,0,0,3);
  }

  writeLedData(255,0,0);
}

/**
* Data smoothing, feedback the smoothed data into itself and give it a filterVar
* (smoothnes level between 0 and 1, where 1 is ultra smooth (slow data change))
*/
int smooth(int data, float filterVar, float smoothedVal){

  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}
