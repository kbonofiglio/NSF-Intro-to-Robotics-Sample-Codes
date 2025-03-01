/*
 * Activity 01 -- Move it!
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

// TODO, Section, 4.2: Add line to include Chassis.h

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

//Set up the chassis
Chassis chassis(7.0, 1440, 14.7);

// A helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  // TODO: call chassis.idle() to stop the motors
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("/idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // Initialize the chassis (which also initializes the motors)
  chassis.init();

  // TODO: adjust the PID coefficients
  chassis.setMotorPIDcoeffs(5, 0.5);

  idle();

  // Initializes the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section 6.1 (but not before!): Edit the function definition to accept a distance and speed
void drive(void)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: make a call to chassis.driveFor()
  Serial.println("I should be driving!");
  chassis.driveFor(dist, speed);
  //chassis.setMotorEfforts(60, 60);

}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO, Section 6.1: Make a call to chassis.turnFor()

}

// TODO, Section 6.1: Declare function handleMotionComplete(), which calls idle()

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  // TODO: add "emergency stop"
  if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == UP_ARROW) drive(50, 10);
      break;
      
    default:
      break;
  }
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  // TODO, Section 3.1: Temporarily edit to pass true to getKeyCode()
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 

      // TODO, Section 6.1: Uncomment to handle completed motion
      // if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;

    default:
      break;
  }
}
