/*
 * Activity 01 -- Move it!
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>

// Setup the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

//TODO: set up the chassis

// Helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Define the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR};
ROBOT_STATE robotState = ROBOT_IDLE;

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  // TODO: call chassis.idle() to stop the motors

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

  // TODO: initialize the chassis (which also initializes the motors)

  // TODO: adjust the PID coefficients

  idle();

  // initialize the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// In a Section 6, we'll edit the function definition to accept a distance and speed
void drive(void)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: In Section 4.2, add a call to chassis.setWheelSpeeds() to test the motion

  // TODO: In Section X.X, remove the call to setWheelSpeeds() and add a call to chassis.driveFor()

}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: make a call to chassis.driveFor()

}

// TODO: declare function handleMotionComplete() --> idle

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  // TODO: add "emergency stop"
  // if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      // TODO: handle IR keys
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
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 

      // TODO: uncomment in Section ??
      // if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;

    default:
      break;
  }
}
