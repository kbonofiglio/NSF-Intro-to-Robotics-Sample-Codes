/*
 * Activity 02 -- Staying on track
 *
 * Line following with speed control. Pauses at an intersection and waits for a command.
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>

// Declare a chassis object with nominal dimensions
// TODO, Section 5.2: Adjust the kinematic parameters to what you found in the previous activity
Chassis chassis(7.0, 1440, 14.9);

// Setup the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// Define the robot states
// TODO, Section 5.2: Define a state for line following
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR};
ROBOT_STATE robotState = ROBOT_IDLE;

// TODO, Section 5.2: Define a baseSpeed


// A helper function to stop the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  //stop motors 
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
}

// A helper command to drive a set distance
void drive(float dist, float speed)
{
  Serial.println("drive()");
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;
  chassis.driveFor(dist, speed);
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  Serial.println("turn()");
  setLED(HIGH);
  robotState = ROBOT_DRIVE_FOR;
  chassis.turnFor(ang, speed);
}

// Used to handle when the motions are complete
void handleMotionComplete(void)
{
  idle();
}

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  //ENTER_SAVE idles, regardless of state -- E-stop
  if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == UP_ARROW) drive(50, 10);
      else if(keyPress == DOWN_ARROW) drive(-50, 10);
      else if(keyPress == LEFT_ARROW) turn(90, 45);
      else if(keyPress == RIGHT_ARROW) turn(-90, 45);

      //TODO, Section 5.2: Respond to SETUP_BTN press
    
      break;
      
    // TODO, Section 5.3: respond to speed +/- commands (when in ROBOT_LINING state)
    // Use the VOLplus and VOLminus keys on your IR remote
 
     default:
      break;
  }
}

void handleLineFollowing(float baseSpeed)
{
  // TODO, Section 5.2: Add line following control
}

// TODO, Section 6.1: Define a darkThreshold

bool checkIntersectionEvent(uint16_t darkThreshold)
{
  static bool prevIntersection = false;

  bool retVal = false;

  // TODO, Section 6.1: Add logic to check for intersection
  
  return retVal;
}

void handleIntersection(void)
{
  Serial.println("Intersection!");

  // TODO, Section 6.1: add a line to drive forward to center the robot
  

  robotState = ROBOT_DRIVE_FOR;
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

  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();

  // These values should work well, though you may want to use what you found in the previous activity
  chassis.setMotorPIDcoeffs(5, 0.5);

  // Initialize the IR decoder
  decoder.init();

  // Ensure the line sesnors are inputs
  pinMode(LEFT_LINE_SENSE, INPUT);
  pinMode(RIGHT_LINE_SENSE, INPUT);

  Serial.println("/setup()");
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
       if(chassis.checkMotionComplete()) handleMotionComplete(); 
       break;

    // TODO, Section 5.2: Add a case to handle line following

    // TODO, Section 6.1: check/handle intersection

    default:
      break;
  }
}
