  //CAB 1.1.2022
//smartWander.ino smartWander.cpp
//implement all the layers of the subsumption archiecture or state machine for smart wander which includes.
//random wander, collide and runaway so that the robot explores the world without hitting anything

#include <Arduino.h>                // include Arduino library
#include <wpi-32u4-lib.h>           // include WPI robot library
#include <Rangefinder.h>            // include sonar range finder
#include <Chassis.h>                // define robot chassis
Rangefinder rangefinder(11, 4);     // create instance of sonar echo pin 11, trigger pin 4
Chassis chassis(7.0, 1440, 14.9);   // Declares a chassis object with nominal dimensions
#define LED_PIN 13                  // LED connected to builtin pin 13
#define red_LED 5                   // red LED on pin 5
#define grn_LED 6                   // green LED on pin 
int robotSpeed = 10;                //set robot sp6eed
float distance;                     //sonar distance in cm
float inches;                       //sonar distance in inches
float error;                        //error = desired - actual distance
int wrnDistance = 12;               //obstacle warning distance
int dgrDistance = 6;               //obstacle danger distance

enum ROBOT_STATE {ROBOT_IDLE, ROBOT_WANDER,ROBOT_AVOID}; // Define the robot states
ROBOT_STATE robotState = ROBOT_IDLE;  // initialze robt in wander state


void setLED(bool value)             //function to turn on and off LED on pin 13
{
  //Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

void readSonar(){                         //read ultrasonic range sensor
  distance = rangefinder.getDistance();   //distance in cm
  inches = distance*0.393701;             //distance in inches
}

void idle(void)                         //helper function to stop the motors
{
  //Serial.println("idle()");         //print idle to serial monitor
  digitalWrite(red_LED,LOW);
  digitalWrite(grn_LED,LOW);
  setLED(LOW);                      // turn off LED on pin 13
  chassis.idle();                   //stop motors 
  //robotState = ROBOT_IDLE;        //set state to idle
}


void drive(float dist, float speed)   //helper function to drive a set distance (cm, cm/s)
{
  //Serial.println("drive()");              //print drive to serial monitor
  setLED(HIGH);                           // turn on LED on pin 13
  chassis.setWheelSpeeds(speed, speed);   //MOVES WHEELS FOREVER
  //chassis.driveFor(dist,speed);         //MOVES FOR A CERTAIN DISTANCE
}

void turn(float ang, float speed)     //helper function to turn a set angle (deg, deg/s)
{
  //Serial.println("turn()");
  setLED(HIGH);
  //chassis.setWheelSpeeds(speed, speed);
  chassis.turnFor(ang,speed);
}

//TO DO: Modify this function to make a unique random wander routine
//try changing the robot speed, random number seed and modulo
void randomWander(){      //function to move robot random forward and turn
  digitalWrite(red_LED,LOW);
  digitalWrite(grn_LED,HIGH);
  Serial.println("randomWander()");
  int randNumber = random(millis());    //generate a random number
  int distance = randNumber % 50;       //limit the values between 0 and 50 cm
  int angle = randNumber % 90;          //limit the values between 0 and 90 degrees
   Serial.print(distance);
   Serial.println(" deg\t");
   Serial.print(angle);
   Serial.print(" cm\t");
  //chassis.setWheelSpeeds(robotSpeed,robotSpeed);
  drive(distance,robotSpeed);
  delay(1000);
  idle();
  turn(angle,2*robotSpeed);
  delay(1000);
  idle();
}

//TO DO: Modify the Halt behavior to stop at a given distance when an obstacle is detected.
void Collide(){
  Serial.println("\tCollide");
  digitalWrite(red_LED,HIGH);
  digitalWrite(grn_LED,LOW);
  idle();     //stop the robot
  //TO DO: add code to turn away from obstacle and continue moving
  //turn(90,5*robotSpeed);
  delay(100);
}

//TO DO: Modify the Avoid behavior to move the robot toward or away from an obstacle proportional to error
// only engage the controller if delta error exceeds some threshold
// the robot should slow down the closer it gets to an obstacle
// you have the option in your design of driving the robot away slowly or quickly
// adjust the proportional gain for different reactions
void Avoid(){
  int kp = 5;   //proportional gain
  int dist = 10;  //default move distance = 10 cm
  digitalWrite(red_LED,HIGH);
  digitalWrite(grn_LED,HIGH);
  Serial.print("\tAvoid\terror ");
  Serial.println(error);
  if (abs(error) > 0.5) drive(dist,robotSpeed*(kp*error/wrnDistance));     //move to or from obstacle proportional to error
  //chassis.setWheelSpeeds(robotSpeed*error/wrnDistance,robotSpeed*error/wrnDistance);
  delay(100);
}

void avoidObstacle(){ //avoid Obstacle behavior with collide and avoid primitive behaviors
  if (distance < dgrDistance){
      Collide();  //Robot collide behavior to top before hitting an obstacle.
      readSonar();
   } 
  Avoid();    //Robot runAway behavior to move away proportional to obstacle.
}

void handleMotionComplete(void) // Used to check if the motions above are complete
{
  idle();
}


void setup() // run this once at beginning
{
  Serial.begin(115200);               //begin serial communication for debuggint
  chassis.init();                     //initialize the chassis (which also initializes the motors)
  chassis.setMotorPIDcoeffs(5, 0.5);  //PID controller for driving robot chassis
  rangefinder.init();                 // Call init() to set up the rangefinder
  digitalWrite(red_LED,HIGH);         //test RED LED
  digitalWrite(grn_LED,HIGH);         //test GREEN LED
  delay(5000);                        //insert delay to get robot off test stand before moving on floor
  robotState = ROBOT_WANDER;          //change robot state to random wander
  digitalWrite(red_LED,LOW);         //test RED LED
  digitalWrite(grn_LED,LOW);         //test GREEN LED
  
  //Serial.println("To infinity and beyond but don't hit anything! ");
}

void loop() //run continuously on the microcontroller
{
  readSonar();
  delay(100);                                  //read code and sonar recharge delay

  if (inches < wrnDistance){                    //check for osbtacles 
    error = inches - dgrDistance;              //error = desired distance - actual distance
    robotState = ROBOT_AVOID;                   //change robot state to avoid obstacle
  } else robotState = ROBOT_WANDER;

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_IDLE: 
       if(chassis.checkMotionComplete()) handleMotionComplete(); 
       break;
    case ROBOT_WANDER: 
        randomWander();
        robotState = ROBOT_WANDER;
       break;
    case ROBOT_AVOID: 
        avoidObstacle();
        robotState = ROBOT_WANDER;
       break;
    default:
      break;
  }
}