#include "vex.h"

using namespace vex;
competition Competition;

// forward declaration of test function implemented in autons.cpp
void drive_test();

//big thanks to ben from my sister team for helping me change from pros to jar
//i couldve done it myself probably but it would've been harder and we didnt have much time so

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
// ZERO_TRACKER_NO_ODOM
,

//Left Motors:
motor_group(frontLeft, middleLeft, upsideDownLeft),

//Right Motors:
motor_group(frontRight, middleRight, upsideDownRight),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT12,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
0.6,

//Gyro scale
360,

/*---------------------------------------------------------------------------*/
/*            The rest is for POSITION TRACKING (we are using it).           */
/*---------------------------------------------------------------------------*/

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port.
PORT16,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Forward Tracker center distance (positive = tracker on right side of robot):
7.5,

//Sideways Tracker Port (ignored for ZERO_TRACKER_ODOM, but kept for completeness):
PORT21,

//Sideways tracker diameter:
-2.75,

//Sideways tracker center distance:
-3.5

);

int current_auton_selection = 0;
bool auto_started = false;

//important constants
const float FORWARD_TRACKER_CENTER_DISTANCE = 0.0;
const float SIDE_TRACKER_CENTER_DISTANCE = 0.0;
const float TICKS_PER_REV = 360.0;
const float WHEEL_DIAM = 3.25;
const float PI = 3.14159265358979323846f;

/**
 * pre_auton
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  // (you've commented out the auton selector UI, leaving it is fine)
}

/**
 * autonomous
 */
extern void startIntakeAntiJam();
extern void stopIntakeAntiJam();

void autonomous(void) {
  auto_started = true;
 startIntakeAntiJam();
  // Currently using your existing auton:
  //right_side();
  // If you want to run a specific test instead, call it here instead of right_side().
  left_side();
stopIntakeAntiJam();
  wait(15, msec);
}

/**
 * usercontrol
 */

void usercontrol(void) {
  // User control code here, inside the loop
  scraper = false; 
  bool buttonA = false; 
  bool buttonB = false;
  bool buttonX = false;
  bool upWasPressed = false;
  bool yWasPressed = false;

  // Initial odom guess for driver; will get corrected when you hit Y
  chassis.set_coordinates(8.0, 8.0, 0.0);

  while (1) {
    int R1L1Speed = (Controller.ButtonL1.pressing() * (-1) + Controller.ButtonR1.pressing()) * 127; 
    int R2L2Speed = (Controller.ButtonL2.pressing() * (-1) + Controller.ButtonR2.pressing()) * 127; 

    if (Controller.ButtonA.pressing()) {
      if(buttonA){
        scraper = !scraper;
        buttonA = false; 
      }
    } else{
      buttonA = true; 
    }

    if (Controller.ButtonX.pressing()) {
      if(buttonX){
        descore = !descore;
        buttonX = false; 
      }
    } else{
      buttonX = true; 
    }

    if (Controller.ButtonB.pressing()) {
      if(buttonB){
        R1L1Speed *= 0;
        buttonB = !buttonB; 
      }
    }

    bottomIntake.spin(directionType::fwd, R1L1Speed, vex::velocityUnits::pct);
    lowerMiddleIntake.spin(directionType::fwd, R1L1Speed, vex::velocityUnits::pct);
    upperMiddleIntake.spin(directionType::fwd, R1L1Speed, vex::velocityUnits::pct);
    topIntake.spin(directionType::rev, R2L2Speed, vex::velocityUnits::pct);
    
    chassis.control_arcade();

    // ---- Y button: distance-odom correction + debug ----
    bool yNow = Controller.ButtonY.pressing();
    if (yNow && !yWasPressed) {
      float oldX = chassis.get_X_position();
      float oldY = chassis.get_Y_position();

      float dFront = frontDist.isObjectDetected() ? frontDist.objectDistance(inches) : -1.0f;
      float dBack  = backDist.isObjectDetected()  ? backDist.objectDistance(inches)  : -1.0f;
      float dLeft  = leftDist.isObjectDetected()  ? leftDist.objectDistance(inches)  : -1.0f;
      float dRight = rightDist.isObjectDetected() ? rightDist.objectDistance(inches) : -1.0f;

      bool changed = distanceOdomCorrect(true, true);

      float newX = chassis.get_X_position();
      float newY = chassis.get_Y_position();

      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Corr:%d", changed ? 1 : 0);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("OldX:%.1f OldY:%.1f   ", oldX, oldY);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("NewX:%.1f NewY:%.1f   ", newX, newY);
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("F:%.1f B:%.1f   ", dFront, dBack);
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("L:%.1f R:%.1f   ", dLeft, dRight);

      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("C:%d", changed ? 1 : 0);
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("B:%.1f   ", dBack);
      Controller.Screen.setCursor(3, 1);
      Controller.Screen.print("OX:%.1f OY:%.1f", oldX, oldY);
      Controller.Screen.setCursor(4, 1);
      Controller.Screen.print("NX:%.1f NY:%.1f", newX, newY);
    }
    yWasPressed = yNow;

    // ---- Up button: run drive_test() (your wall test or whatever) ----
    bool upNow = Controller.ButtonUp.pressing();
    if (upNow && !upWasPressed) {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("drive_test()");
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Running drive_test");

      drive_test();
    }
    upWasPressed = upNow;

    // ---- Always show current pose on Brain (optional) ----
    float x = chassis.get_X_position();
    float y = chassis.get_Y_position();
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("X: %.1f  Y: %.1f   ", x, y);

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  inert.calibrate();
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
