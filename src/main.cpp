#include "vex.h"

using namespace vex;
competition Competition;
Odom odom;
//big thanks to ben from my sister team for helping me change from pros to jar
//i couldve done it myself probably but it would've been harder and we didnt have much time so

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
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
// TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
// ZERO_TRACKER_NO_ODOM
,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(frontLeft, middleLeft, upsideDownLeft),

//Right Motors:
motor_group(frontRight, middleRight, upsideDownRight),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT12,


//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT16,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-1.5,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT21,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
-3.5

);

int current_auton_selection = 0;
bool auto_started = false;

//important costants

/*AI USE DISCLAIMER */
//the next few lines of code were vibecoded but i didn't copy and paste anything 
//so its marginally better than just copy and pasting wholecloth
const float FORWARD_TRACKER_CENTER_DISTANCE = 0.0;
const float SIDE_TRACKER_CENTER_DISTANCE = 0.0;
const float TICKS_PER_REV = 360.0;
const float WHEEL_DIAM = 3.25;
//you wanted precision i'll give you precision
const float PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679; 


/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  odom.set_physical_distances(FORWARD_TRACKER_CENTER_DISTANCE, SIDE_TRACKER_CENTER_DISTANCE);

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "what are we doing guys");
    // switch(current_auton_selection){
    //   case 0:
    //     Brain.Screen.printAt(5, 140, "Auton 1");
    //     break;
    //   case 1:
    //     Brain.Screen.printAt(5, 140, "Auton 2");
    //     break;
    //   case 2:
    //     Brain.Screen.printAt(5, 140, "Auton 3");
    //     break;
    //   case 3:
    //     Brain.Screen.printAt(5, 140, "Auton 4");
    //     break;
    //   case 4:
    //     Brain.Screen.printAt(5, 140, "Auton 5");
    //     break;
    //   case 5:
    //     Brain.Screen.printAt(5, 140, "Auton 6");
    //     break;
    //   case 6:
    //     Brain.Screen.printAt(5, 140, "Auton 7");
    //     break;
    //   case 7:
    //     Brain.Screen.printAt(5, 140, "Auton 8");
    //     break;
    // }
    // if(Brain.Screen.pressing()){
    //   while(Brain.Screen.pressing()) {}
    //   current_auton_selection ++;
    // } else if (current_auton_selection == 8){
    //   current_auton_selection = 0;
    // }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */
//  void intake() {
//   bottomIntake.spinFor(forward, 1000, deg, 480, rpm, false); //use spinFor henceforth
//   lowerMiddleIntake.spinFor(forward, 1000, deg, 480, rpm, false);
//   upperMiddleIntake.spinFor(forward, 1000, deg, 480, rpm, false);
//  }

//  void outtake() {
//   bottomIntake.spinFor(reverse, 1000, deg, 480, rpm, false); //use spinFor henceforth
//   lowerMiddleIntake.spinFor(reverse, 1000, deg, 480, rpm, false);
//   upperMiddleIntake.spinFor(reverse, 1000, deg, 480, rpm, false);
//  }
//  void scoreHigh() {
//   topIntake.spinFor(reverse, 1000, deg, 480, rpm, false);
//  }
//  void scoreMid() {
//   topIntake.spinFor(forward, 1000, deg, 480, rpm, false);
//  }

void autonomous(void) {
  auto_started = true;
  //V UNCOMMENT THIS LINE IF DOWNLOADING TO SLOT 2 V
  // right_side();
  //V UNCOMMENT THIS LINE IF DOWNLOADING TO SLOT 1 V
  left_side();
  Brain.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
  wait(15, msec);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  scraper = true; 
  bool buttonA = true; 
  bool buttonB = false;
  bool buttonX = false;
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
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  inert.calibrate();
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
