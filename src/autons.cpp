#include "vex.h"

/**
* Resets the constants for auton movement.
* Modify these to change the default behavior of functions like
* drive_distance(). For explanations of the difference between
* drive, heading, turning, and swinging, as well as the PID and
* exit conditions, check the docs.
*/

//  motor_group intakes = motor_group(bottomIntake, lowerMiddleIntake, upperMiddleIntake);

//for straight starting position :)
void right_side() {
  chassis.drive_max_voltage = 6;
  chassis.turn_max_voltage = 6;
  chassis.set_heading(0);
  chassis.turn_to_angle(7);
  intake();
  wait(15, msec);
  chassis.drive_distance(16);
  chassis.drive_distance(0);
  chassis.turn_to_angle(7);
  chassis.drive_distance(5);
  chassis.drive_distance(0);
  wait(0.2, sec);
  bottomIntake.stop(brake);
  lowerMiddleIntake.stop(brake);
  upperMiddleIntake.stop(brake);
  chassis.turn_to_angle(145);
  wait(15, msec);
  chassis.drive_distance(-17);
  chassis.drive_distance(0);
  wait(15, msec);
  intake();
  scoreHigh();
  wait(500, msec);
  bottomIntake.stop(coast);
  lowerMiddleIntake.stop(coast);
  upperMiddleIntake.stop(coast);
  topIntake.stop(coast);
  // chassis.drive_distance(43.3);
  // wait(15, msec);
  // chassis.turn_to_angle(180);
  // wait(15, msec);
  // scraper.set(true);
  // wait(15, msec);
  // for (int i = 0; i < 5; i++) {
  //   chassis.drive_distance(1);
  //   wait(15, msec);
  //   chassis.drive_distance(-1);
  //   wait(15, msec);
  // }
  // chassis.drive_distance(20);
  // wait(15, msec);
  // bottomIntake.spin(forward);
  // lowerMiddleIntake.spin(forward);
  // upperMiddleIntake.spin(forward);
  // topIntake.spin(reverse);
  Controller.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
  wait(15, msec);
}
void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  // chassis.set_heading_constants(0, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  // chassis.set_turn_constants(12, .4, 0, 0, 0);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

// /**
//  * The expected behavior is to return to the start position.
//  */

void intake() {
  bottomIntake.spinFor(reverse, 10000, deg, 480, rpm, false); //use spinFor henceforth
  lowerMiddleIntake.spinFor(reverse, 10000, deg, 480, rpm, false);
  upperMiddleIntake.spinFor(reverse, 10000, deg, 480, rpm, false);
 }

 void outtake() {
  bottomIntake.spinFor(reverse, 1000, deg, 480, rpm, false); //use spinFor henceforth
  lowerMiddleIntake.spinFor(reverse, 1000, deg, 480, rpm, false);
  upperMiddleIntake.spinFor(reverse, 1000, deg, 480, rpm, false);
 }
 void scoreHigh() {
  topIntake.spinFor(forward, 1000, deg, 480, rpm, false);
 }
 void scoreMid() {
  topIntake.spinFor(reverse, 1000, deg, 480, rpm, false);
 }

void drive_test(){
  // chassis.drive_distance(24);
  // chassis.drive_distance(12);
  // chassis.drive_distance(18);
  // chassis.drive_distance(-36);
  chassis.turn_to_angle(5);
  intake();
  chassis.set_heading(0);
  wait(15, msec);
  chassis.drive_distance(24);
  wait(1, sec);
  bottomIntake.stop(coast);
  lowerMiddleIntake.stop(coast);
  upperMiddleIntake.stop(coast);
  // chassis.turn_to_angle(128);
  // wait(15, msec);
  // chassis.drive_distance(-2);
  // wait(15, msec);
  // bottomIntake.spin(forward);
  // lowerMiddleIntake.spin(forward);
  // upperMiddleIntake.spin(forward);
  // topIntake.spin(reverse);
  // wait(15, msec);
  // bottomIntake.stop(coast);
  // lowerMiddleIntake.stop(coast);
  // upperMiddleIntake.stop(coast);
  // topIntake.stop(coast);
  // chassis.drive_distance(43.3);
  // wait(15, msec);
  // chassis.turn_to_angle(180);
  // wait(15, msec);
  // scraper.set(true);
  // wait(15, msec);
  // for (int i = 0; i < 5; i++) {
  //   chassis.drive_distance(1);
  //   wait(15, msec);
  //   chassis.drive_distance(-1);
  //   wait(15, msec);
  // }
  // chassis.drive_distance(20);
  // wait(15, msec);
  // bottomIntake.spin(forward);
  // lowerMiddleIntake.spin(forward);
  // upperMiddleIntake.spin(forward);
  // topIntake.spin(reverse);
  Brain.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
  wait(15, msec);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.set_heading(0);
  // chassis.turn_to_angle(5);
  // chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  // chassis.turn_to_angle(225);
  // chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void test_auton() {
  odom_constants();
  chassis.set_heading(0);

}
//all intakes fwd
// minecraft golf club?