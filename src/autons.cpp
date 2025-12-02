#include "vex.h"
#include "JAR-Template/distance_odom.h"

using namespace vex;

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

// This helper is implemented in util.cpp
void turn_relative(float delta_deg);

//  motor_group intakes = motor_group(bottomIntake, lowerMiddleIntake, upperMiddleIntake);

// Simple odom demo (you can keep or ignore)
void right_odom() {
  odom_constants();
}

/**
 * Default PID and exit conditions.
 */
void default_constants() {
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(12, 1.45, 0.001, 17, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  // chassis.set_heading_constants(0, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .005, 3, 15);
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
void odom_constants() {
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage   = 8;
  chassis.drive_settle_error  = 3;
  chassis.boomerang_lead      = 0.5;
  chassis.drive_min_voltage   = 0;
}

// ==========================
// Intake helper functions
// ==========================

// DEFAULT SPEEDS ARE 480
void intake(int speed) {
  bottomIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
}

void intakeForScoring(int speed) {
  // bottomIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
}

void jonathanSpecialMarkOne(int speed) { // intake but only bottom 2 rollers
  bottomIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  // upperMiddleIntake left off
}

void upperUnclogger(int speed) {
  topIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed / 2, rpm, false);
}

void outtake(int speed) { // 480 is default
  bottomIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
  lowerMiddleIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
}

void scoreHigh(int speed) {
  topIntake.spinFor(forward, 10000, deg, speed, rpm, false);
}

void scoreMid(int speed) {
  topIntake.spinFor(forward, -10000, deg, speed, rpm, false);
}

// ==========================
// Simple test auton (used by ButtonUp -> drive_test())
// ==========================

void drive_test() {
  // Very simple forward/back test so ButtonUp has something safe to run.
  odom_constants();

  chassis.drive_max_voltage   = 6;
  chassis.heading_max_voltage = 6;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("drive_test: F12 B12");

  // Drive forward 12", then back 12"
  chassis.drive_distance(12.0f);
  chassis.drive_distance(-12.0f);

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("drive_test done");
}

// ==========================
// Actual right-side auton
// ==========================
//
// Sequence (robot-centric unless otherwise noted):
//  1) turn 7 degrees
//  2) drive forward 28 inches
//  3) turn 90 degrees
//  4) drive forward 14 inches
//  5) drive backwards 51 inches
//  6) turn to 180 degrees FIELD-CENTRIC
//  7) drive forward 5 inches
//  8) back up 30 inches
//
// distanceOdomCorrect is used to correct Y with back/front walls only,
// so pushback elements near the sides don’t trash X.

void right_side() {
  // Use odom-tuned constants for smoother motion
  odom_constants();

  // Slightly conservative voltages for auton safety
  chassis.drive_max_voltage   = 8;
  chassis.heading_max_voltage = 6;

  // --- 0) Initial pose + distance-sensor correction on Y only ---
  // Place the robot in your right-side blue starting spot,
  // roughly facing upfield (0°).
  chassis.set_coordinates(0.0, 0.0, 0.0);

  // Use distance sensors ONLY to correct Y (front/back).
  distanceOdomCorrect(false, true);

  float startX = chassis.get_X_position();
  float startY = chassis.get_Y_position();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("RS X:%.1f Y:%.1f   ", startX, startY);

  // ============================================================
  // Sequence you asked for (robot-centric unless noted)
  // ============================================================

  // 1) Turn 7 degrees (robot-centric)
  turn_relative(7.0f);

  // 2) Drive forward 28 inches
  chassis.drive_distance(28.0f);

  // Optional: a light Y correction could go here if desired:
  // distanceOdomCorrect(false, true);

  // 3) Turn 90 degrees (robot-centric)
  turn_relative(90.0f);

  // 4) Drive forward 14 inches
  chassis.drive_distance(14.0f);

  // 5) Drive backwards 51 inches
  chassis.drive_distance(-51.0f);

  // 6) Turn to 180 degrees FIELD-CENTRIC (absolute)
  chassis.turn_to_angle(180.0f);

  // At 180°, front/back sensors now see opposite walls.
  // Again, correct ONLY Y to avoid pushback confusion on the sides.
  distanceOdomCorrect(false, true);

  // 7) Drive forward 5 inches
  chassis.drive_distance(5.0f);

  // 8) Back up 30 inches
  chassis.drive_distance(-30.0f);

  // Final Y correction + print where we ended
  distanceOdomCorrect(false, true);
  float endX = chassis.get_X_position();
  float endY = chassis.get_Y_position();

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("End X:%.1f Y:%.1f   ", endX, endY);
}

//   chassis.turn_timeout = 1000;
//   chassis.drive_timeout = 2000;
//   chassis.drive_max_voltage = 8;
//   chassis.turn_max_voltage = 6;
//   chassis.set_heading(0);
//   wait(5, msec);
//   intake(430);
//   chassis.drive_distance(33, 0, 7, 2);
//   intake(0);
//   chassis.drive_distance(-10);
//   chassis.turn_to_angle(-80);
  
//   /* THIS IS THE SENSIBLE TECH*/
//   // chassis.drive_distance(-3);
//   // wait(750, msec);
//   // chassis.drive_with_voltage(12, 12);
//   // chassis.drive_distance(5.25);
  
//   /* THIS IS THE PUSHUP TECH*/
//   //RAMNINGNGNGGNGN SPEEDDDDDD
//   chassis.drive_distance(11, -80, 12, 7);
//   wait(250, msec);
//   scraper.set(1);
//   outtake(480);
//   chassis.drive_distance(2, -80, 12, 6.7);
//   wait(1000, msec);
//   scraper.set(0);
//   chassis.drive_distance(-10, -90, 12, 6.7);
//   // wait(100, msec);
//   chassis.drive_distance(-34, -90, 8, 6.7);
//   chassis.turn_to_angle(141);
//   scraper.set(1);
//   wait(230, msec);
//   intake(480);
//   chassis.drive_with_voltage(6, 6);
//   chassis.drive_distance(17.67, 141, 6, 4);
//   wait(500, msec);
//   for (int i = 0; i < 3; i++) {
//     chassis.drive_distance(1.1, -141, 12, 12);
//     chassis.drive_distance(-1.1, -141, 12, 12);
//   }
//   // chassis.drive_distance(-4);
//   // chassis.drive_distance(0);
//   wait(5, msec);
//   chassis.drive_distance(-20, 142, 12, 3);
//   scoreHigh(480);
//   chassis.drive_distance(-6, 142, 12, 3);
// //   chassis.turn_to_angle(6);
// //   intake(480);
// //   wait(5, msec);
// //   chassis.drive_distance(19.5);
// //   wait(200, msec);
// //   bottomIntake.stop(brake);
// //   lowerMiddleIntake.stop(brake);
// //   upperMiddleIntake.stop(brake);
// //   chassis.drive_distance(0);
// //   chassis.turn_to_angle(7);
// //   chassis.drive_distance(6.7);
// //   chassis.drive_distance(0);
// //   wait(5, msec);
// //   chassis.turn_to_angle(-46.5);
// //   wait(5, msec);
// //   chassis.drive_distance(12.5);
// //   wait(25, msec);
// //   outtake(200);
// //   chassis.drive_distance(0);
// //   scraper.set(true);
// //   wait(5, msec);
// //   outtake(200);
// //   scoreMid(200);
// //   wait(925, msec);
// //   topIntake.stop(coast);
// //   // // scoreMid(520);
// //   wait(300, msec);
// //   bottomIntake.stop(coast);
// //   lowerMiddleIntake.stop(coast);
// //   upperMiddleIntake.stop(coast);
// //   scraper.set(false);
// //   // wait(1350, msec);
// //   // topIntake.stop(coast);
// //   // scoreMid(520);
// //   topIntake.stop(coast);
// //   chassis.drive_max_voltage = 12;
// //   chassis.turn_to_angle(-50.5);
// //   chassis.drive_distance(-37);
// //   wait(5, msec);
// //   chassis.drive_max_voltage = 8;
// //   chassis.turn_to_angle(180);
// //   wait(5, msec);
// //   scraper.set(true);
// //   wait(230, msec);
// //   intake(200);
// //   chassis.drive_distance(14);
// //   chassis.drive_distance(0);
// //   wait(5, msec);
// //   chassis.drive_distance(-4);
// //   chassis.drive_distance(0);
// //   wait(5, msec);
// //   chassis.drive_distance(-30);
// //   scoreHigh(200);
// //   // 
// //   Controller.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
// //   wait(15, msec);
// void left_side() {
//   chassis.turn_timeout = 1000;
//   chassis.drive_timeout = 1500;
//   chassis.drive_max_voltage = 6;
//   chassis.turn_max_voltage = 6;
//   chassis.set_heading(0);
//   wait(5, msec);
//   chassis.turn_to_angle(-7);
//   jonathanSpecialMarkOne(480);
//   chassis.drive_distance(19);
//   // chassis.drive_distance(0);
//   chassis.turn_to_angle(-7);
//   chassis.drive_distance(5.2);
//   wait(5, msec);
//   bottomIntake.stop(brake);
//   lowerMiddleIntake.stop(brake);
//   upperMiddleIntake.stop(brake);
//   //this part gets changed
//   chassis.turn_to_angle(-135);
//   wait(5, msec);  
//   chassis.drive_distance(-17);
//   // chassis.drive_distance(0);
//   wait(5, msec);
//   outtake(200);
//   wait(1, msec);
//   intakeForScoring(200);
//   scoreHigh(200);
//   // wait(1350, msec);
//   // topIntake.stop(coast);
//   // scoreMid(520);
//   wait(1650, msec);
//   bottomIntake.stop(coast);
//   lowerMiddleIntake.stop(coast);
//   upperMiddleIntake.stop(coast);
//   topIntake.stop(coast);
//   // chassis.turn_max_voltage = 12;
//   // chassis.turn_to_angle(45);
//   // topIntake.stop(coast);
//   chassis.drive_max_voltage = 9;
//   scoreHigh(20);
//   outtake(20);
  
//   chassis.drive_distance(44);
//   topIntake.stop(coast);
//   wait(5, msec);
//   bottomIntake.stop(coast);
//   lowerMiddleIntake.stop(coast);
//   upperMiddleIntake.stop(coast);
//   chassis.drive_max_voltage = 5;
//   chassis.drive_distance(0);
//   chassis.turn_to_angle(180);
//   wait(5, msec);
//   scraper.set(true);
//   wait(200, msec);
//   jonathanSpecialMarkOne(200);
//   scoreHigh(25);
//   chassis.drive_distance(13);
//   // chassis.drive_max_voltage = 12;
//   // chassis.drive_timeout = 100;
//   // chassis.drive_distance(-5);
//   // // chassis.drive_distance(0);
//   // chassis.drive_distance(5);
//   // chassis.drive_distance(0);
//   chassis.drive_timeout = 1250;
//   chassis.drive_max_voltage = 12;
//   // wait(2, msec);
//   chassis.drive_distance(-5);
//   wait(2, msec);
//   chassis.turn_to_angle(180);
//   // outtake(30);
//   upperUnclogger(200);
//   chassis.drive_distance(-30);
//   topIntake.stop(coast);
//   upperMiddleIntake.stop(coast);
//   lowerMiddleIntake.stop(coast);
//   wait(210, msec);
//   intakeForScoring(200);
//   scoreHigh(200);
//   // 
//   Controller.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
//   wait(15, msec);
// }

//   chassis.turn_to_angle(6);
//   intake(480);
//   wait(5, msec);
//   chassis.drive_distance(19.5);
//   wait(200, msec);
//   bottomIntake.stop(brake);
//   lowerMiddleIntake.stop(brake);
//   upperMiddleIntake.stop(brake);
//   chassis.drive_distance(0);
//   chassis.turn_to_angle(7);
//   chassis.drive_distance(6.7);
//   chassis.drive_distance(0);
//   wait(5, msec);
//   chassis.turn_to_angle(-46.5);
//   wait(5, msec);
//   chassis.drive_distance(12.5);
//   wait(25, msec);
//   outtake(200);
//   chassis.drive_distance(0);
//   scraper.set(true);
//   wait(5, msec);
//   outtake(200);
//   scoreMid(200);
//   wait(925, msec);
//   topIntake.stop(coast);
//   // // scoreMid(520);
//   wait(300, msec);
//   bottomIntake.stop(coast);
//   lowerMiddleIntake.stop(coast);
//   upperMiddleIntake.stop(coast);
//   scraper.set(false);
//   // wait(1350, msec);
//   // topIntake.stop(coast);
//   // scoreMid(520);
//   topIntake.stop(coast);
//   chassis.drive_max_voltage = 12;
//   chassis.turn_to_angle(-50.5);
//   chassis.drive_distance(-37);
//   wait(5, msec);
//   chassis.drive_max_voltage = 8;
//   chassis.turn_to_angle(180);
//   wait(5, msec);
//   scraper.set(true);
//   wait(230, msec);
//   intake(200);
//   chassis.drive_distance(14);
//   chassis.drive_distance(0);
//   wait(5, msec);
//   chassis.drive_distance(-4);
//   chassis.drive_distance(0);
//   wait(5, msec);
//   chassis.drive_distance(-30);
//   scoreHigh(200);
//   // 
//   Controller.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
//   wait(15, msec);
//all intakes fwd
// minecraft golf club?