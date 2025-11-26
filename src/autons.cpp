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

//dig thought the autons
// Swing to face a target point, then drive to it and correct with distance odom.
void go_to_point_with_swing(float targetX, float targetY) {
  // Compute desired absolute heading to the target.
  // 0 deg = +Y direction, matching JAR-Template conventions.
  float currentX = chassis.get_X_position();
  float currentY = chassis.get_Y_position();

  float angle_deg = to_deg(atan2(targetX - currentX, targetY - currentY));
  float current_heading = chassis.get_absolute_heading();
  float delta = reduce_negative_180_to_180(angle_deg - current_heading);

  // Choose swing direction based on sign of delta
  if (delta >= 0) {
    // Turn CCW-ish using left swing
    chassis.left_swing_to_angle(angle_deg);
  } else {
    // Turn CW-ish using right swing
    chassis.right_swing_to_angle(angle_deg);
  }

  // Drive to the point using odom
  chassis.drive_to_point(targetX, targetY);

  // Snap pose back onto walls with distance sensors
  distanceOdomCorrect(true, true);

  // Small pause to let everything settle
  wait(200, msec);
}

void right_odom() {
  odom_constants();










}



void default_constants(){
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
//DEFAULT SPEEDS ARE 480
void intake(int speed) {
  bottomIntake.spinFor(reverse, 10000, deg, speed, rpm, false); //use spinFor henceforth
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
 }
 void intakeForScoring(int speed) {
  // bottomIntake.spinFor(reverse, 10000, deg, speed, rpm, false); //use spinFor henceforth
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
 }
 void jonathanSpecialMarkOne(int speed) { // intakes but only with the bottom 2 intakes and doesnt score
  bottomIntake.spinFor(reverse, 10000, deg, speed, rpm, false); //use spinFor henceforth
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
  // upperMiddleIntake.spinFor(reverse, 10000, deg, speed, rpm, false);
 }
 void upperUnclogger(int speed) {
  topIntake.spinFor(reverse, -10000, deg, speed, rpm, false); //use spinFor henceforth
  upperMiddleIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
  lowerMiddleIntake.spinFor(reverse, 10000, deg, speed / 2, rpm, false);
 }

 void outtake(int speed) { // 480 is default
  bottomIntake.spinFor(reverse, -10000, deg, speed, rpm, false); //use spinFor henceforth
  lowerMiddleIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
  upperMiddleIntake.spinFor(reverse, -10000, deg, speed, rpm, false);
 }
 void scoreHigh(int speed) {
  topIntake.spinFor(forward, 10000, deg, speed, rpm, false);
 }
 void scoreMid(int speed) {
  topIntake.spinFor(forward, -10000, deg, speed, rpm, false);
 }

void drive_test(){
// Use odom-tuned drive constants
  odom_constants();

  // Slow it down so it’s easy to watch and stays well controlled
  chassis.drive_max_voltage   = 4;
  chassis.heading_max_voltage = 4;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Hall Points Test");

  // --- 0) Initial pose: center of first tile, facing forward ---
  // Physically place the robot:
  //   - Centered between hallway walls
  //   - In the middle of the FIRST tile (closest to back wall)
  //   - Facing down the hallway (+Y)
  chassis.set_coordinates(21.0, 12.0, 0.0);   // rough guess

  // Correct with distance sensors
  distanceOdomCorrect(true, true);
  wait(200, msec);

  float startX = chassis.get_X_position();
  float startY = chassis.get_Y_position();

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Start X:%.1f Y:%.1f   ", startX, startY);

  // We’ll keep all points near the center to stay inside the 1×2 tile area.
  // startY ~ 12, so Y up to ~28 is safely within 2 tiles.

  // --- Point 1: straight forward into tile 2 (center line) ---
  float p1X = startX;           // stay on same X (center)
  float p1Y = startY + 8.0f;    // move ~8" forward

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("P1  X:%.1f Y:%.1f   ", p1X, p1Y);

  go_to_point_with_swing(p1X, p1Y);

  // --- Point 2: diagonally left-forward inside tile 2 ---
  float p2X = startX - 6.0f;    // 6" toward the left, still away from wall
  float p2Y = startY + 14.0f;   // a bit further down the hallway

  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("P2  X:%.1f Y:%.1f   ", p2X, p2Y);

  go_to_point_with_swing(p2X, p2Y);

  // --- Point 3: diagonally right-forward, mirrored on the right side ---
  float p3X = startX + 6.0f;    // 6" toward the right, still away from wall
  float p3Y = startY + 14.0f;   // same Y as P2

  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("P3  X:%.1f Y:%.1f   ", p3X, p3Y);

  go_to_point_with_swing(p3X, p3Y);

  // --- Point 4: return back near the starting point ---
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("Back to start");

  go_to_point_with_swing(startX, startY);

  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("End X:%.1f Y:%.1f   ", chassis.get_X_position(), chassis.get_Y_position());
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
//old ahh autons moved down for convenience
void right_side() {
  chassis.turn_timeout = 1000;
  chassis.drive_timeout = 2000;
  chassis.drive_max_voltage = 8;
  chassis.turn_max_voltage = 6;
  chassis.set_heading(0);
  wait(5, msec);
  intake(430);
  chassis.drive_distance(33, 0, 7, 2);
  intake(0);
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-80);
  
  /* THIS IS THE SENSIBLE TECH*/
  // chassis.drive_distance(-3);
  // wait(750, msec);
  // chassis.drive_with_voltage(12, 12);
  // chassis.drive_distance(5.25);
  
  /* THIS IS THE PUSHUP TECH*/
  //RAMNINGNGNGGNGN SPEEDDDDDD
  chassis.drive_distance(11, -80, 12, 7);
  wait(250, msec);
  scraper.set(1);
  outtake(480);
  chassis.drive_distance(2, -80, 12, 6.7);
  wait(1000, msec);
  scraper.set(0);
  chassis.drive_distance(-10, -90, 12, 6.7);
  // wait(100, msec);
  chassis.drive_distance(-34, -90, 8, 6.7);
  chassis.turn_to_angle(141);
  scraper.set(1);
  wait(230, msec);
  intake(480);
  chassis.drive_with_voltage(6, 6);
  chassis.drive_distance(17.67, 141, 6, 4);
  wait(500, msec);
  for (int i = 0; i < 3; i++) {
    chassis.drive_distance(1.1, -141, 12, 12);
    chassis.drive_distance(-1.1, -141, 12, 12);
  }
  // chassis.drive_distance(-4);
  // chassis.drive_distance(0);
  wait(5, msec);
  chassis.drive_distance(-20, 142, 12, 3);
  scoreHigh(480);
  chassis.drive_distance(-6, 142, 12, 3);
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
 }
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
void left_side() {
  chassis.turn_timeout = 1000;
  chassis.drive_timeout = 2000;
  chassis.drive_max_voltage = 8;
  chassis.turn_max_voltage = 6;
  chassis.set_heading(0);
  wait(5, msec);
  intake(430);
  chassis.drive_distance(33, 0, 7, 2);
  intake(0);
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-90);
  
  /* THIS IS THE SENSIBLE TECH*/
  // chassis.drive_distance(-3);
  // wait(750, msec);
  // chassis.drive_with_voltage(12, 12);
  // chassis.drive_distance(5.25);

  chassis.drive_distance(-14, -90, 12, 7);
  // wait(250, msec);
  intake(480);
  scoreHigh(480);
  wait(550, msec);
  // chassis.drive_distance(2, -90, 12, 6.7);
  upperMiddleIntake.stop(brake);
  topIntake.stop(brake);
  chassis.drive_distance(12, -90, 12, 6.7);
  // wait(100, msec);
  chassis.drive_distance(28, -90, 8, 6.7);
  chassis.turn_to_angle(-141);
  scraper.set(1);
  wait(230, msec);
  topIntake.stop(brake);
  intake(480);
  chassis.drive_with_voltage(6, 6);
  wait(750, msec);
  // chassis.drive_distance(15.67, -141, 12, 4);
  for (int i = 0; i < 3; i++) {
    chassis.drive_distance(1.1, -141, 12, 12);
    chassis.drive_distance(-1.1, -141, 12, 12);
  }
  chassis.drive_distance(-4);
  chassis.drive_distance(0);
  wait(5, msec);
  outtake(120);
  chassis.drive_distance(-20, -142, 12, 3);
  intake(480);
  scoreHigh(480);
  chassis.drive_distance(-6, -142, 12, 3);
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
}
//all intakes fwd
// minecraft golf club?