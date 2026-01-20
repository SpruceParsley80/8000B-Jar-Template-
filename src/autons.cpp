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

// Forward declarations for helpers used before they’re defined
void intake(int speed);
void scoreHigh(int speed);

/*
//  motor_group intakes = motor_group(bottomIntake, lowerMiddleIntake, upperMiddleIntake);
*/

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

// ------------ Intake anti-jam global state ------------
int  currentIntakeTargetRPM = 0;   // >0 when we want to intake
bool intakeUnjamming        = false;

bool intakeJamEnabled       = false;
bool intakeJamTaskStarted   = false;

// Background task: detects jams on upperMiddleIntake (PORT20)
int intakeAntiJamTask() {
  const float MIN_TARGET_RPM  = 100.0f; // ignore tiny setpoints
  const float JAM_RATIO       = 0.45f;  // <45% of target rpm => suspect jam
  const int   CHECK_PERIOD_MS = 20;     // how often we check
  const int   JAM_TIME_MS     = 180;    // must be jammed this long
  const int   UNJAM_TIME_MS   = 240;    // how long to reverse
  const float UNJAM_VOLT      = 11.0f;  // strong reverse voltage

  int jamTimer = 0;

  while (true) {
    // Only care when auton has enabled this AND we want to intake
    if (!intakeJamEnabled || currentIntakeTargetRPM <= 0) {
      jamTimer = 0;
      task::sleep(CHECK_PERIOD_MS);
      continue;
    }

    // Don’t stack multiple unjams
    if (intakeUnjamming) {
      jamTimer = 0;
      task::sleep(CHECK_PERIOD_MS);
      continue;
    }

    float targetRPM = (float)currentIntakeTargetRPM;
    float actualRPM = upperMiddleIntake.velocity(rpm);

    // Check for "stuck" condition
    if (fabs(targetRPM) > MIN_TARGET_RPM &&
        fabs(actualRPM) < JAM_RATIO * fabs(targetRPM)) {

      jamTimer += CHECK_PERIOD_MS;

      if (jamTimer >= JAM_TIME_MS) {
        // ---- RUN STRONG UNJAM ----
        intakeUnjamming = true;

        // Reverse HARD in outtake direction (opposite of intake()).
        // If this is the wrong direction on your robot,
        // swap forward <-> reverse here.
        bottomIntake.spin(forward, UNJAM_VOLT, volt);
        lowerMiddleIntake.spin(forward, UNJAM_VOLT, volt);
        upperMiddleIntake.spin(forward, UNJAM_VOLT, volt);
        task::sleep(UNJAM_TIME_MS);

        // Resume normal commanded intake in RPM mode
        intake(currentIntakeTargetRPM);

        intakeUnjamming = false;
        jamTimer        = 0;
      }
    } else {
      // RPM looks healthy
      jamTimer = 0;
    }

    task::sleep(CHECK_PERIOD_MS);
  }

  return 0;
}

void startIntakeAntiJam() {
  if (!intakeJamTaskStarted) {
    task t(intakeAntiJamTask);
    intakeJamTaskStarted = true;
  }
  intakeJamEnabled = true;
}

void stopIntakeAntiJam() {
  intakeJamEnabled = false;
}


// DEFAULT SPEEDS ARE 480

// Background task: wait until robot has moved a certain distance,
// then start high-goal scoring.
int scoreHighHalfwayTask() {
  // Record starting pose
  float startX = chassis.get_X_position();
  float startY = chassis.get_Y_position();

  const float triggerDist = 21.0f;  // half of 28"

  while (true) {
    float curX = chassis.get_X_position();
    float curY = chassis.get_Y_position();

    float dx = curX - startX;
    float dy = curY - startY;
    float dist = sqrtf(dx*dx + dy*dy);

    if (dist >= triggerDist) {
      // Start scoring; non-blocking spinFor means this returns quickly.
      scoreHigh(480);
      break;
    }

    task::sleep(10);  // don't hog CPU
  }

  return 0;
}

// ==== Intake helpers used by auton (continuous spin, not spinFor) ====

void intake(int speed) {
  // speed is RPM for auton intake
  currentIntakeTargetRPM = speed;

  if (speed > 0) {
    // Intake direction – keep the same direction you were using before.
    // If triballs go the wrong way, swap reverse <-> forward here.
    bottomIntake.spin(reverse, speed, rpm);
    lowerMiddleIntake.spin(reverse, speed, rpm);
    upperMiddleIntake.spin(reverse, speed, rpm);
  } else {
    // Stop intake
    bottomIntake.stop(brakeType::coast);
    lowerMiddleIntake.stop(brakeType::coast);
    upperMiddleIntake.stop(brakeType::coast);
  }
}

void stopIntake() {
  intake(0);
}

void outtake(int speed) {
  // While outtaking, we consider "intake" target to be 0 so the jam
  // detector doesn't try to fight this.
  currentIntakeTargetRPM = 0;

  // Outtake direction – opposite of intake; swap if it’s wrong.
  bottomIntake.spin(forward, speed, rpm);
  lowerMiddleIntake.spin(forward, speed, rpm);
  upperMiddleIntake.spin(forward, speed, rpm);
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
// distanceOdomCorrect is used to correct Y with back/front walls only,
// so pushback elements near the sides don’t trash X.
//
void right_side() {
  // Use odom-tuned constants for smoother motion
  odom_constants();

  // Global caps (we also pass per-move caps to drive_distance)
  chassis.drive_max_voltage   = 12;
  chassis.heading_max_voltage = 6;

  // --- 0) Initial pose & distance-sensor-based localization ---

  // Rough seed; distance_odom will actually set the real X/Y.
  chassis.set_coordinates(0.0, 0.0, 0.0);

  // Let sensors wake up
  wait(150, msec);

  // At the start you're aligned with real walls, so we can trust all 4 sensors:
  // correct both X and Y using distance sensor odom.
  distanceOdomCorrect(true, true);

  float startX = chassis.get_X_position();
  float startY = chassis.get_Y_position();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("RS X:%.1f Y:%.1f   ", startX, startY);

  // ============================================================
  // Sequence (robot-centric turns, straight drives with heading)
  // ============================================================

  // ---- Step 1: turn 10° robot-centric (tank turn) ----
  turn_relative(11.0f);                      // uses chassis.turn_to_angle() under the hood
  float H1 = chassis.get_absolute_heading(); // new field heading

  // start intaking while we move toward the center balls
  intake(400);

  // ---- Step 2: drive forward 30" holding heading H1 ----
  chassis.drive_distance(30.0f, H1,
                         6.0f,   // drive max V
                         3.0f);  // heading correction max V
  intake(0);

  // small backup to position better before turning
  chassis.drive_distance(-7.0f, H1,
                         8.0f,
                         3.0f);
  
  // ---- Step 3: turn -55° robot-centric (tank turn) ----
  turn_relative(-55.0f);                     // right turn
  float H2 = chassis.get_absolute_heading(); // new field heading

  // ==============================
  //  PUSH-UP / LOW-GOAL SCORING
  // ==============================

  // "Ramming speed" into low goal
  chassis.drive_distance(11.0f, H2,
                         12.0f,  // strong drive
                         7.0f);  // strong heading correction

  wait(250, msec);

  // Deploy scraper and outtake to score
  scraper.set(1);
  wait(250, msec);  
  outtake(480);

  // Nudge a bit further into the goal
  chassis.drive_distance(2.0f, H2,
                         12.0f,
                         6.7f);

  wait(1000, msec);

  // Retract scraper
  scraper.set(0);
  intake(0);

  // Back away from the goal
  chassis.drive_distance(-10.0f, H2,
                         12.0f,
                         6.7f);

  // ---- Step 5: drive backwards 41" holding heading H2 ----
  chassis.drive_distance(-30.0f, H2,
                         8.0f,
                         3.0f);

  // ---- Step 6: turn to 180° field-centric (tank turn) ----
  chassis.turn_to_angle(180.0f);
  float H180 = 180.0f;  // explicit for clarity

  scraper.set(1);
  wait(300, msec);
  intake(430);

  // At this pose you may be nearer a back/side wall again.
  // To avoid pushbacks messing with X, correct Y-only here:
  distanceOdomCorrect(false, true);

  // ---- Step 7: drive forward 15.5" holding 180° with timeout ----
  // distance, heading,  driveV, headingV,  settle_error, settle_time(ms), timeout(ms)
  chassis.drive_distance(15.5f, H180,
                         6.0f,   // drive max voltage
                         3.0f,   // heading max voltage
                         1.5f,   // settle error (inches)
                         300.0f, // settle time (ms)
                         1500.0f // timeout (ms) -> 1.5 seconds
  );

  wait(500, msec);

  // ---- Step 8: back up 28" holding 180° ----
  chassis.drive_distance(-28.0f, H180,
                         10.0f,
                         4.0f);

  scoreHigh(480);
  intake(480);

  // Final Y snap from distance sensors
  distanceOdomCorrect(false, true);

  float endX = chassis.get_X_position();
  float endY = chassis.get_Y_position();

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("End X:%.1f Y:%.1f   ", endX, endY);
}

void left_side() {
  // Use odom-tuned constants for smoother motion
  odom_constants();

  // Global caps (we also pass per-move caps to drive_distance)
  chassis.drive_max_voltage   = 12;
  chassis.heading_max_voltage = 6;

  // --- 0) Initial pose & distance-sensor-based localization ---

  // Rough seed; distance_odom will actually set the real X/Y.
  chassis.set_coordinates(0.0, 0.0, 0.0);

  // Let sensors wake up
  wait(150, msec);

  // At the start you're aligned with real walls, so we can trust all 4 sensors:
  // correct both X and Y using distance sensor odom.
  distanceOdomCorrect(true, true);

  float startX = chassis.get_X_position();
  float startY = chassis.get_Y_position();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("LS X:%.1f Y:%.1f   ", startX, startY);

  // ============================================================
  // Sequence (robot-centric turns, straight drives with heading)
  // Mirrored from right_side
  // ============================================================

  // ---- Step 1: turn -11° robot-centric (mirror of +11°) ----
  turn_relative(-11.0f);                     // tank turn
  float H1 = chassis.get_absolute_heading(); // new field heading

  // start intaking while we move toward the center balls
  intake(400);

  // ---- Step 2: drive forward 30" holding heading H1 ----
  chassis.drive_distance(30.0f, H1,
                         6.0f,   // drive max V
                         3.0f);  // heading correction max V


  // small backup to position better before turning
  chassis.drive_distance(-7.0f, H1,
                         8.0f,
                         3.0f);

  // ---- Step 3: turn -115° robot-centric (your tuned angle) ----
  turn_relative(-120.0f);                    // left-ish turn as you tuned
  float H2 = chassis.get_absolute_heading(); // new field heading

  // ==============================
  //  MID-GOAL SCORING
  //  (uses scoreMid instead of low-goal push-up)
  // ==============================

  // Drive into position for the middle-height goal
  chassis.drive_distance(-17.0f, H2,
                         12.0f,  // strong drive
                         7.0f);  // strong heading correction


  // Fire into the mid-height goal
  scoreMid(480);   // mid goal (scoreMid), as requested
  intake(250);

  // Optional: small settle time to let triballs clear
  wait(1200, msec);

  topIntake.stop(brake);

  // Forward away from the goal
  chassis.drive_distance(43.0f, H2,
                         10.0f,
                         5.0f);



  // ---- Step 6: turn to 180° field-centric (tank turn) ----
  chassis.turn_to_angle(180.0f);
  float H180 = 180.0f;  // explicit for clarity

  scraper.set(1);
  wait(300, msec);
  intake(430);

  // At this pose you may be nearer a back/side wall again.
  // To avoid pushbacks messing with X, correct Y-only here:

  // PUSHBACKS? THE 2025-2026 VEX V5 ROBOTICS COMPETITION?
  distanceOdomCorrect(false, true);

  // ---- Step 7: drive forward 21.5" holding 180° with timeout ----
  // distance, heading,  driveV, headingV,  settle_error, settle_time(ms), timeout(ms)
  chassis.drive_distance(21.0f, H180,
                         6.0f,   // drive max voltage
                         3.0f,   // heading max voltage
                         1.5f,   // settle error (inches)
                         300.0f, // settle time (ms)
                         1500.0f // timeout (ms)
  );

  wait(500, msec);
  
  // ---- Step 8: back up 28" holding 180° ----
  // First half
  chassis.drive_distance(-14.0f, H180,
                         10.0f,
                         4.0f);
  scoreHigh(480);  // high-goal scoring at the end (you had this)
  // Second half
  chassis.drive_distance(-14.0f, H180,
                         10.0f,
                         4.0f);

  // Final Y snap from distance sensors
  distanceOdomCorrect(false, true);

  float endX = chassis.get_X_position();
  float endY = chassis.get_Y_position();

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("End X:%.1f Y:%.1f   ", endX, endY);
}



// The giant commented legacy autons you had before this point are kept as-is
// so you can still reference them, but they don’t affect compilation.


//well so much for that
//
//
/*
//   chassis.turn_timeout = 1000;
//   chassis.drive_timeout = 2000;
//   chassis.drive_max_voltage = 8;
//   chassis.turn_max_voltage = 6;
//   chassis.set_heading(0);
//   ...
//   Controller.Screen.print("sdhgfisdjgbndjgbdgkdbgfjdg");
//   wait(15, msec);
// }

//   chassis.turn_to_angle(6);
//   intake(480);
//   ...
// minecraft golf club?
*/
