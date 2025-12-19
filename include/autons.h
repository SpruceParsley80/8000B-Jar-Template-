#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;
// extern motor_group intakes;
//constants
void default_constants();
void odom_constants();
//actual game autons
void right_odom();
void left_odom();
void right_side();
void left_side();

//helper functions
void intake(int);
void outtake(int);
void scoreHigh(int);
void scoreMid(int);
void intakeForScoring(int);
void jonathanSpecialMarkOne(int);
void upperUnclogger(int);

//test autons
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
void test_auton();