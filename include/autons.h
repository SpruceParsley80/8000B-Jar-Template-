#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;
// extern motor_group intakes;
void default_constants();

//actual game autons
void right_side();

//helper functions
void intake();
void outtake();
void scoreHigh();
void scoreMid();

//test autons
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
void test_auton();