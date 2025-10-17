#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

motor frontLeft = motor(PORT13, ratio6_1, true); 
motor middleLeft = motor(PORT17, ratio6_1, true);
motor upsideDownLeft = motor(PORT20, ratio6_1, false);

motor frontRight = motor(PORT11, ratio6_1, false); 
motor middleRight = motor(PORT18, ratio6_1, false);
motor upsideDownRight = motor(PORT19, ratio6_1, true);

motor bottomIntake = motor(PORT1, ratio6_1, true);
motor lowerMiddleIntake = motor(PORT10, ratio6_1, true);
motor upperMiddleIntake = motor(PORT9, ratio6_1, true);
motor topIntake = motor(PORT21, ratio6_1, true);

digital_out scraper = digital_out(Brain.ThreeWirePort.A);  //not put

// digital_out descore = digital_out(Brain.ThreeWirePort.A); // Not put 

controller Controller; 

void vexcodeInit( void ) {
  // nothing to initialize
}