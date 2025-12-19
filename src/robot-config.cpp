#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

motor frontLeft       = motor(PORT13, ratio6_1, true); 
motor middleLeft      = motor(PORT16, ratio6_1, true);
motor upsideDownLeft  = motor(PORT19, ratio6_1, false);

motor frontRight      = motor(PORT11, ratio6_1, false); 
motor middleRight     = motor(PORT15, ratio6_1, false);
motor upsideDownRight = motor(PORT18, ratio6_1, true);

motor bottomIntake        = motor(PORT1, ratio6_1, true);
motor lowerMiddleIntake   = motor(PORT9, ratio6_1, true);
motor upperMiddleIntake   = motor(PORT20, ratio6_1, true);
motor topIntake           = motor(PORT8, ratio6_1, true);

// ---- Distance sensors: define each ONCE, with your correct ports ----
distance frontDist = distance(PORT3);   // front-facing sensor
distance backDist  = distance(PORT17);  // back-facing sensor
distance leftDist  = distance(PORT2);   // left side, facing left wall
distance rightDist = distance(PORT4);   // right side, facing right wall
// ---------------------------------------------------------------------

digital_out scraper = digital_out(Brain.ThreeWirePort.H);  
digital_out descore = digital_out(Brain.ThreeWirePort.G);  

controller Controller; 
inertial inert(PORT12); 

void vexcodeInit( void ) {
  inert.calibrate();
}
