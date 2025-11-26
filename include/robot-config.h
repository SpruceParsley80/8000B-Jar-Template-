#include <iterator>
using namespace vex;

//brain
extern brain Brain;

//chassis
extern motor frontLeft; 
extern motor middleLeft;
extern motor upsideDownLeft;

extern motor frontRight; 
extern motor middleRight;
extern motor upsideDownRight;

//sensors
extern inertial inert;
extern distance rightDist;
extern distance frontDist;

//intake
extern motor bottomIntake;
extern motor lowerMiddleIntake;
extern motor upperMiddleIntake;
extern motor topIntake;

//pneumatics
extern digital_out scraper;
extern digital_out descore;
// extern digital_out descore; 

extern vex::distance frontDist;
extern vex::distance backDist;
extern vex::distance leftDist;
extern vex::distance rightDist;

extern controller Controller; 

void  vexcodeInit( void );