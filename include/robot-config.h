using namespace vex;

extern brain Brain;

extern motor frontLeft; 
extern motor middleLeft;
extern motor upsideDownLeft;



extern inertial inert;
extern motor frontRight; 
extern motor middleRight;
extern motor upsideDownRight;

extern motor bottomIntake;
extern motor lowerMiddleIntake;
extern motor upperMiddleIntake;
extern motor topIntake;
// extern motor_group intakes;

extern digital_out scraper;
extern digital_out descore;
// extern digital_out descore; 

extern vex::distance frontDist;
extern vex::distance backDist;
extern vex::distance leftDist;
extern vex::distance rightDist;

extern controller Controller; 

void  vexcodeInit( void );