#include "vex.h"
#include "JAR-Template/distance_odom.h"

using namespace vex;

// From your project (already defined elsewhere)
extern Drive chassis;
extern Odom odom;
extern brain Brain;

// Distance sensors from robot-config.cpp
extern distance frontDist;
extern distance backDist;
extern distance leftDist;
extern distance rightDist;

// ---------- FIELD GEOMETRY ----------
// Origin at back-left inside corner of the field: (0,0)
// X: 0 -> right side wall
// Y: 0 -> back wall (behind the robot at start)
//
// Inside dimension is about 141" on a 6x6 VRC field.
// You can tweak these if you want tighter absolute alignment.

static constexpr float FIELD_WIDTH_IN  = 141.0f;  // X direction (left-right)
static constexpr float FIELD_LENGTH_IN = 141.0f;  // Y direction (back-front)

static constexpr float LEFT_WALL_X  = 0.0f;
static constexpr float RIGHT_WALL_X = FIELD_WIDTH_IN;

static constexpr float BACK_WALL_Y  = 0.0f;
static constexpr float FRONT_WALL_Y = FIELD_LENGTH_IN;

// ---------- ROBOT GEOMETRY ----------
// Robot ~17.5" long (front-back), 15" wide (left-right).
// Half-length ≈ 8.75", half-width ≈ 7.5".
//
// You measured:
//  - Back sensor is 1" from the back bumper.
//  - Front sensor is 4.25" from the front bumper.
//  - Side sensors are flush with the sides.
//
// So from robot center:
//  - Back sensor is 8.75 - 1.0  = 7.75" toward the BACK wall.
//  - Front sensor is 8.75 - 4.25 = 4.50" toward the FRONT wall.
//  - Side sensors are 7.50" toward left/right walls.

static constexpr float FRONT_SENSOR_OFFSET_Y = 4.50f;  // from center toward FRONT
static constexpr float BACK_SENSOR_OFFSET_Y  = 7.75f;  // from center toward BACK
static constexpr float LEFT_SENSOR_OFFSET_X  = 7.50f;  // from center toward LEFT
static constexpr float RIGHT_SENSOR_OFFSET_X = 7.50f;  // from center toward RIGHT

// Distance sensor trust range (based on your tests)
static constexpr float MIN_TRUST_DIST_IN = 3.0f;
static constexpr float MAX_TRUST_DIST_IN = 40.0f;

// Only correct when you're roughly axis-aligned to the field
// (so sensors point along X/Y, not diagonally).
static constexpr float HEADING_TOL_DEG = 20.0f;

// ---------- Helper: read a sensor if it sees something ----------

static bool readTrusted(distance &sensor, float &outInches) {
  if (!sensor.isObjectDetected()) return false;

  float d = sensor.objectDistance(inches);
  if (d < MIN_TRUST_DIST_IN || d > MAX_TRUST_DIST_IN) return false;

  outInches = d;
  return true;
}

// ---------- Conversions for specific headings ----------
//
// Convention:
// - 0°  : robot front points +Y (toward FRONT_WALL_Y).
// - 180°: robot front points -Y (toward BACK_WALL_Y).
//
// Left/right sensors always look sideways to the X walls,
// but which wall they "see" depends on heading.
// Front/back sensors swap which Y wall they see at 0° vs 180°.

// Heading ~ 0° (front toward +Y):
//   - back sensor  → BACK_WALL_Y
//   - front sensor → FRONT_WALL_Y
//   - left sensor  → LEFT_WALL_X
//   - right sensor → RIGHT_WALL_X

static float centerYFromBack_h0(float d) {
  // (Ycenter - BACK_WALL_Y) - BACK_SENSOR_OFFSET_Y = d
  // => Ycenter = BACK_WALL_Y + BACK_SENSOR_OFFSET_Y + d
  return BACK_WALL_Y + BACK_SENSOR_OFFSET_Y + d;
}

static float centerYFromFront_h0(float d) {
  // FRONT_WALL_Y - (Ycenter + FRONT_SENSOR_OFFSET_Y) = d
  // => Ycenter = FRONT_WALL_Y - FRONT_SENSOR_OFFSET_Y - d
  return FRONT_WALL_Y - FRONT_SENSOR_OFFSET_Y - d;
}

static float centerXFromLeft_h0(float d) {
  // (Xcenter - LEFT_WALL_X) - LEFT_SENSOR_OFFSET_X = d
  // => Xcenter = LEFT_WALL_X + LEFT_SENSOR_OFFSET_X + d
  return LEFT_WALL_X + LEFT_SENSOR_OFFSET_X + d;
}

static float centerXFromRight_h0(float d) {
  // RIGHT_WALL_X - (Xcenter + RIGHT_SENSOR_OFFSET_X) = d
  // => Xcenter = RIGHT_WALL_X - RIGHT_SENSOR_OFFSET_X - d
  return RIGHT_WALL_X - RIGHT_SENSOR_OFFSET_X - d;
}

// Heading ~ 180° (front toward -Y):
//   - front sensor → BACK_WALL_Y
//   - back sensor  → FRONT_WALL_Y
//   - left sensor  → RIGHT_WALL_X
//   - right sensor → LEFT_WALL_X

static float centerYFromFront_h180(float d) {
  // (Ycenter - BACK_WALL_Y) - FRONT_SENSOR_OFFSET_Y = d
  // => Ycenter = BACK_WALL_Y + FRONT_SENSOR_OFFSET_Y + d
  return BACK_WALL_Y + FRONT_SENSOR_OFFSET_Y + d;
}

static float centerYFromBack_h180(float d) {
  // FRONT_WALL_Y - (Ycenter + BACK_SENSOR_OFFSET_Y) = d
  // => Ycenter = FRONT_WALL_Y - BACK_SENSOR_OFFSET_Y - d
  return FRONT_WALL_Y - BACK_SENSOR_OFFSET_Y - d;
}

static float centerXFromLeft_h180(float d) {
  // LEFT sensor now faces RIGHT wall:
  // RIGHT_WALL_X - (Xcenter + LEFT_SENSOR_OFFSET_X) = d
  // => Xcenter = RIGHT_WALL_X - LEFT_SENSOR_OFFSET_X - d
  return RIGHT_WALL_X - LEFT_SENSOR_OFFSET_X - d;
}

static float centerXFromRight_h180(float d) {
  // RIGHT sensor now faces LEFT wall:
  // (Xcenter - LEFT_WALL_X) - RIGHT_SENSOR_OFFSET_X = d
  // => Xcenter = LEFT_WALL_X + RIGHT_SENSOR_OFFSET_X + d
  return LEFT_WALL_X + RIGHT_SENSOR_OFFSET_X + d;
}

// ---------- Main correction function ----------

bool distanceOdomCorrect(bool correctX, bool correctY) {
  float heading = chassis.get_absolute_heading();
  heading = reduce_0_to_360(heading);

  // Determine which cardinal direction we're closest to.
  float diff0   = fabs(reduce_negative_180_to_180(heading - 0.0f));
  float diff180 = fabs(reduce_negative_180_to_180(heading - 180.0f));

  bool use_h0   = diff0   <= HEADING_TOL_DEG;
  bool use_h180 = diff180 <= HEADING_TOL_DEG;

  if (!use_h0 && !use_h180) {
    // Not close enough to 0° or 180°, skip correction
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("NoCorr: heading=%.1f   ", heading);
    return false;
  }

  float sumX = 0.0f;
  int   countX = 0;
  float sumY = 0.0f;
  int   countY = 0;
  float d;

  // ---- Heading ~ 0° ----
  if (use_h0) {
    if (correctY && readTrusted(backDist, d)) {
      float y = centerYFromBack_h0(d);
      sumY += y;
      countY++;
    }

    if (correctY && readTrusted(frontDist, d)) {
      float y = centerYFromFront_h0(d);
      sumY += y;
      countY++;
    }

    if (correctX && readTrusted(leftDist, d)) {
      float x = centerXFromLeft_h0(d);
      sumX += x;
      countX++;
    }

    if (correctX && readTrusted(rightDist, d)) {
      float x = centerXFromRight_h0(d);
      sumX += x;
      countX++;
    }
  }

  // ---- Heading ~ 180° ----
  if (use_h180) {
    if (correctY && readTrusted(frontDist, d)) {
      float y = centerYFromFront_h180(d);
      sumY += y;
      countY++;
    }

    if (correctY && readTrusted(backDist, d)) {
      float y = centerYFromBack_h180(d);
      sumY += y;
      countY++;
    }

    if (correctX && readTrusted(leftDist, d)) {
      float x = centerXFromLeft_h180(d);
      sumX += x;
      countX++;
    }

    if (correctX && readTrusted(rightDist, d)) {
      float x = centerXFromRight_h180(d);
      sumX += x;
      countX++;
    }
  }

  if (countX == 0 && countY == 0) {
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("NoCorr: no hits      ");
    return false; // nothing usable
  }

  // --- DIRECTLY UPDATE ODOM ---
  if (countX > 0) {
    odom.X_position = sumX / countX;
  }
  if (countY > 0) {
    odom.Y_position = sumY / countY;
  }

  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("NewX:%.1f NewY:%.1f   ", odom.X_position, odom.Y_position);

  return true;
}
