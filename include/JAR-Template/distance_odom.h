// distance_odom.h
#pragma once

// Try to correct odometry using the 4 distance sensors.
// - correctX: whether to try to correct X using left/right sensors
// - correctY: whether to try to correct Y using front/back sensors
// Returns true if at least one axis was corrected.
bool distanceOdomCorrect(bool correctX = true, bool correctY = true);
