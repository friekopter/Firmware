
#include "compass_utils.h"

float compassNormalize(float heading) {
    if (heading < 0.0f)
	heading += 360.0f;
    if (heading >= 360.0f)
	heading -= 360.0f;

    return heading;
}

// calculate the shortest distance in yaw to get from b => a
float compassDifference(float a, float b) {
    float diff = b - a;

    if (diff > 180.0f)
	diff -= 360.0f;
    if (diff <= -180.0f)
	diff += 360.0f;

    return diff;
}
