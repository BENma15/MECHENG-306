#include "HelperFunctions.h"

// Distance to encoder count equation variables
const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 7;

long distanceToCounts(double distance_mm) {
    return (long) round(COUNTS_PER_REV * distance_mm / (2.0 * PI * WHEEL_RADIUS_MM));
}

// double countsToDistance(long counts) {
//     return
// }