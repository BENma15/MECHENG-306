#include "helperFunctions.h"

long distanceToCounts(double distance_mm) {
    return (long) round(COUNTS_PER_REV * distance_mm / (2.0 * PI * WHEEL_RADIUS_MM));
}

double countsToDistance(long counts) {
    return
}