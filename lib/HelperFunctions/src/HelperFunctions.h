#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_REV;;
extern const double WHEEL_RADIUS_MM;

long distanceToCounts(double distance_mm);
// double countsToDistance(long counts);

#endif