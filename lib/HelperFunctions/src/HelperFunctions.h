#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_MM;

long distanceToCounts(double distance_mm);
double countsToDistance(long counts);

#endif