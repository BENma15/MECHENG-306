#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_MM;
long startTime = 0;
bool timerActive = false;

long distanceToCounts(double distance_mm);
double countsToDistance(long counts);
bool timer(int waitTime); // waitTime is in ms
#endif