#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_MM;
extern unsigned long startTime;
extern bool timerActive;
extern unsigned long startTime;
extern bool timerActive;

long distanceToCounts(double distance_mm);
double countsToDistance(long counts);
bool timer(unsigned long waitTime); // waitTime is in ms
bool timer(int waitTime); // Add this so existing code that calls timer(int) links correctly:
#endif