#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_MM;

long distanceToCounts(double distance_mm);
double countsToDistance(long counts);
bool encoderCountsChanged();
class Timer
{
private:
    unsigned long startTime;
    bool timerActive;

public:
    Timer();

    bool startTimer(unsigned long waitTime);
    void reset();
};
#endif