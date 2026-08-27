#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <Arduino.h>

extern const double COUNTS_PER_MM;

long globalPosX; //mm
long globalPosY; //mm

long distanceToCounts(double distance_mm);
double countsToDistance(long counts);
int8_t outputToDirection(double output);
int applyMotorPwmLimits(int pwm, int8_t direction, int minimumDrivePwm);
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