#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

extern volatile long countA;
extern volatile long countB;
extern int moveSpeed;

void MotionInit();
void moveTo(double dx, double dy);
void runAxis(double deltaA_mm, double deltaB_mm);
long distanceToCounts(double distance_mm);
void motionsetRightMotor(int8_t dir, int pwm);
void motionsetLeftMotor(int8_t dir, int pwm);


#endif