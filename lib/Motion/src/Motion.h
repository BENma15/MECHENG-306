#ifndef MOTION_H
#define MOTION_H

extern volatile long countA;
extern volatile long countB;
extern int moveSpeed;

void MotionInit();
void moveTo(double dx, double dy);
void runAxis(double deltaA_mm, double deltaB_mm);
long distanceToCounts(double distance_mm);
void setRightMotor(int8_t dir, int pwm);
void setLeftMotor(int8_t dir, int pwm);


#endif