#ifndef MOTION_H
#define MOTION_H

extern volatile long countA;
extern volatile long countB;
extern int moveSpeed;

void setupMotion();
void moveTo(double dx, double dy);

#endif