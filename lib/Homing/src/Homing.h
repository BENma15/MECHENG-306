#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include <LimitSwitchDebounce.h>
#include <Motion.h>

void homingInit(); // initialisation for homing to be called in main.cpp
void driveUntilTopLimit();
void driveUntilLeftLimit();
void testLoop();    // to be called in main.cpp 

#endif