#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>

void setupLimitSwitch();
void updateLeftLimit();
bool isLeftLimitPressed();

#endif