#ifndef LIMIT_SWITCH_DEBOUNCE_H
#define LIMIT_SWITCH_DEBOUNCE_H

#include <Arduino.h>

void setupLimitSwitches();
void updateLimitSwitches();

bool isLeftLimitPressed();
bool isRightLimitPressed();
bool isTopLimitPressed();
bool isBottomLimitPressed();

bool isAnyLimitPressed();

#endif
