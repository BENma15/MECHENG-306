#ifndef LIMIT_SWITCH_DEBOUNCE_H
#define LIMIT_SWITCH_DEBOUNCE_H

#include <Arduino.h>

void setupLimitSwitches();

bool LimitSwitch_leftPressed();
bool LimitSwitch_rightPressed();
bool LimitSwitch_topPressed();
bool LimitSwitch_bottomPressed();

#endif
