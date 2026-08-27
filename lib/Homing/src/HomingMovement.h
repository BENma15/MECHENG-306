#include <Arduino.h>

void homingMove_start(double unitX, double unitY, double vf_mm_s);
void homingMove_tick();
void homingMove_stop();
bool homingMove_isActive();