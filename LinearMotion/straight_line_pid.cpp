#include "straight_line_pid.h"

StraightLinePID::StraightLinePID (double kp, double ki, double kd) : _kp(kp), _ki(ki), _kd(kd), _integral(0), _prev_error(0) {}

double StraightLinePID::compute (double setpoint, double measured) {
  double error = setpoint - measured;
  _integral += error;
  double derivative = (error - _prev_error);
  _prev_error = error;
  return _kp * error + _ki * _integral + _kd * derivative;
}

void StraightLinePID::reset() {
  _integral = 0;
  _prev_error = 0;
}