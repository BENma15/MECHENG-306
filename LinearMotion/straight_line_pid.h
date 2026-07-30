#ifndef STRAIGHT_LINE_PID_H
#define STRAIGHT_LINE_PID_H

class StraightLinePID {
  private:
    double _kp, _ki, _kd;
    double _integral, _prev_error;
  public:
    StraightLinePID (double kp, double ki, double kd);
    double compute (double setpoint, double measured);
    void reset();
};

#endif