#include "PID.hpp"

void PID::setParameters(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::calculate(double error)
{
    //error = 0.1 * prev_error_ + 0.9 * error;
    integral_ += error;
    derivative_ = error - prev_error_;
    control_signal_ = kp_ * error + ki_ * integral_ + kd_ * derivative_;
    prev_error_ = error;
}