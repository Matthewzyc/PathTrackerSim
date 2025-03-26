#include "PID.hpp"

void PID::setParameters(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::calculate(double error)
{  
    derivative_ = error - prev_error_;
    integral_ += error;
    control_signal_ = kp_ * error + ki_ * integral_ + kd_ * (derivative_ - prev_derivative_);
    prev_error_ = error;
    prev_derivative_ = derivative_;
}