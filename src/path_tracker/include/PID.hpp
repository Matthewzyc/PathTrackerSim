#ifndef PID_H
#define PID_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class PID 
{
public:
    PID(double &control_signal) : control_signal_(control_signal) {}
    void setParameters(double kp, double ki, double kd);
    void calculate(double average_lateral_deviation);

private:
    double &control_signal_;
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
    double derivative_;
    double prev_derivative_;
};

#endif