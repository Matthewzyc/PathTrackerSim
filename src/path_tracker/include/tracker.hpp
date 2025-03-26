#ifndef TRACKER_H
#define TRACKER_H

#include "PID.hpp"
#include "SMC.hpp"

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class Tracker
{
public:
    Tracker();

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void publishCommand(double control_signal);

    const double PI_ = 3.141592653589793;

    ros::NodeHandle nh_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher cmd_publisher_;

    PID pid_;
    SMC smc_;

    double control_signal_;
};

#endif
