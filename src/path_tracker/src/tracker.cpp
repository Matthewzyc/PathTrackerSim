#include "tracker.hpp"

Tracker::Tracker() : pid_(control_signal_), smc_(control_signal_)
{
    scan_subscriber_ = nh_.subscribe("/scan", 1, &Tracker::scanCallback, this);
    cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pid_.setParameters(0.5, 0.0, 0.);
    smc_.setParameters(0.7, 0.5, 0.05);
}

void Tracker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    double angle_increment = msg->angle_increment;
    double angle_min = msg->angle_min;
    int total_points = msg->ranges.size();
    
    double right_angle_start = -(PI_ / 3);
    double right_angle_end = -(PI_ / 4);
    double left_angle_start = PI_ / 4;
    double left_angle_end = PI_ / 3;

    int right_start = (right_angle_start - angle_min) / angle_increment;
    int right_end = (right_angle_end - angle_min) / angle_increment;
    int left_start = (left_angle_start - angle_min) / angle_increment;
    int left_end = (left_angle_end - angle_min) / angle_increment;

    double right_sum = 0, left_sum = 0;
    int right_count = 0, left_count = 0;

    for (int i = right_start; i <= right_end; i++) {
        if (msg->ranges[i] <= 5 && msg->ranges[i] >= 0.2) {
            right_sum += msg->ranges[i];
            right_count++;
        }
    }

    for (int j = left_start; j <= left_end; j++) {
        if (msg->ranges[j] <= 5 && msg->ranges[j] >= 0.2) {
            left_sum += msg->ranges[j];
            left_count++;
        }
    }

    double right_45 = right_count > 0 ? right_sum / right_count : 0;
    double left_45 = left_count > 0 ? left_sum / left_count : 0;

    double error = left_45 - right_45;
    std::cout << "error(left - right):" << error << std::endl;

    if (std::abs(error) <= 0.02) {
        error = 0;
    }

    pid_.calculate(error);
    publishCommand(control_signal_);
}

void Tracker::publishCommand(double control_signal)
{
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 1.0;
    cmd_msg.angular.z = control_signal;

    // 获取当前时间戳
    ros::Time current_time = ros::Time::now();
    std::cout << "Control Signal: " << control_signal << std::endl;

    cmd_publisher_.publish(cmd_msg);
}
