#include "tracker.hpp"

Tracker::Tracker() : pid_(control_signal_), smc_(control_signal_)
{
    scan_subscriber_ = nh_.subscribe("/scan", 1, &Tracker::scanCallback, this);
    cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pid_.setParameters(1.1, 0.0, 4.5);
    smc_.setParameters(0.2, 0.5, 0.02); // 设置 epsilon 为 0.02，可以根据实际情况调整
}

void Tracker::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double angle_increment = msg->angle_increment;
    double angle_min = msg->angle_min;
    int total_points = msg->ranges.size();
    int mid_point = total_points / 2;

    // 定义左右两侧激光的角度范围
    double right_angle_start = -1.57;  // 右侧开始角度
    double right_angle_end = -0.7;     // 右侧结束角度
    double left_angle_start = 0.7;     // 左侧开始角度
    double left_angle_end = 1.57;      // 左侧结束角度

    // 计算左右激光点的索引
    int right_start = mid_point + (int)((right_angle_start - angle_min) / angle_increment);
    int right_end = mid_point + (int)((right_angle_end - angle_min) / angle_increment);
    int left_start = mid_point + (int)((left_angle_start - angle_min) / angle_increment);
    int left_end = mid_point + (int)((left_angle_end - angle_min) / angle_increment);

    // 初始化累加值
    double right_sum = 0, left_sum = 0;
    double right_weight_sum = 0, left_weight_sum = 0;

    // 计算右侧的加权距离
    for (int i = right_start; i <= right_end; i++)
    {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
        {
            double weight = cos(i * angle_increment); // 距离较近的点权重更大
            right_sum += msg->ranges[i] * weight;
            right_weight_sum += weight;
        }
    }

    // 计算左侧的加权距离
    for (int j = left_start; j <= left_end; j++)
    {
        if (msg->ranges[j] >= msg->range_min && msg->ranges[j] <= msg->range_max)
        {
            double weight = cos(j * angle_increment); // 距离较近的点权重更大
            left_sum += msg->ranges[j] * weight;
            left_weight_sum += weight;
        }
    }

    // 计算左右两侧的加权平均距离
    double right_average = (right_weight_sum > 0) ? right_sum / right_weight_sum : 0;
    double left_average = (left_weight_sum > 0) ? left_sum / left_weight_sum : 0;

    // 计算误差 (小车相对于中心的偏移)
    double error = left_average - right_average;

    // 如果误差小于某个阈值，认为没有偏移
    if (std::abs(error) <= 0.05)
    {
        error = 0;  // 如果小车已经很接近中心线，设置误差为零
    }

    // 使用滑模控制计算 (也可以用 PID 控制)
    smc_.calculate(error);

    // 发布控制命令
    publishCommand(control_signal_);

    // 调试输出控制信号
    std::cout << "Control Signal: " << control_signal_ << std::endl;
}



void Tracker::publishCommand(double control_signal)
{
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0.5;              
    cmd_msg.angular.z = -control_signal;
    cmd_publisher_.publish(cmd_msg);
}