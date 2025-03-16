#include "SMC.hpp"

// 设置滑模控制器参数
void SMC::setParameters(double lambda, double k, double epsilon)
{
    lambda_ = lambda;
    k_ = k;
    epsilon_ = epsilon;
}

// 滑模控制算法
void SMC::calculate(double error)
{
    // 计算误差的变化率
    double error_dot = (error - prev_error_) / dt_;
    prev_error_ = error;

    // 计算滑模面
    double s = error_dot + lambda_ * error;

    // 使用低通滤波器平滑控制信号，减少高频抖动
    double control_signal_raw = k_ * s / (epsilon_ + fabs(s));

    // 控制信号平滑（低通滤波）
    control_signal_ = 0.8 * prev_control_signal_ + 0.2 * control_signal_raw;  // 加强平滑效果

    // 更新上一轮控制信号
    prev_control_signal_ = control_signal_;
}