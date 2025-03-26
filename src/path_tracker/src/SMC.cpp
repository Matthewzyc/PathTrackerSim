#include "SMC.hpp"

// 设置滑模控制器参数
void SMC::setParameters(double lambda, double k, double epsilon)
{
    lambda_ = lambda;
    k_ = k;
    epsilon_ = epsilon;
}

void SMC::calculate(double error)
{
    // 计算误差的变化率
    double error_dot = (error - prev_error_) / dt_;
    prev_error_ = error;

    // 计算滑模面
    double s = error_dot + lambda_ * error;

    // 增加更强的控制信号响应，避免过度反应
    double control_signal_raw = k_ * tanh(s / (epsilon_ + 0.05 * fabs(error))) * 0.7;

    // 增加误差衰减项，避免大误差时过度修正
    double attenuation = 1.0 / (1.0 + 0.3 * fabs(error));  // 增加衰减
    control_signal_raw *= attenuation;

    // 自适应低通滤波，误差大时减少滤波，误差小时增加滤波
    double alpha = 0.8 - 0.7 * tanh(fabs(error));  // 增加滤波强度
    control_signal_ = alpha * prev_control_signal_ + (1 - alpha) * control_signal_raw;

    // 限制控制信号的范围，避免过大或过小
    control_signal_ = std::max(-0.8, std::min(0.8, control_signal_));

    // 更新上一轮控制信号
    prev_control_signal_ = control_signal_;
}

