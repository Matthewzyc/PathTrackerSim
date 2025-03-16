#ifndef SMC_H
#define SMC_H

#include <iostream>
#include <cmath>

class SMC
{
public:
    SMC(double &control_signal) : control_signal_(control_signal), prev_error_(0.0), lambda_(1.0), k_(0.2), epsilon_(0.02), prev_control_signal_(0.0) {}
    void setParameters(double lambda, double k, double epsilon);
    void calculate(double error);

private:
    double &control_signal_;      // 控制信号引用
    double prev_error_;           // 上一次的偏差
    double lambda_;               // 滑模面参数
    double k_;                    // 滑模增益
    double epsilon_;              // 平滑因子
    double prev_control_signal_;  // 上一次控制信号，用于低通滤波
    const double dt_ = 0.1;       // 控制器时间步长（假设 10 Hz）
};

#endif