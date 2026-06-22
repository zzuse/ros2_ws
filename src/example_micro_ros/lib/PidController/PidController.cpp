#include "PidController.h"
#include <Arduino.h>

PidController::PidController(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float PidController::update(float current)
{
    error_ = target_ - current; // 计算误差
    error_sum_ += error_;       // 计算误差积分
    // 积分限幅，防止积分过大导致系统不稳定
    if (error_sum_ > integral_up_) {
        error_sum_ = integral_up_;
    } else if (error_sum_ < -integral_up_) {
        error_sum_ = -integral_up_;
    }
    derror_ = error_ - last_error_; // 计算误差微分, 误差变化率
    last_error_ = error_;           // 更新上一次的误差值

    // 计算PID输出公式
    float output = kp_ * error_ + ki_ * error_sum_ + kd_ * derror_;
    // 输出限幅，防止输出过大导致系统不稳定
    if (output > out_max_) {
        output = out_max_;
    } else if (output < out_min_) {
        output = out_min_;
    }
    return output;
}

void PidController::update_target(float target) { target_ = target; }

void PidController::update_pid(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset()
{
    error_ = 0;
    error_sum_ = 0;
    derror_ = 0;
    last_error_ = 0;
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    integral_up_ = 2500;
    out_min_ = 0;
    out_max_ = 0;
}

void PidController::out_limit(float min, float max)
{
    out_min_ = min;
    out_max_ = max;
}