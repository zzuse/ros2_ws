#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"

typedef struct {
    float per_pulse_distance; // 每个脉冲对应的距离，单位mm
    int16_t motor_speed;
    int64_t last_encoder_ticks; // 上一次编码器的计数值
} motor_param_t;

typedef struct {
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
} odom_t;

class Kinematics {
private:
    motor_param_t motor_params[2]; // 0: left, 1: right
    uint64_t last_update_time;     // 上一次更新的时间，单位ms
    float wheel_distance = 0.0;    // 轮距，单位mm
    int16_t deltaTicks[2]{0, 0};
    odom_t odom;

public:
    Kinematics() = default;
    ~Kinematics() = default;

    void update_odom(uint16_t dt_ms);
    odom_t& get_odom();
    void TransAngleInPi(float angle, float& out_angle);

    void set_wheel_distance(float distance); // 设置轮距，单位mm

    void set_motor_param(uint8_t motor_id, float per_pulse_distance);
    // 计算线速度和角速度, 正解
    void kinematics_forward(float left_speed, float right_speed, float* out_linear_speed, float* out_angular_speed);
    // 计算轮速, 逆解
    void kinematics_inverse(float linear_speed, float angular_speed, float* out_left_speed, float* out_right_speed);
    // 更新电机速度和编码器计数值
    void update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick);
    // 获取当前线速度和角速度
    int16_t get_motor_speed(uint8_t motor_id);
};

#endif // __KINEMATICS_H__