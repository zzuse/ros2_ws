#include "Kinematics.h"

void Kinematics::set_wheel_distance(float distance) { wheel_distance = distance; }

void Kinematics::set_motor_param(uint8_t motor_id, float per_pulse_distance)
{
    if (motor_id < 2) {
        motor_params[motor_id].per_pulse_distance = per_pulse_distance;
    }
}

void Kinematics::kinematics_forward(float left_speed, float right_speed, float* out_linear_speed,
                                    float* out_angular_speed)
{
    *out_linear_speed = (left_speed + right_speed) / 2.0;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;
}

void Kinematics::kinematics_inverse(float linear_speed, float angular_speed, float* out_left_speed,
                                    float* out_right_speed)
{
    *out_left_speed = linear_speed - (angular_speed * wheel_distance / 2.0);
    *out_right_speed = linear_speed + (angular_speed * wheel_distance / 2.0);
}

void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick)
{
    if (current_time > last_update_time) {
        uint64_t delta_time = current_time - last_update_time;
        deltaTicks[0] = left_tick - motor_params[0].last_encoder_ticks;
        deltaTicks[1] = right_tick - motor_params[1].last_encoder_ticks;
        motor_params[0].motor_speed = deltaTicks[0] * motor_params[0].per_pulse_distance * 1000.0 / delta_time;
        motor_params[1].motor_speed = deltaTicks[1] * motor_params[1].per_pulse_distance * 1000.0 / delta_time;
        // 为了下次计算速度
        motor_params[0].last_encoder_ticks = left_tick;
        motor_params[1].last_encoder_ticks = right_tick;
        last_update_time = current_time;

        update_odom(delta_time);
    }
}

int16_t Kinematics::get_motor_speed(uint8_t motor_id)
{
    if (motor_id < 2) {
        return motor_params[motor_id].motor_speed;
    }
    return 0;
}

odom_t& Kinematics::get_odom() { return odom; }

void Kinematics::TransAngleInPi(float angle, float& out_angle)
{
    if (angle > PI) {
        out_angle -= 2 * PI;
    } else if (angle < -PI) {
        out_angle += 2 * PI;
    }
}

void Kinematics::update_odom(uint16_t dt_ms)
{
    float dt_s = float(dt_ms) / 1000.0; // ms to s
    // realtime angle speed and linear speed
    this->kinematics_forward(motor_params[0].motor_speed, motor_params[1].motor_speed, &odom.linear_speed,
                             &odom.angular_speed);
    // 计算里程计信息
    odom.linear_speed = odom.linear_speed;
    odom.angle += odom.angular_speed * dt_s;
    TransAngleInPi(odom.angle, odom.angle);
    // 线速度分解到X轴Y轴
    float delta_distance = odom.linear_speed * dt_s;
    odom.x += delta_distance * std::cos(odom.angle);
    odom.y += delta_distance * std::sin(odom.angle);
}