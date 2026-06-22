#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController {
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    float target_;
    float kp_;
    float ki_;
    float kd_;
    // pid
    float error_;
    float error_sum_;
    float derror_;
    float last_error_;
    float integral_up_ = 2500;
    float out_min_;
    float out_max_;

public:
    float update(float current);
    void update_target(float target);
    void update_pid(float kp, float ki, float kd);
    void reset();
    void out_limit(float min, float max); // 输出限幅
};

#endif // __PID_CONTROLLER_H__