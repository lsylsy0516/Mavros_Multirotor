#include "pid.h"

PID::PID(float kp, float ki, float kd, float max_out, float range_rough, float range_fine)
    : kp(kp), ki(ki), kd(kd), max_out(max_out), range_rough(range_rough), range_fine(range_fine) {}
PID::PID(){}
PID::~PID() {}

void PID::init()
{
    target = 0.0f;
    error = 0.0f;
    error_last = 0.0f;
    error_sum = 0.0f;
    output = 0.0f;
}
void PID::reset()
{
    error = 0.0f;
    error_last = 0.0f;
    error_sum = 0.0f;
    output = 0.0f;
}
void PID::set_target(float t)
{
    target = t;
}
float PID::update(float current)
{
    error = target - current;
    float error_delta = error - error_last;

    if (error > range_rough)  // bangbang
    {
        output = max_out;
        error_sum = 0;
    }
    else if (error < -range_rough)  // bangbang
    {
        output = -max_out;
        error_sum = 0;
    }
    else if (fabsf(error) < range_fine)  //细调
    {
        error_sum += error;  //积分上限判断
        if (error_sum > max_out)
            error_sum = max_out;
        if (error_sum < -max_out)
            error_sum = -max_out;

        output = kp * error + ki * error_sum + kd * error_delta;
    }
    else  //粗调
    {
        output = kp * (error + ((error > 0.0f) ? range_rough : -range_rough)) + kd * error_delta;
        error_sum = ((error > 0.0f) ? max_out : -max_out);
    }

    /*----- 输出上限 -----*/
    if (output > max_out)
        output = max_out;
    if (output < -max_out)
        output = -max_out;

    error_last = error;

    return output;
}