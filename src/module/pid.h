/**
 * @file pid.h
 * @brief PID控制器类
 * 用于控制无人机的速度，输入为无人机当前位置，输出为无人机期望速度
*/
#include <cmath>
#ifndef __PID_H
#define __PID_H
class PID
{
public:
    PID(float kp,float ki,float kd,float max_out,float range_rough,float range_fine);
    ~PID();
    void init();
    void reset();
    void set_target(float target);
    float update(float current);
private:
    float kp,ki,kd;
    float target;
    float error;
    float error_last;
    float error_sum;
    float output;
    float max_out;
    float range_rough;
    float range_fine;
};



#endif