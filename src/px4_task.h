/**
 * 对无人机进行封装
 */
#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <Eigen/Eigen>
#include "module/pid.h"

#define SERVO_CLOSE 10
#define SERVO_OPEN 90
#ifndef __PX4_TASK_H
#define __PX4_TASK_H
class Multirotor
{
public:
    Multirotor();
    ~Multirotor();
    void init();
    void run();

private:
    \\ PID参数
        PID vel_pid_x;
    PID vel_pid_y;
    PID vel_pid_z;
    PID acc_pid_x;
    PID acc_pid_y;
    PID acc_pid_z;

    ros::NodeHandle nh;
    ros::Subscriber state_sub;          // 订阅无人机当前时刻状态
    ros::Subscriber position_sub;       // 订阅无人机当前位置
    ros::Subscriber velocity_sub;       // 订阅无人机当前速度
    ros::Publisher vel_pub;             // 发布无人机期望速度
    ros::Publisher pos_pub;             // 发布无人机期望位置
    ros::Publisher drop_pub;            // 发布投放指令
    ros::ServiceClient arming_client;   // 无人机解锁
    ros::ServiceClient set_mode_client; // 无人机设置模式
    mavros_msgs::OverrideRCIn drop_cmd; // 投放指令

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void pid_init();
    void pid_reset();

    void flytopoint(Eigen::Vector3d point);
    void setoffboardmode();
    void takeoff();
    void land();
    void drop_bottle();

    mavros_msgs::State current_state;         // 无人机当前状态
    Eigen::Vector3d drone_pos;                // 无人机当前位置  // enu
    Eigen::Vector3d drone_vel;                // 无人机当前速度
    std::vector<Eigen::Vector3d> task_points; // 任务点
    float fly_height;                         // 飞行高度
    float min_dis;                            // 到达任务点的最小距离
    int switchflag;                           // 任务点切换标志
};

#endif