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
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

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
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber position_sub;
    ros::Publisher move_pub;
    ros::Publisher drop_pub ;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    mavros_msgs::OverrideRCIn drop_cmd;

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void flytopoint(Eigen::Vector3d point);
    void setoffboardmode();
    void takeoff();
    void land();
    void drop_bottle();
    
    mavros_msgs::State current_state;
    Eigen::Vector3d drone_pos; // 无人机当前位置
    std::vector<Eigen::Vector3d> task_points; // 任务点
    float fly_height; // 飞行高度
    float min_dis; // 到达任务点的最小距离
    int switchflag; // 任务点切换标志
};


#endif