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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

#include <Eigen/Eigen>
#include "module/pid.h"

#ifndef DRONE_SERVICE
#define DRONE_SERVICE

class Multirotor_Service{
public:
    Multirotor_Service();
    ~Multirotor_Service();
    void init();
    void run();

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;          // 订阅无人机当前时刻状态
    ros::Subscriber position_sub;       // 订阅无人机当前位置
    ros::Subscriber velocity_sub;       // 订阅无人机当前速度
    ros::Publisher pos_pub;             // 全程使用位置控制
    mavros_msgs::OverrideRCIn drop_cmd; // 投放指令

    void state_cb();
    void position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void 
};


#endif