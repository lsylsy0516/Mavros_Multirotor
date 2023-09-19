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
#include "offboard/drop.h"
#include "offboard/flightByVel.h"
#include "offboard/flightByOffset.h"
#include "offboard/takeoffOrLanding.h"

#ifndef DRONE_SERVICE
#define DRONE_SERVICE

class Multirotor_Server
{
public:
        Multirotor_Server();
        ~Multirotor_Server();
        void run();

private:
        // drone state
        mavros_msgs::State current_state;
        Eigen::Vector3d drone_pos;
        Eigen::Vector3d drone_vel;
        int flight_status;
        float vel_x;
        float vel_y;
        float delta_x;
        float delta_y;
        float fly_time;

        // drone control
        enum FlightStatus
        {
                TAKEOFF=1,
                LANDING,
                FLIGHTBYVEL,
                FLIGHTBYOFFSET,
                DROP,
                IDLE
        };

        // control the drone
        void takeoff();
        void landing();
        void flightByVel();
        void flightByOffset();
        void drop();
        void idle();

        // subscribe the state of the drone
        ros::NodeHandle nh;
        ros::Subscriber state_sub;    // 订阅无人机当前时刻状态
        ros::Subscriber position_sub; // 订阅无人机当前位置
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

        // execute drone control
        ros::Subscriber velocity_sub;       // 订阅无人机当前速度
        ros::Publisher pos_pub;             // 发布无人机期望位置
        ros::Publisher vel_pub;             // 发布无人机期望速度
        ros::ServiceClient set_mode_client;  // 发布起飞降落
        mavros_msgs::OverrideRCIn drop_cmd; // 投放指令

        // do service from the upper module
        ros::ServiceServer takeoffOrLanding_server;
        ros::ServiceServer flightByVel_server;
        ros::ServiceServer flightByOffset_server;
        ros::ServiceServer drop_server;

        bool takeoffOrLanding_serverCB(offboard::takeoffOrLanding::Request &req,
                                       offboard::takeoffOrLanding::Response &res);
        bool flightByVel_serverCB(offboard::flightByVel::Request &req,
                                  offboard::flightByVel::Response &res);
        bool flightByOffset_serverCB(offboard::flightByOffset::Request &req,
                                     offboard::flightByOffset::Response &res);
        bool drop_serverCB(offboard::drop::Request &req,
                           offboard::drop::Response &res);

};

#endif