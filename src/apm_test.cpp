#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

// 无人机状态回调函数
void state_callback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe("/mavros/state", 10, state_callback);

    ros::Rate rate(20);

    ROS_INFO("Initializing...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected.");

    // 设置无人机模式为GUIDED
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if (cl.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    // 解锁无人机
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }

    // 发布起飞的指令，向上飞行10米
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 10;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff_cl.call(srv_takeoff)) {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
    }

    // 等待10秒.  这一行代码非常重要，如果没有则不能飞行
    sleep(10);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    ros::Publisher local_vel_pub = n.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Time time_start = ros::Time::now();

    //转圈圈飞行
    while (ros::ok()) {
        pose.pose.position.x = sin(2.0 * M_PI * 2.0 * (ros::Time::now() - time_start).toSec());
        pose.pose.position.y = cos(2.0 * M_PI * 2.0 * (ros::Time::now() - time_start).toSec());

        vel.linear.x = 2.0 * M_PI * 2.0 * cos(2.0 * M_PI * 0.1 * (ros::Time::now() - time_start).toSec());
        vel.linear.y = -2.0 * M_PI * 2.0 * sin(2.0 * M_PI * 0.1 * (ros::Time::now() - time_start).toSec());

        local_pos_pub.publish(pose);
        local_vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
