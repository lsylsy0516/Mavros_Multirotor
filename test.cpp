#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

Eigen::Vector3d pos_drone_vio;                          //无人机当前位置 (vision) using FLU
Eigen::Quaterniond q_vio;
Eigen::Vector3d Euler_vio;                              //无人机当前姿态 (vision)
ros::Publisher vision_pub;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void send_to_fcu()
{
    geometry_msgs::PoseStamped vision;  // should be ENU
    vision.pose.position.x = pos_drone_vio[0] ; // E 
    vision.pose.position.y = pos_drone_vio[1] ; // N
    vision.pose.position.z = pos_drone_vio[2] ; // U
    ROS_INFO("E,N,U= %f,%f,%f",vision.pose.position.x,vision.pose.position.y,vision.pose.position.z);
    vision.pose.orientation.x = q_vio.x();
    vision.pose.orientation.y = q_vio.y();
    vision.pose.orientation.z = q_vio.z();
    vision.pose.orientation.w = q_vio.w();

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision); // vision 
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    // ROS_INFO("seq is %d",current_state.header.seq);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    uint32_t sequence_number = msg->header.seq;
    ros::Time timestamp = msg->header.stamp;
    std::string frame_id = msg->header.frame_id;

    // 获取pose信息
    double position_x = msg->pose.position.x;
    double position_y = msg->pose.position.y;
    double position_z = msg->pose.position.z;

    double orientation_x = msg->pose.orientation.x;
    double orientation_y = msg->pose.orientation.y;
    double orientation_z = msg->pose.orientation.z;
    double orientation_w = msg->pose.orientation.w;

    // 打印信息
    // ROS_INFO("Header: seq = %u, timestamp = %f, frame_id = %s", sequence_number, timestamp.toSec(), frame_id.c_str());
    // ROS_INFO("Position: x = %f, y = %f, z = %f", position_x, position_y, position_z);
    // ROS_INFO("Orientation: x = %f, y = %f, z = %f, w = %f", orientation_x, orientation_y, orientation_z, orientation_w);
}

void vins_estimator_cb(const nav_msgs::Odometry::ConstPtr& msg)
{   
    ROS_INFO("VISION RECEIVED!");
    nav_msgs::Odometry vision_pose;
    vision_pose = *msg;

    //  FLU xi
    // pos_drone_vio[0] = vision_pose.pose.pose.position.y; //bugs
    // pos_drone_vio[1] = vision_pose.pose.pose.position.x;
    // pos_drone_vio[2] = (-1)*vision_pose.pose.pose.position.z; 
    pos_drone_vio[0] = vision_pose.pose.pose.position.x;
    pos_drone_vio[1] = vision_pose.pose.pose.position.y;
    pos_drone_vio[2] = vision_pose.pose.pose.position.z; 

    q_vio.x() = vision_pose.pose.pose.orientation.x;
    q_vio.y() = vision_pose.pose.pose.orientation.y;
    q_vio.z() = vision_pose.pose.pose.orientation.z;
    q_vio.w() = vision_pose.pose.pose.orientation.w;

    send_to_fcu();
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"mav_test_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                 ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber vins_estimator_sub = nh.subscribe<nav_msgs::Odometry>
                                 ("vins/odometry", 10, vins_estimator_cb);
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("mavros/vision_pose/pose", 10);
    ros::spin();
    return 0;
}
