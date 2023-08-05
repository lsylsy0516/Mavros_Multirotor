#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

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
    ROS_INFO("Header: seq = %u, timestamp = %f, frame_id = %s", sequence_number, timestamp.toSec(), frame_id.c_str());
    ROS_INFO("Position: x = %f, y = %f, z = %f", position_x, position_y, position_z);
    ROS_INFO("Orientation: x = %f, y = %f, z = %f, w = %f", orientation_x, orientation_y, orientation_z, orientation_w);
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"mav_test_node");
    ros::NodeHandle("~");
    return 0;
}
