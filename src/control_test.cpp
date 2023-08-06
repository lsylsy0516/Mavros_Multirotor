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
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                                   ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>
                                   ("/mavros/setpoint_accel/accel", 10);
    ros::Publisher AttitudeTarget_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                                        ("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher PositionTarget_pub = nh.advertise<mavros_msgs::PositionTarget>
                                        ("/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //我希望我只用这玩意就行
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::PositionTarget PositionTarget;
    mavros_msgs::AttitudeTarget AttitudeTarget;

    //以下是为了在仿真里面解决arm 无法解除的问题
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    int flag = 0;
    while(ros::ok())
    {   

        if(!current_state.armed)
        {
            if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
        }
        // 设置offboard 模式，并设置为armed
        if( current_state.mode != "OFFBOARD")
        {
            if( set_mode_client.call(offb_set_mode) && // 设置为 offboard模式
                    offb_set_mode.response.mode_sent) 
            ROS_INFO("Offboard enabled");
        }

        // if (current_state.mode != "OFFBOARD")
            // break;

        local_pos_pub.publish(pose);
        ROS_INFO("position only %d",flag);
        
        ros::spinOnce();
        flag ++;
        rate.sleep();
        if (flag == 200)
            break;
    }
    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "AUTO.PRECLAND";
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && // 设置为 AUTO.PRECLAND模式
                    offb_set_mode.response.mode_sent) // 接收成功
        {
            ROS_INFO("Land Success");
        }
    return 0;
}