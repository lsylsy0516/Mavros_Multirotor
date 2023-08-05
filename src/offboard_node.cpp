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
    ROS_INFO("seq is %d",current_state.header.seq);
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

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    geometry_msgs::TwistStamped vel_linear;
    vel_linear.twist.linear.x=1;
    vel_linear.twist.linear.y=1;
    vel_linear.twist.linear.z=1;
    geometry_msgs::Vector3Stamped acc;
    acc.vector.x=1;
    acc.vector.y=1;
    acc.vector.z=1;
    //
    mavros_msgs::PositionTarget PositionTarget;
    mavros_msgs::AttitudeTarget AttitudeTarget;
    local_pos_pub.publish(pose);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int flag=0;
    while(ros::ok())
    {
        // 设置offboard 模式，并设置为armed
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            if( set_mode_client.call(offb_set_mode) && // 设置为 offboard模式
                    offb_set_mode.response.mode_sent) // 接收成功
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 如果上一环节完成，超过10ms 没有last_request，
        if(ros::Time::now() - last_request > ros::Duration(10.0))
        {
            flag++;
            last_request = ros::Time::now();
        }

       if(flag==0)
       {
           local_pos_pub.publish(pose);
           ROS_INFO("position only");
       }
       if(flag==1)
       {
           local_vel_pub.publish(vel_linear);
           ROS_INFO("velocity only");
       }
       if(flag==2)
       {
           local_acc_pub.publish(acc);
           ROS_INFO("acceleration_or_force only");
       }

        if(flag==0)
        {
            PositionTarget.position.x=0;
            PositionTarget.position.y=0;
            PositionTarget.position.z=2;
            PositionTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
            PositionTarget.type_mask=PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
            PositionTarget_pub.publish(PositionTarget);
            ROS_INFO("position");
        }
        if(flag==1)
        {
            PositionTarget.velocity.x=1;
            PositionTarget.velocity.y=0;
            PositionTarget.position.z=3;
            PositionTarget.type_mask=PositionTarget.IGNORE_PX|PositionTarget.IGNORE_PY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
            PositionTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
            PositionTarget_pub.publish(PositionTarget);
            ROS_INFO("velocity and alt");
        }
        if(flag==2)
        {
            PositionTarget.acceleration_or_force.x=0;
            PositionTarget.acceleration_or_force.y=1;
            PositionTarget.position.z=2;
            PositionTarget.type_mask=PositionTarget.IGNORE_PX|PositionTarget.IGNORE_PY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
            PositionTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
            PositionTarget_pub.publish(PositionTarget);
            ROS_INFO("acceleration_or_force and alt");
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
