#include "DroneServer.h"
#include <mavros_msgs/State.h>
using namespace std;


Multirotor_Server::Multirotor_Server()
{
    // subscribe the state of the drone
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Multirotor_Server::state_cb, this);
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Multirotor_Server::position_cb, this);
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, &Multirotor_Server::velocity_cb, this);
    
    // execute drone control
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    // do service from the upper module
    takeoffOrLanding_server = nh.advertiseService("takeoffOrLanding", &Multirotor_Server::takeoffOrLanding_serverCB, this);
    flightByVel_server = nh.advertiseService("flightByVel", &Multirotor_Server::flightByVel_serverCB, this);
    flightByOffset_server = nh.advertiseService("flightByOffset", &Multirotor_Server::flightByOffset_serverCB, this);
    drop_server = nh.advertiseService("drop", &Multirotor_Server::drop_serverCB, this);

}

Multirotor_Server::~Multirotor_Server()
{

}

void Multirotor_Server::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void Multirotor_Server::position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_pos(0) = msg->pose.position.x;
    drone_pos(1) = msg->pose.position.y;
    drone_pos(2) = msg->pose.position.z;
}

void Multirotor_Server::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    drone_vel(0) = msg->twist.linear.x;
    drone_vel(1) = msg->twist.linear.y;
    drone_vel(2) = msg->twist.linear.z;
}

bool Multirotor_Server::takeoffOrLanding_serverCB(offboard::takeoffOrLanding::Request  &req,
        offboard::takeoffOrLanding::Response &res)
{
    res.ack = 1;
    ROS_INFO("takeoff or landing server cb ");
    if (req.takeoffOrLanding == 1)
    {
        flight_status = Multirotor_Server::FlightStatus::TAKEOFF;
    }
    else if (req.takeoffOrLanding == 2)
    {
        flight_status = Multirotor_Server::FlightStatus::LANDING;
    }
   
    return true;
}

bool Multirotor_Server::flightByVel_serverCB(offboard::flightByVel::Request  &req, 
        offboard::flightByVel::Response &res)
{
    res.ack = 1;
    ROS_INFO("flight by vel server cb ");
    
    flight_status = Multirotor_Server::FlightStatus::FLIGHTBYVEL;
    vel_x = req.vel_x;
    vel_y = req.vel_y;
    fly_time = req.fly_time;
    return true;
    
}

bool Multirotor_Server::flightByOffset_serverCB(offboard::flightByOffset::Request  &req, 
        offboard::flightByOffset::Response &res)
{
    res.ack = 1;
    ROS_INFO("flight by offset server cb ");
    
    flight_status = Multirotor_Server::FlightStatus::FLIGHTBYOFFSET;
    delta_x = req.delta_x;
    delta_y = req.delta_y;
    return true;
}

bool Multirotor_Server::drop_serverCB(offboard::drop::Request  &req, 
        offboard::drop::Response &res)
{
    res.ack = 1;
    ROS_INFO("drop server cb");
    flight_status = Multirotor_Server::FlightStatus::DROP;
    return true;
}

void Multirotor_Server::run()
{
    ros::Rate rate(10.0);   // 10HZ
    while (ros::ok())
    {
        TicToc tictoc;
        switch (flight_status)
        {
            case Multirotor_Server::FlightStatus::TAKEOFF:
                ROS_INFO("takeoff");
                Multirotor_Server::takeoff();
                rate.sleep();
                break;
            case Multirotor_Server::FlightStatus::LANDING:
                ROS_INFO("landing");
                Multirotor_Server::landing();
                rate.sleep();
                break;
            case Multirotor_Server::FlightStatus::FLIGHTBYVEL:
                ROS_INFO("flight by vel");
                for (int i = 0; i <(fly_time/100) ; i++)
                {
                    Multirotor_Server::flightByVel();
                    rate.sleep();
                }           
                break;
            case Multirotor_Server::FlightStatus::FLIGHTBYOFFSET:
                ROS_INFO("flight by offset");
                Multirotor_Server::flightByOffset();
                break;
            case Multirotor_Server::FlightStatus::DROP:
                ROS_INFO("drop");
                Multirotor_Server::drop();
                rate.sleep();
                break;
            case Multirotor_Server::FlightStatus::IDLE:
                ROS_INFO("idle");
                Multirotor_Server::idle();
                rate.sleep();
                break;
            default:
                ROS_INFO("default");
                Multirotor_Server::idle();
                rate.sleep();
                break;
        }
        ros::spinOnce();
        ROS_INFO("sleep time= %f",tictoc.toc());
    }
}

void Multirotor_Server::takeoff()
{
    ros::ServiceClient set_takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = nh.param<float>("fly_height", 5.0);  // 5m
    if (current_state.mode == "GUIDED" && current_state.armed){}
    else{
        ROS_ERROR("PLEASE ARM AND SWITCH TO GUIDED MODE FIRST");
        return;
    }

    if (set_takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success)
    {
        ROS_INFO("takeoff success");
        ros::Duration(5).sleep();
        flight_status = Multirotor_Server::FlightStatus::IDLE;
    }
    else
    {
        ROS_INFO("takeoff failed");
        flight_status = Multirotor_Server::FlightStatus::IDLE;
    }
}

void Multirotor_Server::landing()
{
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "LAND";

    if (current_state.mode != "LAND")
    {
        if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
        {
            ros::Duration(10.0).sleep();
            ROS_INFO("landing success");
        }
        else
        {
            ROS_INFO("landing failed");
        }
    }
    flight_status = Multirotor_Server::FlightStatus::IDLE;

}

void Multirotor_Server::flightByVel()
{

    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.header.stamp = ros::Time::now();
    vel_cmd.twist.linear.x = vel_x;
    vel_cmd.twist.linear.y = vel_y;
    vel_cmd.twist.linear.z = 0.0;
    vel_pub.publish(vel_cmd);
    flight_status = Multirotor_Server::FlightStatus::IDLE;
}

void Multirotor_Server::flightByOffset()
{
    geometry_msgs::PoseStamped pos_cmd;
    pos_cmd.header.stamp = ros::Time::now();
    pos_cmd.pose.position.x = drone_pos(0) + delta_x;
    pos_cmd.pose.position.y = drone_pos(1) + delta_y;
    pos_cmd.pose.position.z = drone_pos(2);
    pos_pub.publish(pos_cmd);
    ros::Duration(5).sleep();
    flight_status = Multirotor_Server::FlightStatus::IDLE;
}

void Multirotor_Server::drop()
{
    ros::Publisher drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    drop_cmd.channels[0] = 0;
    drop_cmd.channels[1] = 0;
    drop_cmd.channels[2] = 0;
    drop_cmd.channels[3] = 0;
    drop_cmd.channels[4] = 0;
    drop_cmd.channels[5] = 0;
    drop_cmd.channels[6] = 0;
    drop_cmd.channels[7] = 0;
    drop_pub.publish(drop_cmd);
}

void Multirotor_Server::idle()
{
    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.header.stamp = ros::Time::now();
    vel_cmd.twist.linear.x = 0.0;
    vel_cmd.twist.linear.y = 0.0;
    vel_cmd.twist.linear.z = 0.0;
    vel_pub.publish(vel_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_server");
    Multirotor_Server multirotor_server;
    multirotor_server.run();
    return 0;
}