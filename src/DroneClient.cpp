#include "ros/ros.h"
#include "offboard/flightByVel.h"
#include "offboard/flightByOffset.h"
#include "offboard/drop.h"
#include "offboard/takeoffOrLanding.h"

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"drone_client");
    ros::NodeHandle n;
    if (argc < 2)
    {
        ROS_ERROR("please set :1-takeoff 2-landing 3-flightByVel 4-flightByOffset 5-drop");
        return 1;
    }
    else
    {
        if (atoi(argv[1]) == 3)
        {
            ROS_INFO("flightByVel");
            ros::ServiceClient client = n.serviceClient<offboard::flightByVel>("flightByVel");
            offboard::flightByVel client_call;
            client_call.request.vel_x = atof(argv[2]);
            client_call.request.vel_y = atof(argv[3]);
            client_call.request.fly_time = atoi(argv[4]);
            bool result = client.call(client_call);
            if (result)
            {
                ROS_INFO("success!!!!!");
                ROS_INFO("result：%d",client_call.response.ack);
            }
            else
            {
                ROS_INFO("failed!!!!!!");
            }
        }
        else if(atoi(argv[1]) == 4)
        {
            ROS_INFO("flightByOffset");
            ros::ServiceClient client = n.serviceClient<offboard::flightByOffset>("flightByOffset");
            offboard::flightByOffset client_call;
            client_call.request.delta_x = atof(argv[2]);
            client_call.request.delta_y = atof(argv[3]);
            bool result = client.call(client_call);
            if (result)
            {
                ROS_INFO("success!!!!!");
                ROS_INFO("result：%d",client_call.response.ack);
            }
            else
            {
                ROS_INFO("failed!!!!!!");
            }
        }
        else if(atoi(argv[1]) == 5)
        {
            ROS_INFO("drop");
            ros::ServiceClient client = n.serviceClient<offboard::drop>("drop");
            offboard::drop client_call;
            client_call.request.drop = true;
            bool result = client.call(client_call);
            if (result)
            {
                ROS_INFO("success!!!!!");
                ROS_INFO("result：%d",client_call.response.ack);
            }
            else
            {
                ROS_INFO("failed!!!!!!");
            }
        }
        else if(atoi(argv[1]) == 1)
        {
            ROS_INFO("takeoff");
            ros::ServiceClient client = n.serviceClient<offboard::takeoffOrLanding>("takeoffOrLanding");
            offboard::takeoffOrLanding client_call;
            client_call.request.takeoffOrLanding = 1;
            bool result = client.call(client_call);
            if (result)
            {
                ROS_INFO("success!!!!!");
                ROS_INFO("result：%d",client_call.response.ack);
            }
            else
            {
                ROS_INFO("failed!!!!!!");
            }
        }
        else if(atoi(argv[1]) == 2)
        {
            ROS_INFO("landing");
            ros::ServiceClient client = n.serviceClient<offboard::takeoffOrLanding>("takeoffOrLanding");
            offboard::takeoffOrLanding client_call;
            client_call.request.takeoffOrLanding = 2;
            bool result = client.call(client_call);
            if (result)
            {
                ROS_INFO("success!!!!!");
                ROS_INFO("result：%d",client_call.response.ack);
            }
            else
            {
                ROS_INFO("failed!!!!!!");
            }
        }
    }

    return 0;
}