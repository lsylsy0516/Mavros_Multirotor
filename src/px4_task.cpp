#include "px4_task.h"
using namespace std;
Multirotor::Multirotor()
{
    state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, &Multirotor::state_cb,this);
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("mavros/local_position/pose", 10, &Multirotor::position_cb,this);
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                ("mavros/local_position/velocity_body", 10, &Multirotor::velocity_cb,this);
    drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>
                                    ("mavros/rc/override", 10);
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                                    ("mavros/setpoint_velocity/cmd_vel", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");
}

Multirotor::~Multirotor()
{
}

// callback function

void Multirotor::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    ROS_INFO("mode = %s",current_state.mode.c_str());
    ROS_INFO("current_state.armed = %d",current_state.armed);
}

void Multirotor::position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pos[0] = msg->pose.position.x ;
    drone_pos[1] = msg->pose.position.y ;
    drone_pos[2] = msg->pose.position.z ;
    ROS_INFO("drone_pos: x,y,z= %f,%f,%f",drone_pos[0],drone_pos[1],drone_pos[2]);
}

void Multirotor::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    drone_vel[0] = msg->twist.linear.x ;
    drone_vel[1] = msg->twist.linear.y ;
    drone_vel[2] = msg->twist.linear.z ;
    ROS_INFO("drone_vel: x,y,z= %f,%f,%f",drone_vel[0],drone_vel[1],drone_vel[2]);
}

// move function

void Multirotor::flytopoint(Eigen::Vector3d point)
{
    // 使用PID控制器计算期望速度
    pos_pid_x.set_target(point[0]); 
    pos_pid_y.set_target(point[1]);
        // 位置
    float target_vel_x = pos_pid_x.update(drone_pos[0]); 
    float target_vel_y = pos_pid_y.update(drone_pos[1]);
        // 转换为速度环的目标速度
    vel_pid_x.set_target(target_vel_x); 
    vel_pid_y.set_target(target_vel_y);
        // 速度环
    float output_vel_x = vel_pid_x.update(drone_vel[0]) + drone_vel[0];
    float output_vel_y = vel_pid_y.update(drone_vel[1]) + drone_vel[1];
        // 发布期望速度
    geometry_msgs::TwistStamped velTarget;
    velTarget.header.stamp = ros::Time::now();
    velTarget.twist.linear.x = output_vel_x;
    velTarget.twist.linear.y = output_vel_y;
    velTarget.twist.linear.z = 0;
    vel_pub.publish(velTarget);
}

void Multirotor::takeoff()
{
    geometry_msgs::PoseStamped PositionTarget;
    PositionTarget.header.stamp = ros::Time::now();
    PositionTarget.pose.position.x = drone_pos[0];
    PositionTarget.pose.position.y = drone_pos[1];
    PositionTarget.pose.position.z = fly_height;
    pos_pub.publish(PositionTarget);
    ROS_INFO("takeoff x,y,z= %f,%f,%f",PositionTarget.pose.position.x,PositionTarget.pose.position.y,PositionTarget.pose.position.z);
}

void Multirotor::land()
{
    geometry_msgs::PoseStamped PositionTarget;
    PositionTarget.header.stamp = ros::Time::now();
    PositionTarget.pose.position.x = drone_pos[0];
    PositionTarget.pose.position.y = drone_pos[1];
    PositionTarget.pose.position.z = 0;
    pos_pub.publish(PositionTarget);
    ROS_INFO("land x,y,z= %f,%f,%f",PositionTarget.pose.position.x,PositionTarget.pose.position.y,PositionTarget.pose.position.z);
}

void Multirotor::drop_bottle()
{
    mavros_msgs::OverrideRCIn drop_cmd;
    drop_cmd.channels[0] = 0;
    drop_cmd.channels[1] = 0;
    drop_cmd.channels[2] = 0;
    drop_cmd.channels[3] = 0;
    drop_cmd.channels[4] = 0;
    drop_cmd.channels[5] = 0;
    drop_cmd.channels[6] = 0;
    drop_cmd.channels[7] = 1500;
    drop_pub.publish(drop_cmd);
    ROS_INFO("drop");
}

// set mode function

void Multirotor::setoffboardmode()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    // 设置offboard 模式，并设置为armed

        if( current_state.mode != "OFFBOARD")
        {
            if( set_mode_client.call(offb_set_mode) && // 设置为 offboard模式
                    offb_set_mode.response.mode_sent) // 接收成功
            {
                ROS_INFO("Offboard enabled");
            }
        }
        else
        {
            if( !current_state.armed )
            {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
            }
        }
}

// init and run function

void Multirotor::init()
{
    float pos_max, pos_kp, pos_ki, pos_kd, pos_range_rough, pos_range_fine;
    float vel_max, vel_kp, vel_ki, vel_kd, vel_range_rough, vel_range_fine;
    // Load pos_pid parameters with default values
    nh.param<float>("pos_max", pos_max, 0.5);
    nh.param<float>("pos_kp", pos_kp, 0.5);
    nh.param<float>("pos_ki", pos_ki, 0.0);
    nh.param<float>("pos_kd", pos_kd, 0.0);
    nh.param<float>("pos_range_rough", pos_range_rough, 1.0);
    nh.param<float>("pos_range_fine", pos_range_fine, 0.3);

    // Load vel_pid parameters with default values
    nh.param<float>("vel_max", vel_max, 0.5);
    nh.param<float>("vel_kp", vel_kp, 0.5);
    nh.param<float>("vel_ki", vel_ki, 0.0);
    nh.param<float>("vel_kd", vel_kd, 0.0);
    nh.param<float>("vel_range_rough", vel_range_rough, 1.0);
    nh.param<float>("vel_range_fine", vel_range_fine, 0.3);

    // Initialize PID controllers
    pos_pid_x = PID(pos_kp, pos_ki, pos_kd, pos_max, pos_range_rough, pos_range_fine);
    pos_pid_y = PID(pos_kp, pos_ki, pos_kd, pos_max, pos_range_rough, pos_range_fine);
    vel_pid_x = PID(vel_kp, vel_ki, vel_kd, vel_max, vel_range_rough, vel_range_fine);
    vel_pid_y = PID(vel_kp, vel_ki, vel_kd, vel_max, vel_range_rough, vel_range_fine);
    
    nh.param<float>("fly_height",fly_height,0.5);
    nh.param<float>("min_dis",min_dis,0.1);
    // set task_points
    task_points.push_back(Eigen::Vector3d(drone_pos[0],drone_pos[1],fly_height)); // 起飞点
    task_points.push_back(Eigen::Vector3d(30, 0,fly_height)); // 任务点1
    task_points.push_back(Eigen::Vector3d(30,-4,fly_height)); // 任务点2
    task_points.push_back(Eigen::Vector3d(50,-4,fly_height)); // 任务点3
    task_points.push_back(Eigen::Vector3d(50, 4,fly_height)); // 任务点4

    ros::Rate rate(20); 
    switchflag = 0;
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Multirotor Inited!");
}

void Multirotor::run()
{
    ros::Rate rate(20); //接收和发布频率
    
    // Takeoff
    // Pid module init
    pos_pid_x.init();
    pos_pid_y.init();
    vel_pid_x.init();
    vel_pid_y.init();

    while(switchflag == 0)
    {
        setoffboardmode();
        takeoff();
        rate.sleep();
        ros::spinOnce();

        double distance = abs(drone_pos[2]-fly_height);
        if (distance < 0.01){
            switchflag = 1;
            pos_pid_x.reset();
            pos_pid_y.reset();
            vel_pid_x.reset();
            vel_pid_y.reset();
        }
    }
    
    // wait 1s more
    sleep(1.0);
    // reset the switchflag
    switchflag = 0;
    int point_num = 0;

    
    // task_points
    while (switchflag == 0)
    {
        setoffboardmode();
        flytopoint(task_points[point_num]);
        rate.sleep();
        ros::spinOnce();

        // calc the distance to the task point
        double distance = sqrt(pow((drone_pos[0]-task_points[point_num][0]),2)+pow((drone_pos[1]-task_points[point_num][1]),2));
        if (distance < min_dis)
        {
            point_num++;
            if (point_num == 3)   {
                switchflag = 1;
                pos_pid_x.reset();
                pos_pid_y.reset();
                vel_pid_x.reset();
                vel_pid_y.reset();
            }
            // wait 1s more
            sleep(1.0);
        }
    }

    // drop
    int count = 0;
    while (count < 60) // 3s
    {
        drop_bottle();
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    
    // wait 1s more
    sleep(1.0);
    // reset the switchflag
    switchflag = 0;
    pos_pid_x.reset();
    pos_pid_y.reset();
    vel_pid_x.reset();
    vel_pid_y.reset();

    // return to the takeoff point
    while (switchflag == 0)
    {
        flytopoint(task_points[0]);
        ros::spinOnce();
        rate.sleep();
        // calc the distance to the task point
        double distance = sqrt(pow((drone_pos[0]-task_points[0][0]),2)+pow((drone_pos[1]-task_points[0][1]),2));
        if (distance < min_dis)
        {
            switchflag = 2;
        }
    }

    count = 0;
    // land 
    while (count < 100) // 5s
    {
        land();
        ros::spinOnce();
        rate.sleep();
        count++;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_task");
    Multirotor multirotor;
    multirotor.init();
    multirotor.run();
    return 0;
}