#include "px4_task.h"
using namespace std;

float calc_distance3d(Eigen::Vector3d a, Eigen::Vector3d b)
{
    return sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2) + pow((a[2] - b[2]), 2));
}

float calc_distance2d(Eigen::Vector3d a, Eigen::Vector3d b)
{
    return sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2));
}

Multirotor::Multirotor()
{
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Multirotor::state_cb, this);
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Multirotor::position_cb, this);
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 10, &Multirotor::velocity_cb, this);
    drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

Multirotor::~Multirotor()
{
}

// callback function

void Multirotor::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void Multirotor::position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // drone_pos ENU
    drone_pos[0] = msg->pose.position.x;
    drone_pos[1] = msg->pose.position.y;
    drone_pos[2] = msg->pose.position.z;
}

void Multirotor::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    drone_vel[0] = msg->twist.linear.x;
    drone_vel[1] = msg->twist.linear.y;
    drone_vel[2] = msg->twist.linear.z;
}

// move function

// 使用位置环和速度环控制无人机到达目标点
void Multirotor::flytopoint(Eigen::Vector3d point)
{
    // 使用PID控制器计算期望速度
    vel_pid_x.set_target(point[0]);
    vel_pid_y.set_target(point[1]);
    vel_pid_z.set_target(point[2]);
    // 位置
    float target_vel_x = vel_pid_x.update(drone_pos[0]);
    float target_vel_y = vel_pid_y.update(drone_pos[1]);
    float target_vel_z = vel_pid_z.update(drone_pos[2]);
    // 转换为速度环的目标速度
    acc_pid_x.set_target(target_vel_x);
    acc_pid_y.set_target(target_vel_y);
    acc_pid_z.set_target(target_vel_z);
    // 速度环
    float output_vel_x = acc_pid_x.update(drone_vel[0]) + drone_vel[0];
    float output_vel_y = acc_pid_y.update(drone_vel[1]) + drone_vel[1];
    float output_vel_z = acc_pid_z.update(drone_vel[2]) + drone_vel[2];
    // 发布期望速度
    geometry_msgs::TwistStamped velTarget;
    velTarget.header.stamp = ros::Time::now();
    velTarget.twist.linear.x = output_vel_x;
    velTarget.twist.linear.y = output_vel_y;
    velTarget.twist.linear.z = output_vel_z;
    vel_pub.publish(velTarget);

    // 输出当前和目标状态
    ROS_INFO("Current Position: [%f, %f, %f]", drone_pos[0], drone_pos[1], drone_pos[2]);
    ROS_INFO("Target  Position: [%f, %f, %f]", point[0], point[1], point[2]);
    ROS_INFO("Current Velocity: [%f, %f, %f]", drone_vel[0], drone_vel[1], drone_vel[2]);
    ROS_INFO("Desired Velocity: [%f, %f, %f]", output_vel_x, output_vel_y, output_vel_z);
    ROS_INFO("------------------------------");
}

void Multirotor::takeoff()
{
    // POSITION
    geometry_msgs::PoseStamped PositionTarget;
    PositionTarget.header.stamp = ros::Time::now();
    PositionTarget.pose.position.x = drone_pos[0]; // 无人机当前位置
    PositionTarget.pose.position.y = drone_pos[1];
    PositionTarget.pose.position.z = fly_height;
    pos_pub.publish(PositionTarget);
    // 输出当前和目标状态
    ROS_INFO("------------------------------");
    ROS_INFO("drone_pos: x,y,z= %f,%f,%f",drone_pos[0],drone_pos[1],drone_pos[2]);
    ROS_INFO("takeoff  : x,y,z= %f,%f,%f",PositionTarget.pose.position.x,PositionTarget.pose.position.y,PositionTarget.pose.position.z);
    // ros::ServiceClient set_takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    // mavros_msgs::CommandTOL takeoff;
    // takeoff.request.altitude = 5;
    // if(current_state.armed && !takeoff.response.success){
    //     if(set_takeoff_client.call(takeoff) && takeoff.response.success)
    //         ROS_INFO("------------------------------");
    //         ROS_INFO("Vehicle takeoff");
    //         ROS_INFO("drone_pos: x,y,z= %f,%f,%f",drone_pos[0],drone_pos[1],drone_pos[2]);
    // }
    // 据说很重要
    // sleep(10);
}

void Multirotor::land()
{
    // POSITION
    geometry_msgs::PoseStamped PositionTarget;
    PositionTarget.header.stamp = ros::Time::now();
    PositionTarget.pose.position.x = drone_pos[0];
    PositionTarget.pose.position.y = drone_pos[1];
    PositionTarget.pose.position.z = 0;
    pos_pub.publish(PositionTarget);
    // 输出当前和目标状态
    ROS_INFO("------------------------------");
    ROS_INFO("drone_pos: x,y,z= %f,%f,%f", drone_pos[0], drone_pos[1], drone_pos[2]);
    ROS_INFO("landing  :  x,y,z= %f,%f,%f", PositionTarget.pose.position.x, PositionTarget.pose.position.y, PositionTarget.pose.position.z);
}

void Multirotor::drop_bottle()
{
    // 使用参数服务器来更新 舵机的占空比 (已弃用)
    // nh.setParam("servo_position",SERVO_OPEN); // 占空比
    // POSITION
    mavros_msgs::OverrideRCIn drop_cmd;
    drop_cmd.channels[0] = 0;
    drop_cmd.channels[1] = 0;
    drop_cmd.channels[2] = 0;
    drop_cmd.channels[3] = 0;
    drop_cmd.channels[4] = 0;
    drop_cmd.channels[5] = 0;
    drop_cmd.channels[6] = 0;
    drop_cmd.channels[7] = 0;
    drop_pub.publish(drop_cmd);
    ROS_INFO("droping bottle");
    Eigen::Vector3d current_pos = Eigen::Vector3d(drone_pos[0],drone_pos[1],fly_height);
    flytopoint(current_pos);
    // ROSINFO
}

// set mode function

void Multirotor::settakeoffmode()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 设置offboard 模式，并设置为armed
    if( current_state.mode != "OFFBOARD")
    {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
            ROS_INFO("offboard enabled");
    }
    else
    {
        if( !current_state.armed )
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed");
        }
    }
}

void Multirotor::setoffboardmode()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 设置offboard 模式，并设置为armed
    if (current_state.mode != "OFFBOARD")
    {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            ROS_INFO("Offboard enabled");
    }
    else
    {
        if (!current_state.armed)
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed");
        }
    }
}

// init and run function
void Multirotor::pid_init()
{
    std::vector<std::string> axes = {"x", "y", "z"};
    std::vector<std::string> controls = {"vel", "acc"};

    std::vector<PID> vel_pid_ctrs(axes.size());
    std::vector<PID> acc_pid_ctrs(axes.size());

    for (size_t axis_idx = 0; axis_idx < axes.size(); ++axis_idx)
    {
        for (size_t control_idx = 0; control_idx < controls.size(); ++control_idx)
        {
            // Init axis and control
            const std::string &axis = axes[axis_idx];
            const std::string &control = controls[control_idx];

            // Load PID parameters with default values
            float output_max, kp, ki, kd, range_rough, range_fine;
            nh.param<float>(axis + "/" + control + "/kp", kp, 0.0);
            nh.param<float>(axis + "/" + control + "/ki", ki, 0.0);
            nh.param<float>(axis + "/" + control + "/kd", kd, 0.0);
            nh.param<float>(axis + "/" + control + "/outputmax", output_max, 0.0);
            nh.param<float>(axis + "/" + control + "/range_rough", range_rough, 0.0);
            nh.param<float>(axis + "/" + control + "/range_fine", range_fine, 0.0);

            // Initialize PID controllers and store them in the vectors
            PID pid_controller(kp, ki, kd, output_max, range_rough, range_fine);
            if (control == "vel")
            {
                vel_pid_ctrs[axis_idx] = pid_controller;
            }
            else if (control == "acc")
            {
                acc_pid_ctrs[axis_idx] = pid_controller;
            }
        }
    }

    // Load PID Class
    vel_pid_x = vel_pid_ctrs[0];
    vel_pid_y = vel_pid_ctrs[1];
    vel_pid_z = vel_pid_ctrs[2];
    acc_pid_x = acc_pid_ctrs[0];
    acc_pid_y = acc_pid_ctrs[1];
    acc_pid_z = acc_pid_ctrs[2];

    // Pid module init
    vel_pid_x.init();
    vel_pid_y.init();
    vel_pid_z.init();
    acc_pid_x.init();
    acc_pid_y.init();
    acc_pid_z.init();
}

void Multirotor::pid_reset()
{
    vel_pid_x.reset();
    vel_pid_y.reset();
    vel_pid_z.reset();
    acc_pid_x.reset();
    acc_pid_y.reset();
    acc_pid_z.reset();
}

void Multirotor::init()
{
    ros::Rate rate(20);
    // Load PID parameters with default values
    pid_init();
    nh.setParam("servo_position", SERVO_CLOSE); // 初始化占空比
    nh.param<float>("fly_height", fly_height, 0.5);
    nh.param<float>("min_dis", min_dis, 0.1);

    // Get Drone_pos
    for (int i = 0; i < 20; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Set Task_points
    task_points.push_back(Eigen::Vector3d(drone_pos[0],drone_pos[1],fly_height)); // takeoff point
    task_points.push_back(Eigen::Vector3d( 0,10,fly_height)); // task point 1
    // task_points.push_back(Eigen::Vector3d(-4,30,fly_height)); // task point 2
    // task_points.push_back(Eigen::Vector3d(-4,50,fly_height)); // task point 3
    // task_points.push_back(Eigen::Vector3d( 4,50,fly_height)); // task point 4

    // For Test
    // task_points.push_back(Eigen::Vector3d(drone_pos[0], drone_pos[1], fly_height));
    // ROS_INFO("task_point:%f,%f,%f",drone_pos[0], drone_pos[1], fly_height);
    // task_points.push_back(Eigen::Vector3d(drone_pos[0], drone_pos[1], fly_height));
    // task_points.push_back(Eigen::Vector3d(drone_pos[0],drone_pos[1],fly_height));
    // task_points.push_back(Eigen::Vector3d(drone_pos[0],drone_pos[1],fly_height));

    // Get Connected
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // Set SwitchFlag
    switchflag = 0;

    ROS_INFO("Multirotor Inited!");
}

void Multirotor::run()
{
    ros::Rate rate(20);

    // takeoff
    while (switchflag == 0)
    {
        settakeoffmode();
        takeoff(); // POSITION

        rate.sleep();
        ros::spinOnce();

        double distance1d = abs(drone_pos[2]-task_points[0][2]);
        
        if (distance1d < min_dis)
        {
            ROS_INFO("distance : %f", distance1d);
            // reset the switchflag
            switchflag = 1;
            // reset pid
            pid_reset();
            // wait 1s more
            sleep(1.0);
        }
        else
        {
            ROS_INFO("distance : %f", distance1d);
        }
    }

    // reset the switchflag
    switchflag = 0;

    // task_points
    int point_num = 1;
    while (switchflag == 0)
    {
        setoffboardmode();
        flytopoint(task_points[point_num]); // VELOCILITY


        double distance2d = calc_distance2d(drone_pos, task_points[point_num]);
        if (distance2d < min_dis)
        {
            // reset pid
            pid_reset();
            sleep(1.0);
            point_num++;
            ROS_INFO("ARRIVED POINT:%d", point_num);
            if (point_num == task_points.size())
                switchflag = 1;
        }
        else
        {
            ROS_INFO("distance :%f", distance2d);
        }
        rate.sleep();
        ros::spinOnce();
    }

    // reset the switchflag
    switchflag = 0;

    // drop
    int count = 0;
    while (switchflag == 0)
    {
        drop_bottle();

        ros::spinOnce();
        rate.sleep();

        count++;
        if (count == 90) // 3s
        {
            // reset pid
            pid_reset();
            // reset the switchflag
            switchflag = 1;
            // wait 1s more
            sleep(1.0);
        }
    }

    // reset the switchflag
    switchflag = 0;

    // return to the takeoff point
    while (switchflag == 0)
    {
        setoffboardmode();
        flytopoint(task_points[0]);

        ros::spinOnce();
        rate.sleep();

        double distance2d = calc_distance2d(drone_pos, task_points[0]);
        if (distance2d < min_dis)
        {
            switchflag = 2;
        }
        else
        {
            ROS_INFO("distance : %f", distance2d);
        }
    }

    // reset the switchflag
    switchflag = 0;

    // land
    count = 0;
    while (switchflag == 0)
    {
        land();

        ros::spinOnce();
        rate.sleep();
        count++;
        if (count == 60) // 3s
        {
            // reset pid
            pid_reset();
            // reset the switchflag
            switchflag = 1;
            // wait 1s more
            sleep(1.0);
        }
    }

    // finish
    ROS_INFO("------------------------------");
    ROS_INFO("task finished!!");
    ROS_INFO("------------------------------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_task");
    Multirotor multirotor;
    multirotor.init();
    multirotor.run();
    return 0;
}