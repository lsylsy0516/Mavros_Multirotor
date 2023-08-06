#include <px4_task.h>

Multirotor::Multirotor()
{
    state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, &Multirotor::state_cb,this);
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("mavros/local_position/pose", 10, &Multirotor::position_cb,this);
    drop_pub = nh.advertise<mavros_msgs::OverrideRCIn>
                                    ("mavros/rc/override", 10);
    move_pub = nh.advertise<geometry_msgs::PoseStamped>
                                    ("mavros/setpoint_position/local", 10);
}

Multirotor::~Multirotor()
{
}

void Multirotor::init()
{
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // 参数初始化
        //Read the takeoff point
    for(int i=0;i<5;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }
    nh.param<float>("fly_height",fly_height,0.5);
    nh.param<float>("min_dis",min_dis,0.1);
    task_points[0] = Eigen::Vector3d(drone_pos[0],drone_pos[1],fly_height); // 起飞点
    task_points[1] = Eigen::Vector3d(30, 0,fly_height); // 任务点1
    task_points[2] = Eigen::Vector3d(30,-4,fly_height); // 任务点2
    task_points[3] = Eigen::Vector3d(50,-4,fly_height); // 任务点3
    task_points[4] = Eigen::Vector3d(50, 4,fly_height); // 任务点4
    

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    switchflag = 0;
}

void Multirotor::run()
{
    while(switchflag == 0)
    {
        takeoff();
        rate.sleep();
        ros::spinOnce();
        // calc the distance to the task point
        distance = sqrt(pow((drone_pos[0]-task_points[1][0]),2)+pow((drone_pos[1]-task_points[1][1]),2));
        if (distance < min_dis)
        {
            switchflag = 1;
        }
    }

    // wait 1s more
    sleep(1.0);
    // reset the switchflag
    switchflag = 0;
    int point_num = 1;
    
    while (switchflag == 0)
    {
        flytopoint(task_points[point_num]);
        // calc the distance to the task point
        distance = sqrt(pow((drone_pos[0]-task_points[point_num][0]),2)+pow((drone_pos[1]-task_points[point_num][1]),2));
        if (distance < min_dis)
        {
            point_num++;
            if (point_num == 5)
            {
                switchflag = 1;
            }
            // wait 1s more
            sleep(1.0);
        }
        rate.sleep();
        ros::spinOnce();
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

    // return to the takeoff point
    while (switchflag == 1)
    {
        flytopoint(task_points[0]);
        ros::spinOnce();
        rate.sleep();
        // calc the distance to the task point
        distance = sqrt(pow((drone_pos[0]-task_points[0][0]),2)+pow((drone_pos[1]-task_points[0][1]),2));
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

void Multirotor::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void Multirotor::position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pos = Eigen::Vector3f(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void Multirotor::flytopoint(Eigen::Vector3d point)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.coordinate_frame =PositionTarget.FRAME_LOCAL_NED;
    pos_setpoint.position.x = point[0];
    pos_setpoint.position.y = point[1];
    pos_setpoint.position.z = point[2];
    pos_setpoint.type_mask =PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
    move_pub.publish(pos_setpoint);
    ROS_INFO("fly to point: [%f, %f, %f]",point[0],point[1],point[2]);
}

void Multirotor::takeoff()
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.coordinate_frame =PositionTarget.FRAME_LOCAL_NED;
    pos_setpoint.position.x = task_points[0][0];
    pos_setpoint.position.y = task_points[0][1];
    pos_setpoint.position.z = task_points[0][2];
    pos_setpoint.type_mask =PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
    move_pub.publish(pos_setpoint);
    ROS_INFO("takeoff");
}

void Multirotor::land()
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.coordinate_frame =PositionTarget.FRAME_LOCAL_NED;
    pos_setpoint.position.x = drone_pos[0];
    pos_setpoint.position.y = drone_pos[1];
    pos_setpoint.position.z = 0;
    pos_setpoint.type_mask =PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
    move_pub.publish(pos_setpoint);
    ROS_INFO("land");
}


void drop_bottle()
{
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_task");
    Multirotor multirotor;
    multirotor.init();
    multirotor.run();
    return 0;
}