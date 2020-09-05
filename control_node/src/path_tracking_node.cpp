#include "control_node/path_tracking_node.h"
#include "fstream"

PathTrackingNode::PathTrackingNode(){
    ros::NodeHandle nh;
    //读取config文件，目前还没有使用
    std::string path = "config file path";
    if(LoadConfig(path)){
        node_state = State::HAVE_CONFIG;
    }else{
        ROS_WARN("path tracking node: Config failed");
    }
    //subscribe and publish
    cmd_sub = nh.subscribe<robot_msgs::Cmd>("path_tracking_cmd",10,&PathTrackingNode::CmdCallback,this);
    pose_sub = nh.subscribe<nav_msgs::Odometry>("odom",10,&PathTrackingNode::RobotPoseCallback,this);
    path_sub = nh.subscribe<nav_msgs::Path>("path",10,&PathTrackingNode::PathCallback,this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    state_pub = nh.advertise<std_msgs::String>("path_tracking_state",10);
    //state_pub_thread = new std::thread(&PathTrackingNode::StatePub,this);
    path_tracking = new std::thread(&PathTrackingNode::PathTrackingThread,this);
    //初始化tracking参数
    tracking.target_uint  =  1.0;
    tracking.pathFollow = true;
    tracking.pathFollowEx = false;
    get_path = false;
    max_v = 1.0;
    target_pose.position.x = 0;
    target_pose.position.y = 0;
}


//path tracking函数，单独开了一个线程
void PathTrackingNode::PathTrackingThread(){
    ROS_INFO("start Tracking thread");
    ros::NodeHandle nh;
    ros::Rate loop(10);
    while(ros::ok()){
        //等待规划路径
        while(ros::ok()){
            if(get_path && node_state == State::RUNNING){
                break;
            }
            loop.sleep();
            ros::spinOnce();
        }
        ROS_INFO("path tracking start");
        get_path = false;
        ros::Rate loop2(100);

        double start_time = ros::WallTime::now().toSec();
        geometry_msgs::Twist speed;
        //path follow.首先到达目标点
        impl::pt2D me = PoseTospt2D(robot_pose);
        impl::pt2D target = PoseTospt2D(target_pose);
        while(impl::norm2(target-me) > 0.14 && node_state == State::RUNNING ){
            //得到规划路径，计算期望速度,获取机器人角度
            impl::pt2D robot_speed;
            double time = ros::WallTime::now().toSec() - start_time; 
            double yaw = QuaternionToYaw(robot_pose.orientation);
            robot_speed = tracking(time, {robot_pose.position.x, robot_pose.position.y}, 1.0);
            robot_speed = impl::fromVxy2VW(yaw,robot_speed);
            //发布速度指令
            speed.linear.x = robot_speed.x;
            speed.linear.y = 0;
            speed.linear.z = 0;
            speed.angular.x = 0.0; 
            speed.angular.y = 0.0;
            speed.angular.z = robot_speed.y;
            twist_pub.publish(speed);
            me = PoseTospt2D(robot_pose);
            StatePub("running");
            loop2.sleep();
            ros::spinOnce();
        }
        //纠正角度
        double target_yaw = QuaternionToYaw(target_pose.orientation);
        double yaw = QuaternionToYaw(robot_pose.orientation);
        while(fabs(target_yaw - yaw) > 0.05 && node_state == State::RUNNING){
            speed.linear.x = 0;
            speed.linear.y = 0;
            speed.linear.z = 0;
            speed.angular.x = 0.0; 
            speed.angular.y = 0.0;
            speed.angular.z = 0.6 * (((target_yaw - yaw) > 0) ? 1 : -1);
            yaw = QuaternionToYaw(robot_pose.orientation);
            twist_pub.publish(speed);
            StatePub("running");
            loop2.sleep();
            ros::spinOnce();
        }
        //小车停 记录本次target
        speed.angular.z = 0.0;
        twist_pub.publish(speed);
        StatePub("success");
        ROS_INFO("mission complete, path tracking node standby");
        node_state = State::PAUSED;
        ros::spinOnce();
    }
}

//记录机器人位姿
void PathTrackingNode::RobotPoseCallback(const nav_msgs::OdometryConstPtr &msg){
    robot_pose = msg->pose.pose;
}

//发布Tracking状态
void PathTrackingNode::StatePub(std::string state){
    for(int i = 0; i < 2; i++){
        std_msgs::String tracking_state;
        tracking_state.data = state;
        state_pub.publish(tracking_state);

    }
}


//将ros类型的path转化为pline
void PathTrackingNode::PathCallback(const nav_msgs::PathConstPtr &msg){
    int i;
    //判断是否为新的目标点
    double distance;
    distance = impl::norm2(PoseTospt2D(msg->poses.back().pose) - PoseTospt2D(last_target_pose));
    if(distance == 0){
        return ;
    }
    std::vector<double> temp_x,temp_y,temp_t;
    //计算所有路点的时间t
    impl::pt2D pose,last_pose;
    last_pose = PoseTospt2D(msg->poses.at(0).pose);
    double last_t = 0.0, t = 0.0; 
    for(i = 0; i < msg->poses.size(); i++){
        //安装学长的公式计算t
        pose = PoseTospt2D(msg->poses.at(i).pose);
        t = last_t + impl::norm2(pose - last_pose)/max_v; //t(k+1) = t(k) + distance/v;
        last_t = t;
        last_pose = pose;
        temp_x.push_back(msg->poses.at(i).pose.position.x);
        temp_y.push_back(msg->poses.at(i).pose.position.y);
        temp_t.push_back(t);
        
    }
    //初始化pline
    path.setpoints(temp_t,temp_x,temp_y);
    tracking.pline = &path;
    target_pose = msg->poses.back().pose;
    get_path = true;
    last_target_pose = target_pose;
}

void PathTrackingNode::Reset(){
    geometry_msgs::Pose reset_pose;
    get_path = false;
    last_target_pose = reset_pose;
    geometry_msgs::Twist speed;
    twist_pub.publish(speed);

}

//一些功能函数
impl::pt2D  PathTrackingNode::PoseTospt2D(const geometry_msgs::Pose pose){
    impl::pt2D temp;
    temp.x = pose.position.x;
    temp.y = pose.position.y;
    return temp;

}

double PathTrackingNode::QuaternionToYaw(const geometry_msgs::Quaternion quaterion){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaterion, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    return yaw;
}

bool PathTrackingNode::LoadConfig(std::string file){
    ROS_INFO("path tracking node: load config");
    //TODO 应该在这里读取配置文件，目前只初始化更新频率和节点名称；
    update_frequence = 10;
    node_name = "path tracking node";
}

void PathTrackingNode::UpdateState(){
    ros::NodeHandle nh;
    ros::Rate loop(update_frequence);
    double start_time = ros::Time().now().toSec();
    ROS_INFO("path tracking node: state thread start");
    while(nh.ok() && node_state!=State::EXIT){
        robot_msgs::PerceptionNodeMsg msg;
        //TODO 目前获取的是ros秒可能需要进一步处理
        msg.time = ros::Time().fromSec(ros::Time().now().toSec() - start_time);
        msg.node_name = "driver_node";
        msg.state.state = (uint8_t)node_state;
        //msg.state.pose = 
        state_pub.publish(msg);
        loop.sleep();
        ros::spinOnce();
    }
}


State PathTrackingNode::Start(){
    if(node_state != State::RUNNING){
        ROS_INFO("path tracking node get command");
        ROS_INFO("path tracking node running");
    }
    return State::RUNNING;
}
//Stop需要reset参数
State PathTrackingNode::Stop(){
    if(node_state != State::STOP){
        ROS_INFO("path tracking node get command");
        ROS_INFO("path tracking node stop");
    }
    return State::STOP;
}
//退出需要清理线程
State PathTrackingNode::Exit(){
    if(node_state != State::EXIT){
        ROS_INFO("path tracking node get command");
        ROS_INFO("path tracking node exit");
    }
    state_pub.shutdown();
    cmd_sub.shutdown();
    return State::EXIT;
}
//pause和resume不需要对数据进行处理
State PathTrackingNode::Pause(){
    if(node_state!=State::RUNNING)
        return node_state;
    if(node_state != State::PAUSED){
        ROS_INFO("path tracking node get command");
        ROS_INFO("path tracking node pause");
        Reset();
    }
    return State::PAUSED;
}
State PathTrackingNode::Resume(){
    if(node_state!=State::PAUSED)
        return node_state;
    if(node_state != State::RUNNING){
        ROS_INFO("path tracking node get command");
        ROS_INFO("path tracking node resume");
    }
    return State::RUNNING;
}

void PathTrackingNode::CmdCallback(const robot_msgs::CmdConstPtr &msg){
    switch(msg->cmd){
        case (int)Cmd::START : node_state = Start();  break;
        case (int)Cmd::PAUSE : node_state = Pause();  break;
        case (int)Cmd::STOP  : node_state = Stop();   break;
        case (int)Cmd::RESUME: node_state = Resume(); break;
        case (int)Cmd::EXIT  : node_state = Exit();   break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracking_node");
    PathTrackingNode path_tracking_node;
    //path_tracking_node.UpdateState();
    ros::Rate loop(100);
    while(ros::ok()){
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}


