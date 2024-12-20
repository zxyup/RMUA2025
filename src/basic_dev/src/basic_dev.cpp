#ifndef _BASIC_DEV_CPP_
#define _BASIC_DEV_CPP_

#include "basic_dev.hpp"

int main(int argc, char** argv)
{
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    ros::init(argc, argv, "basic_dev"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    BasicDev go(&n);
    go.takeoffDrone();
    ros::Rate loop_rate(10);
 
    while (ros::ok())
    {
        go.gostraight();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}

BasicDev::BasicDev(ros::NodeHandle *nh)
{  
    //创建图像传输控制句柄
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    front_left_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    velcmd.twist.angular.z = -0.001;//z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 2; //x方向线速度(m/s)
    velcmd.twist.linear.y = 0;//y方向线速度(m/s)
    velcmd.twist.linear.z = -0.05; //z方向线速度(m/s)

    pwm_cmd.rotorPWM0 = 0.1;
    pwm_cmd.rotorPWM1 = 0.1;
    pwm_cmd.rotorPWM2 = 0.1;
    pwm_cmd.rotorPWM3 = 0.1;

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&BasicDev::pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&BasicDev::gps_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&BasicDev::imu_cb, this, std::placeholders::_1));//imu数据
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&BasicDev::lidar_cb, this, std::placeholders::_1));//imu数据
    front_left_view_suber = it->subscribe("airsim_node/drone_1/front_left/Scene", 1, std::bind(&BasicDev::front_left_view_cb, this,  std::placeholders::_1));
    front_right_view_suber = it->subscribe("airsim_node/drone_1/front_right/Scene", 1, std::bind(&BasicDev::front_right_view_cb, this,  std::placeholders::_1));
    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    //通过publisher实现对无人机的控制
    vel_publisher = nh->advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("airsim_node/drone_1/rotor_pwm_cmd", 1);
    
    // takeoff_client.call(takeoff); //起飞
    // land_client.call(land); //降落
    reset_client.call(reset); //重置

    //发布处理后的点云
    point_cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("lidar_points_with_axes", 1);

}

BasicDev::~BasicDev()
{
}

void BasicDev::takeoffDrone()
{
    takeoff_client.call(takeoff);
}

void BasicDev::gostraight()
{
    vel_publisher.publish(velcmd);

    // 打印日志信息
    ROS_INFO("Go straight");
}

void BasicDev::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    printf("True_Pose: [%f, %f, %f]\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO("Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    printf("GPS_Pose : [%f, %f, %f]\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Get imu data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
}

void BasicDev::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_left_ptr->image.empty())
    {
        ROS_INFO("Get front left image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
}

void BasicDev::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_right_ptr->image.empty())
    {
        ROS_INFO("Get front right image.%f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
}

void BasicDev::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pts);
    // 添加坐标轴点
    int num_points = 100; // 每个轴插入的点数

    // X 轴：红色
    for (int i = 0; i <= 20; ++i)
    {
        double t = static_cast<double>(i) / num_points;
        double x = 0 + t * 20.0; // 从 0 到 20.0
        double y = 0;
        double z = 0;
        pts->push_back(pcl::PointXYZ(x, y, z));
    }

    // Y 轴：绿色
    for (int i = 0; i <= 10; ++i)
    {
        double t = static_cast<double>(i) / num_points;
        double x = 0;
        double y = 0 + t * 10.0; // 从 0 到 10.0
        double z = 0;
        pts->push_back(pcl::PointXYZ(x, y, z));
    }

    // Z 轴：蓝色
    for (int i = 0; i <= 5; ++i)
    {
        double t = static_cast<double>(i) / num_points;
        double x = 0;
        double y = 0;
        double z = 0 + t * 5.0; // 从 0 到 5.0
        pts->push_back(pcl::PointXYZ(x, y, z));
    }

    // 将 PCL 点云转换回 ROS 点云消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pts, output);
    output.header = msg->header; // 保持时间戳和帧 ID 一致

    // 发布点云消息
    point_cloud_pub.publish(output);

    ROS_INFO("Get lidar data. time: %f, size: %ld", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9, pts->size());
}

#endif