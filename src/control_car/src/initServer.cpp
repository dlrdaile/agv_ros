#include "ros/ros.h"
#include "communicate_with_stm32/InitMotor.h"
/*
    服务端实现：解析客户端提交的数据，并运算产生响应
        1.包含头文件
        2.初始化ROS节点
        3.创建节点句柄
        4.创建一个服务器对象
        5.处理请求并产生响应
        6.spin()
*/
uint8_t visit_time = 0;
bool doInit(communicate_with_stm32::InitMotor::Request &request,
            communicate_with_stm32::InitMotor::Response &response)
{
    // 1.处理请求

    // 2.组织响应
    response.battery_config.isOpen = ros::param::param("~battery_isopen",true);
    response.battery_config.freq = ros::param::param("~battery_freq",5000);
    response.encoder_config.isOpen = ros::param::param("~encoder_isopen",true);
    response.encoder_config.freq = ros::param::param("~encoder_freq",100);
    response.press_config.isOpen = ros::param::param("~press_isopen",false);
    response.press_config.freq = ros::param::param("~press_freq",1000);
    response.success = true;
    visit_time++;
    return true;
}
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    // 2.初始化ROS节点
    ros::init(argc, argv, "initMotor");
    // 3.创建节点句柄
    ros::NodeHandle nh;

    // 4.创建一个服务器对象
    ros::ServiceServer server;
    server = nh.advertiseService("stm32Init", doInit);
    // 5.处理请求并产生响应
    // 6.spin()
    ros::Rate rate(10);
    while (ros::ok())
    {
        if(visit_time!=0)
        {
            ros::param::del("initServer");
            ROS_INFO("goodbye!");
            ros::shutdown();
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    
    return 0;
}