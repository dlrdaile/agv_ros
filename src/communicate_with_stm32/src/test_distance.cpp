#include "ros/ros.h"
#include "communicate_with_stm32/MotorCmd.h"
#include "communicate_with_stm32/MotorControl.h"
/*
decribe the file
*/
int main(int argc, char **argv)
{
    //设置中文
    setlocale(LC_ALL, "");
    //初始化节点
    ros::init(argc, argv, "node_name");
    //创建节点的句柄
    ros::NodeHandle nh;
    ros::Publisher pb = nh.advertise<communicate_with_stm32::MotorCmd>("stm32TopicCtrl", 10);
    ros::ServiceClient sc = nh.serviceClient<communicate_with_stm32::MotorControl>("stm32SeverCtrl");
    communicate_with_stm32::MotorCmd cmd;
    cmd.isUrgent = true;
    ros::Duration timeout;
    ros::Duration(0.5).sleep();
    cmd.cmd = 4;
    pb.publish(cmd);
    ros::Duration(0.5).sleep();
    cmd.cmd = 1;
    if (argc == 1)
    {
        timeout = ros::Duration(3);
        cmd.data = {1000, 1000, 1000, 1000};
    }
    else if(argc == 3)
    { 
        timeout = ros::Duration(atof(argv[1]));
        int16_t speed = atoi(argv[2]);
        cmd.data = {speed,speed,speed,speed};
    }
    pb.publish(cmd);
    timeout.sleep();
    cmd.cmd = 2;
    pb.publish(cmd);
    cmd.cmd = 4;
    ros::Duration(0.5).sleep();
    pb.publish(cmd);
    sc.waitForExistence();
    communicate_with_stm32::MotorControl scmd;
    scmd.request.Key = true;
    scmd.request.cmd = 16;
    ros::Duration(1).sleep();
    sc.call(scmd);
    return 0;
}