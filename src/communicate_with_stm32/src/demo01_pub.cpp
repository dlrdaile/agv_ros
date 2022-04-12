#include "ros/ros.h"
#include "communicate_with_stm32/MotorCmd.h"
/*
decribe the file
*/
int main(int argc,char **argv){
    //设置中文
    setlocale(LC_ALL, "");
    //初始化节点
    ros::init(argc, argv, "demo");
    //创建节点的句柄
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<communicate_with_stm32::MotorCmd>("liaotian",10);
    communicate_with_stm32::MotorCmd m;
    m.cmd = 0x12;
    m.isUrgent = false;
    *(uint32_t *)m.data.elems = 0xffff8000;
    ros::Rate rate(1);
    while (ros::ok()){
        pub.publish(m);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}