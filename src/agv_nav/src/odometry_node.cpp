#include "agv_nav/odometry_node.hpp"
/*
decribe the file
*/
int main(int argc,char **argv){
    //设置中文
    setlocale(LC_ALL, "");
    //初始化节点
    ros::init(argc, argv, "odometry_node");
    //创建节点的句柄
    ros::NodeHandle nh;
    odometry_node odom(&nh);
    ros::spin();
    return 0;
}