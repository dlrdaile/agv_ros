#include "ros/ros.h"
#include "agv_nav/multi_goals_nav.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "multi_goals_node");
    //创建节点的句柄
    ros::NodeHandle nh;
    multi_goals_nav::MultiGoalsNav multi_goal_node(&nh,"dl_agv");
    ros::spin();
    return 0;
}