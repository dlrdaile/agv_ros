#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "communicate_with_stm32/MotorCmd.h"
/*
decribe the file
*/

class demo_node
{
private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub;
    ros::Publisher car_cmd_pub;
    communicate_with_stm32::MotorCmd mcl;
    void sub_tele_cb(const geometry_msgs::TwistConstPtr &msg);
public:
    demo_node(){
        cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",10,&demo_node::sub_tele_cb,this);
        car_cmd_pub = nh.advertise<communicate_with_stm32::MotorCmd>("stm32TopicCtrl", 10,true);
    }
};

void demo_node::sub_tele_cb(const geometry_msgs::TwistConstPtr &msg){
    mcl.cmd = 6;
    mcl.isUrgent = true;
    mcl.data[0] = msg->linear.x * 10000;
    mcl.data[1] = msg->linear.y * 10000;
    mcl.data[2] = msg->angular.z * 10000;
    mcl.data[3] = 0;
    car_cmd_pub.publish(mcl);
}


int main(int argc,char **argv){
    //设置中文
    setlocale(LC_ALL, "");
    //初始化节点
    ros::init(argc, argv, "car_tele_cmd");
    demo_node demo;
    ros::spin();
    return 0;
}