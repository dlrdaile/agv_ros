#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "communicate_with_stm32/MotorData.h"
#include "agv_nav/mecumnamu.h"
#include "cmath"

class odeometry_node
{
public:
    odeometry_node(ros::NodeHandle &nh, mecumnamu &m) : _nh(nh), my_car(m)
    {
        initializePublisher();
        initializeSubscribers();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
    }
    ~odeometry_node() {}

private:
    void initializeSubscribers()
    {
        this->motor_sub = this->_nh.subscribe("motorState", 10, odeometry_node::motorStateCallback, this);
    }
    void initializePublisher()
    {
        this->odeometry_pub = this->_nh.advertise<nav_msgs::Odometry>("odom", 100);
        // this->battery_pub = this->_nh.advertise<>()
    }
    void motorStateCallback(const communicate_with_stm32::MotorData &msg)
    {
        float wheel_speed[4];
        float center_speed[3];
        float dx,dy,dz;
        static tf2_ros::TransformBroadcaster tf_pub;
        static bool flag = false;
        if(!flag)
        {
            if(msg.aveInfo.duration != 0)
            {
                this->last_time = msg.aveInfo.time;
                flag = true;
            }
            return;
        }

        this->current_time = msg.aveInfo.time;
        odom_msg.header.stamp = this->current_time;
        for (size_t i = 0; i < 4; i++)
        {
            //todo:写一个数据校验的功能
            wheel_speed[i] = msg.aveInfo.encoderData[i] / msg.aveInfo.duration * 1000 * this->coef + this->intercept;
        }
        this->my_car.wheel2center(wheel_speed, center_speed);
        odom_msg.twist.twist.linear.x = center_speed[0];
        odom_msg.twist.twist.linear.y = center_speed[1];
        odom_msg.twist.twist.angular.z = center_speed[2];
        ros::Duration delta_time = this->current_time - this->last_time;
        dx = delta_time.toSec() * center_speed[0];
        dy = delta_time.toSec() * center_speed[1];
        dz = delta_time.toSec() * center_speed[2];

        yaw += dz;
        yaw = (yaw > M_PI) ? (yaw - 2*M_PI) : ((yaw < -M_PI) ? (yaw + 2*M_PI) : yaw);
        odom_msg.pose.pose.position.x -= dx*cos(yaw)+dy*sin(yaw);
        odom_msg.pose.pose.position.y += dx*sin(yaw)-dy*cos(yaw);

        static tf2::Quaternion qtn;
        qtn.setRPY(0,0,yaw);
        odom_msg.pose.pose.orientation.x = qtn.getX();
        odom_msg.pose.pose.orientation.y = qtn.getY();
        odom_msg.pose.pose.orientation.z = qtn.getZ();
        odom_msg.pose.pose.orientation.w = qtn.getW();

        geometry_msgs::TransformStamped ts;
        ts.child_frame_id = "base_footprint";
        ts.header.frame_id = "odom";
        ts.header.stamp = this->current_time;
        ts.transform.translation.x = odom_msg.pose.pose.position.x;
        ts.transform.translation.y = odom_msg.pose.pose.position.y;
        ts.transform.translation.z = 0;
        ts.transform.rotation.x = qtn.getX();
        ts.transform.rotation.y = qtn.getY();
        ts.transform.rotation.z = qtn.getZ();
        ts.transform.rotation.w = qtn.getW();
        tf_pub.sendTransform(ts);
        this->odeometry_pub.publish(odom_msg);
        this->last_time = this->current_time;
    }

private:
    ros::NodeHandle &_nh;
    ros::Subscriber motor_sub;
    ros::Publisher odeometry_pub;
    ros::Publisher battery_pub;
    nav_msgs::Odometry odom_msg;
    ros::Time last_time;
    ros::Time current_time;
    mecumnamu &my_car;
    float yaw;
    const float coef = 0.00010397;
    const float intercept = -0.000335;
};