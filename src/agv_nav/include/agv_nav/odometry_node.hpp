#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "communicate_with_stm32/Encoderinfo.h"
#include "agv_nav/mecumnamu.h"
#include "cmath"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
class odometry_node
{
public:
    odometry_node(ros::NodeHandle *nh) : _nh(nh), isfirst(true)
    {
        this->coef = 0.00010397;
        this->intercept = -0.000335;
        this->a = 0.134; // m/s
        this->b = 0.135; // m/s
        my_car = new mecumnamu(this->a, this->b);
        this->use_ekf = ros::param::param("use_ekf", false);
        // this->use_ekf = true;
        initializePublisher();
        initializeSubscribers();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.pose.covariance[0] = 1e-3;
        odom_msg.pose.covariance[7] = 1e-3;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 1e3;
        odom_msg.twist.covariance[0] = 1e-3;
        odom_msg.twist.covariance[7] = 1e-3;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e3;
        this->thre = 6000 * this->coef + this->intercept;
    }
    ~odometry_node()
    {
        if (my_car != NULL)
        {
            delete my_car;
            my_car = NULL;
        }
    }

public:
    void cmdVelCallback(const geometry_msgs::Twist &msg)
    {
        this->cmd_center_speed[0] = msg.linear.x;
        this->cmd_center_speed[1] = msg.linear.y;
        this->cmd_center_speed[2] = msg.angular.z;
        this->my_car->center2wheel(this->cmd_wheel_speed, this->cmd_center_speed);
        ROS_INFO("re:cmd_wheel_speed:%f,%f,%f,%f\n", this->cmd_wheel_speed[0], this->cmd_wheel_speed[1], this->cmd_wheel_speed[2],
                 this->cmd_wheel_speed[3]);
    }

    void motorStateCallback(const communicate_with_stm32::Encoderinfo &msg)
    {
        if (this->isfirst)
        {
            this->isfirst = false;
            this->last_time = msg.header.stamp;
            for (int i = 0; i < 4; i++)
            {
                this->last_encoder[i] = msg.encoderData[i];
            }
            this->x = 0;
            this->y = 0;
            this->yaw = 0;
        }
        else
        {
            this->current_time = msg.header.stamp;
            this->duration_time = this->current_time - this->last_time;
            float dura = this->duration_time.toSec();
            for (int i = 0; i < 4; i++)
            {
                this->current_encoder[i] = msg.encoderData[i];
                this->wheel_speed[i] = (this->current_encoder[i] - this->last_encoder[i]) / dura * this->coef;
            }
            this->my_car->wheel2center(this->wheel_speed, this->center_speed);
            if (this->check_receive_data())
            {
                float dx, dy, dyaw;
                dx = this->center_speed[0] * dura;
                dy = this->center_speed[1] * dura;
                dyaw = this->center_speed[2] * dura;
                this->yaw += dyaw;
                this->yaw = (this->yaw > M_PI) ? (this->yaw - 2 * M_PI) : ((this->yaw < -M_PI) ? (this->yaw + 2 * M_PI) : this->yaw);
                x += (dx * cos(this->yaw) - dy * sin(this->yaw));
                y += (dx * sin(this->yaw) + dy * cos(this->yaw));
                odom_msg.pose.pose.position.x = x;
                odom_msg.pose.pose.position.y = y;
                odom_msg.pose.pose.position.z = 0;
                static tf2::Quaternion qtn;
                qtn.setRPY(0, 0, this->yaw);
                odom_msg.pose.pose.orientation.x = qtn.getX();
                odom_msg.pose.pose.orientation.y = qtn.getY();
                odom_msg.pose.pose.orientation.z = qtn.getZ();
                odom_msg.pose.pose.orientation.w = qtn.getW();
                odom_msg.header.stamp = this->current_time;
                odom_msg.twist.twist.linear.x = this->center_speed[0];
                odom_msg.twist.twist.linear.y = this->center_speed[1];
                odom_msg.twist.twist.angular.z = this->center_speed[2];
                if (!this->use_ekf)
                {
                    geometry_msgs::TransformStamped ts;
                    ts.header = odom_msg.header;
                    ts.child_frame_id = "base_footprint";
                    ts.transform.translation.x = odom_msg.pose.pose.position.x;
                    ts.transform.translation.y = odom_msg.pose.pose.position.y;
                    ts.transform.translation.z = 0;
                    ts.transform.rotation.x = qtn.getX();
                    ts.transform.rotation.y = qtn.getY();
                    ts.transform.rotation.z = qtn.getZ();
                    ts.transform.rotation.w = qtn.getW();
                    this->tf_pub.sendTransform(ts);
                    this->odeometry_pub.publish(odom_msg);
                }
                else
                {
                    this->odom2ekf_pub.publish(odom_msg);
                }
            }
            else
            {
                ROS_ERROR("odom say:receive a error encoder!");
            }
            this->last_time = this->current_time;
            for (int i = 0; i < 4; i++)
            {
                this->last_encoder[i] = this->current_encoder[i];
            }
        }
    }

    void ekfOdomCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
    {
        static nav_msgs::Odometry ekf_odom;
        ekf_odom.header = msg.header;
        ekf_odom.child_frame_id = this->odom_msg.child_frame_id;
        ekf_odom.twist = this->odom_msg.twist;
        ekf_odom.pose.covariance = msg.pose.covariance;
        ekf_odom.pose.pose.position = msg.pose.pose.position;
        ekf_odom.pose.pose.orientation = msg.pose.pose.orientation;
        this->odeometry_pub.publish(ekf_odom);
    }
    void initializeSubscribers()
    {
        this->motor_sub = this->_nh->subscribe("encoderInfo", 100, &odometry_node::motorStateCallback, this);
        if (use_ekf)
        {
            this->odom_tf_sub = this->_nh->subscribe("robot_pose_ekf/odom_combined", 100, &odometry_node::ekfOdomCallback, this);
        }
        // this->cmd_vel_sub = this->_nh->subscribe("cmd_vel", 10, &odometry_node::cmdVelCallback, this);
    }
    void initializePublisher()
    {
        this->odeometry_pub = this->_nh->advertise<nav_msgs::Odometry>("odom", 100);
        if (this->use_ekf)
            this->odom2ekf_pub = this->_nh->advertise<nav_msgs::Odometry>("odom2ekf", 100);
    }

private:
    bool check_receive_data()
    {
        for (int i = 0; i < 4; i++)
        {
            if (abs(this->wheel_speed[i]) >= this->thre)
            {
                ROS_ERROR("to big");
                ROS_ERROR("negative");
                ROS_INFO("wheel_speed:%f,%f,%f,%f\n", this->wheel_speed[0], this->wheel_speed[1], this->wheel_speed[2],
                         this->wheel_speed[3]);
                ROS_INFO("cmd_wheel_speed:%f,%f,%f,%f\n", this->cmd_wheel_speed[0], this->cmd_wheel_speed[1], this->cmd_wheel_speed[2],
                         this->cmd_wheel_speed[3]);
                ROS_INFO("center_speed:%f,%f,%f\n", this->center_speed[0], this->center_speed[1], this->center_speed[2]);
                ROS_INFO("cmd_center_speed:%f,%f,%f\n", this->cmd_center_speed[0], this->cmd_center_speed[1], this->cmd_center_speed[2]);
                return false;
            }
            // if (((this->wheel_speed[i] * this->cmd_wheel_speed[i]) < 0) && (abs(this->wheel_speed[i]) >= 0.01))
            // {
            //     ROS_ERROR("negative");
            //     ROS_INFO("wheel_speed:%f,%f,%f,%f\n", this->wheel_speed[0], this->wheel_speed[1], this->wheel_speed[2],
            //              this->wheel_speed[3]);
            //     ROS_INFO("cmd_wheel_speed:%f,%f,%f,%f\n", this->cmd_wheel_speed[0], this->cmd_wheel_speed[1], this->cmd_wheel_speed[2],
            //              this->cmd_wheel_speed[3]);
            //     ROS_INFO("center_speed:%f,%f,%f\n", this->center_speed[0], this->center_speed[1], this->center_speed[2]);
            //     ROS_INFO("cmd_center_speed:%f,%f,%f\n", this->cmd_center_speed[0], this->cmd_center_speed[1], this->cmd_center_speed[2]);
            //     return false;
            // }
        }
        return true;
    }

private:
    ros::NodeHandle *_nh;
    ros::Subscriber motor_sub;
    ros::Publisher odeometry_pub;
    ros::Publisher odom2ekf_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber odom_tf_sub;
    tf2_ros::TransformBroadcaster tf_pub;

    nav_msgs::Odometry odom_msg;
    mecumnamu *my_car;

    bool isfirst;
    float yaw, x, y;
    int last_encoder[4], current_encoder[4];

    float wheel_speed[4];
    float center_speed[3];
    float cmd_wheel_speed[4];
    float cmd_center_speed[3];
    float thre;

    ros::Time last_time, current_time;
    ros::Duration duration_time;
    float coef;
    float intercept;
    float a;
    float b;
    bool use_ekf;
};