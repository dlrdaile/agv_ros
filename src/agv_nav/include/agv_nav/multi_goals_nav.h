#ifndef MULTI_NAVI_GOAL_PANEL_H
#define MULTI_NAVI_GOAL_PANEL_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_datatypes.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "agv_nav/TaskListAction.h"
#include "agv_nav/MoveSetAction.h"

namespace multi_goals_nav
{
    using namespace std;
    typedef actionlib::SimpleActionClient<agv_nav::MoveSetAction> Client;
    class MultiGoalsNav
    {
    public:
        explicit MultiGoalsNav(ros::NodeHandle *nh, string name);

    public:
        //设置标记
        void markPose(const geometry_msgs::PoseStamped &pose);
        void deleteMark();

    protected:
        //导航相关的函数
        void startNavi(); // start navigate for the first pose
        void cancelNavi();

        void initStatus();

        void goalCntCB(const agv_nav::TaskListGoalConstPtr &ptr); // goal count sub callback function

        void statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses); // status sub callback function


        void setCycle(bool isCycle);

        void completeNavi(); // after the first pose, continue to navigate the rest of poses
        void cycleNavi();

        bool checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list); // check whether arrived the goal
    protected:
        // The ROS node handle.
        ros::NodeHandle *nh_;
        ros::Publisher goal_pub_, cancel_pub_, marker_pub_,moveset_cancel_pub_;
        ros::Subscriber status_sub_;
        
        actionlib::SimpleActionServer<agv_nav::TaskListAction> as_;
        Client moveSetClient_;
        agv_nav::MoveSetGoal move_set_goal;
        int curGoalIdx_ = 0, cycleCnt_ = 0;
        bool permit_ = false, cycle_ = false, arrived_ = false;
        geometry_msgs::PoseArray pose_array_;
        vector<int8_t> move_id_set_;

        actionlib_msgs::GoalID cur_goalid_;

        agv_nav::TaskListFeedback feedback_;
        agv_nav::TaskListResult result_;

        //监听当前小车的位置
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener tfListener;
        double roll, pitch, yaw;
    };

} // end namespace navi-multi-goals-pub-rviz-plugin

#endif // MULTI_NAVI_GOAL_PANEL_H
