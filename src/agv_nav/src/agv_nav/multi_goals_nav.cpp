#include <cstdio>

#include <fstream>
#include <sstream>

#include "agv_nav/multi_goals_nav.h"

namespace multi_goals_nav
{

    MultiGoalsNav::MultiGoalsNav(ros::NodeHandle *nh, string name)
        : nh_(nh),
          as_(*nh_, name, boost::bind(&MultiGoalsNav::goalCntCB, this, _1), false),
          tfListener(buffer),
          moveSetClient_(*nh_,"move_set_action",true)
    {
        this->feedback_.current_pose.header.frame_id = "base_footprint";
        status_sub_ = nh_->subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 1,
                                                                      boost::bind(&MultiGoalsNav::statusCB, this, _1));
        goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

        cancel_pub_ = nh_->advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

        moveset_cancel_pub_ = nh_->advertise<actionlib_msgs::GoalID>("move_set_action/cancel", 1);

        marker_pub_ = nh_->advertise<visualization_msgs::Marker>("visualization_marker", 1);
        this->moveSetClient_.waitForServer();
        as_.start();
    }

    // delete marks in the map
    void MultiGoalsNav::deleteMark()
    {
        visualization_msgs::Marker marker_delete;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        marker_pub_.publish(marker_delete);
    }

    void MultiGoalsNav::initStatus()
    {
        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false, cycle_ = false;
        pose_array_.poses.clear();
        this->move_id_set_.clear();
        this->feedback_.task_status.clear();
        this->feedback_.current_task_iswork = false;
        this->result_.start_time = ros::Time::now();
        this->result_.task_final_status.clear();
        this->result_.task_complete_time.clear();
        this->deleteMark();
    }

    // call back function for counting goals
    void MultiGoalsNav::goalCntCB(const agv_nav::TaskListGoalConstPtr &ptr)
    {
        bool success = true;
        geometry_msgs::PoseStamped markpose;
        this->initStatus();
        pose_array_.header.frame_id = markpose.header.frame_id = ptr->task_pose_list.header.frame_id;
        this->move_id_set_ = ptr->move_id_set;
        this->curGoalIdx_ = ptr->start_goal;
        for (auto &pose : ptr->task_pose_list.poses)
        {
            pose_array_.poses.push_back(pose);
            markpose.pose = pose;
            markPose(markpose);
            this->feedback_.task_status.push_back(this->feedback_.WAITING);
            ros::Duration(0.1).sleep();
        }
        for(auto i = 0;i < this->curGoalIdx_;++i){
            this->feedback_.task_status[i] = this->feedback_.SUCCEEDED;
        }
        ROS_INFO("it is time to start work!");
        this->startNavi();
        ros::Rate rate(10);
        while (this->permit_)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                as_.setPreempted();
                this->cancelNavi();
                this->feedback_.task_status[this->curGoalIdx_-1] = this->feedback_.PAUSE;
                this->permit_ = false;
                success = false;
                break;
            }
            try
            {
                auto son1Toson2 = buffer.lookupTransform("map", "base_footprint", ros::Time(0));
                this->feedback_.current_pose.header.stamp = ros::Time::now();
                this->feedback_.current_pose.pose.orientation = son1Toson2.transform.rotation;
                this->feedback_.current_pose.pose.position.x = son1Toson2.transform.translation.x;
                this->feedback_.current_pose.pose.position.y = son1Toson2.transform.translation.y;
                this->feedback_.current_pose.pose.position.z = son1Toson2.transform.translation.z;
                this->feedback_.text = "[INFO] current status is normal";
                as_.publishFeedback(this->feedback_);
            }
            catch (const std::exception &e)
            {
                this->feedback_.text = "[ERROR] current status is unnormal!";
                as_.publishFeedback(this->feedback_);
                std::cerr << e.what() << '\n';
            }
            rate.sleep();
        }
        this->result_.task_final_status = this->feedback_.task_status;
        for (auto &i : this->result_.task_final_status)
        {
            if (i != this->feedback_.SUCCEEDED)
            {
                success = false;
                break;
            }
        }
        this->result_.end_time = ros::Time::now();
        if (success)
        {
            as_.setSucceeded(this->result_, "sucess to complete the task!");
        }
        else
        {
            as_.setPreempted(this->result_);
        }
    }

    // when setting a Navi Goal, it will set a mark on the map
    void MultiGoalsNav::markPose(const geometry_msgs::PoseStamped &pose)
    {
        if (ros::ok())
        {
            visualization_msgs::Marker arrow;
            visualization_msgs::Marker number;
            arrow.header.frame_id = number.header.frame_id = pose.header.frame_id;
            arrow.ns = "navi_point_arrow";
            number.ns = "navi_point_number";
            arrow.action = number.action = visualization_msgs::Marker::ADD;
            arrow.type = visualization_msgs::Marker::ARROW;
            number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            arrow.pose = number.pose = pose.pose;
            number.pose.position.z += 1.0;
            arrow.scale.x = 1.0;
            arrow.scale.y = 0.2;
            number.scale.z = 1.0;
            arrow.color.r = number.color.r = 1.0f;
            arrow.color.g = number.color.g = 0.98f;
            arrow.color.b = number.color.b = 0.80f;
            arrow.color.a = number.color.a = 1.0;
            arrow.id = number.id = pose_array_.poses.size();
            number.text = std::to_string(pose_array_.poses.size());
            marker_pub_.publish(arrow);
            marker_pub_.publish(number);
        }
    }

    // check whether it is in the cycling situation
    void MultiGoalsNav::setCycle(bool isCycle)
    {
        this->cycle_ = isCycle;
    }

    // start to navigate, and only command the first goal
    void MultiGoalsNav::startNavi()
    {
        curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
        if (!pose_array_.poses.empty() && curGoalIdx_ < pose_array_.poses.size())
        {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = pose_array_.header.frame_id;
            goal.header.stamp = ros::Time::now();
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            this->feedback_.task_status[curGoalIdx_] = this->feedback_.ACTIVE;
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
            curGoalIdx_ += 1;
            permit_ = true;
        }
        else
        {
            ROS_ERROR("Something Wrong");
        }
    }

    // complete the remaining goals
    void MultiGoalsNav::completeNavi()
    {
        // TODO:完成动作节点的任务
        this->feedback_.current_task_iswork = true;
        this->move_set_goal.taskId = this->move_id_set_[this->curGoalIdx_-1];
        this->moveSetClient_.sendGoal(this->move_set_goal);
        this->moveSetClient_.waitForResult();
        this->feedback_.current_task_iswork = false;

        if (curGoalIdx_ < pose_array_.poses.size())
        {
            this->result_.task_complete_time.push_back(ros::Time::now());
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = pose_array_.header.frame_id;
            goal.header.stamp = ros::Time::now();
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            this->feedback_.current_task_iswork = false;
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
            curGoalIdx_ += 1;
        }
        else
        {
            ROS_INFO("All goals are completed");
            permit_ = false;
        }
    }

    // command the goals cyclically
    void MultiGoalsNav::cycleNavi()
    {
        if (permit_)
        {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%lu, in the %dth cycle", curGoalIdx_ % pose_array_.poses.size() + 1,
                     cycleCnt_ + 1);
            curGoalIdx_ += 1;
            cycleCnt_ = curGoalIdx_ / pose_array_.poses.size();
        }
    }

    // cancel the current command
    void MultiGoalsNav::cancelNavi()
    {
        if (!cur_goalid_.id.empty())
        {
            actionlib_msgs::GoalID moveset_goal_id;
            this->deleteMark();
            cancel_pub_.publish(cur_goalid_);
            this->moveset_cancel_pub_.publish(moveset_goal_id);
            ROS_ERROR("Navigation have been canceled");
        }
    }

    // call back for listening current state
    void MultiGoalsNav::statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses)
    {
        if (this->permit_)
        {
            bool arrived_pre = arrived_;
            arrived_ = checkGoal(statuses->status_list);
            if (arrived_ && arrived_ != arrived_pre && ros::ok())
            {
                if (cycle_)
                    cycleNavi();
                else
                    completeNavi();
            }
        }
    }

    // check the current state of goal
    bool MultiGoalsNav::checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list)
    {
        bool done;
        if (!status_list.empty())
        {
            for (auto &i : status_list)
            {
                if (i.status == i.SUCCEEDED)
                {
                    done = true;
                    this->feedback_.task_status[this->curGoalIdx_-1] = this->feedback_.SUCCEEDED;
                    ROS_INFO("completed Goal%d", curGoalIdx_);
                }
                else if (i.status == i.ABORTED)
                {
                    ROS_ERROR("Goal%d is Invalid, Navi to Next Goal%d", curGoalIdx_, curGoalIdx_ + 1);
                    this->feedback_.task_status[this->curGoalIdx_-1] = this->feedback_.FAIL;
                    return true;
                }
                else if (i.status == i.PENDING)
                {
                    done = true;
                }
                else if (i.status == i.ACTIVE)
                {
                    this->feedback_.task_status[this->curGoalIdx_-1] = this->feedback_.ACTIVE;
                    ROS_INFO("Goal%d is working", curGoalIdx_);
                    cur_goalid_ = i.goal_id;
                    done = false;
                }
                else
                    done = false;
            }
        }
        else
        {
            ROS_INFO("Please input the Navi Goal");
            done = false;
        }
        return done;
    }
} // end namespace navi-multi-goals-pub-rviz-plugin
