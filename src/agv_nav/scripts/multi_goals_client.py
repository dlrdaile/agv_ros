#! /usr/bin/env python

import rospy
import actionlib
from agv_nav.msg import *
from geometry_msgs.msg import Pose

"""
    需求:
        创建两个ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整型数据)服务器会计算 1 到 N 之间所有整数的和,
        这是一个循环累加的过程，返回给客户端，这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用 action实现。
    流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 action Client 对象
        4.等待服务
        5.组织目标对象并发送
        6.编写回调, 激活、连续反馈、最终响应
        7.spin
"""

def done_cb(state:actionlib_msgs.msg.GoalStatus,result:TaskListResult):
    print(state)
    print(result)

def active_cb():
    rospy.loginfo("服务被激活....")


def fb_cb(fb:TaskListFeedback):
    print(fb)
    pass

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("action_client_p")
    # 3.创建 action Client 对象
    client = actionlib.SimpleActionClient("dl_agv",TaskListAction)
    # 4.等待服务
    client.wait_for_server()
    # 5.组织目标对象并发送
    goal_obj = TaskListGoal()
    goal_obj.task_pose_list.header.frame_id = "map"
    goal_obj.task_pose_list.header.stamp = rospy.Time.now()
    x_list = [12.89,0.18,]
    y_list = [0.18,-0.980682373046875,2.8268802165985107]
    o_w_list = [0.009753267302542624,0.7535348182573167,0.9889068244024564]
    o_z_list = [-13.98,0.6574079993990891,-0.14853717598718955]
    goal_obj.move_id_set = [1,2,3]
    for i in range(len(x_list)):
        pose = Pose()
        pose.orientation.z = o_z_list[i]
        pose.orientation.w = o_w_list[i]
        pose.position.x = x_list[i]
        pose.position.y = y_list[i]
        goal_obj.task_pose_list.poses.append(pose)
        goal_obj.start_goal = 1
    client.send_goal(goal_obj,done_cb,active_cb,fb_cb)
    # 6.编写回调, 激活、连续反馈、最终响应
    # 7.spin
    rospy.spin()
