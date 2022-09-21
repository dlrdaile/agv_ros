#! /usr/bin/env python
import math
import rospy
import actionlib
from agv_nav.msg import *
from geometry_msgs.msg import Twist


class MoveSetServerNode:
    def __init__(self) -> None:
        self.result = MoveSetResult()
        self.server = actionlib.SimpleActionServer(
            "move_set_action", MoveSetAction, self.cb, False)
        self.cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.rate_num = 2
        self.server.start()
    def task1(self):
        rate = rospy.Rate(self.rate_num)
        num = 0
        while num < 10:
            if self.check_cancle():
                return False
            num+=1/self.rate_num
            rate.sleep()
        return True

    def task2(self):
        rate = rospy.Rate(self.rate_num)
        twist = Twist()
        num = 0
        while num < 5:
            if self.check_cancle():
                return False
            num+=1/self.rate_num
            twist.angular.z = 0.6
            self.cmd_pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        return True

    def task3(self):
        rate = rospy.Rate(self.rate_num)
        twist = Twist()
        num = 0
        while num < 8:
            if self.check_cancle():
                return False
            if num < 2 or num >= 6:
                twist.linear.x = 0.3
            else:
                twist.linear.x = -0.3
            self.cmd_pub.publish(twist)
            num+=1/self.rate_num
            rate.sleep()
        twist.linear.x = 0
        self.cmd_pub.publish(twist)
        return True

    def task4(self):
        rate = rospy.Rate(self.rate_num)
        twist = Twist()
        num = 0
        while num < 4:
            if self.check_cancle():
                return False
            if num < 1 or num >= 3:
                twist.linear.y = 0.2
            else:
                twist.linear.y = -0.2
            self.cmd_pub.publish(twist)
            num+=1/self.rate_num
            rate.sleep()
        twist.linear.y = 0
        self.cmd_pub.publish(twist)
        return True
    
    
    def task5(self):
        rate = rospy.Rate(self.rate_num)
        num = 0
        while num < 4:
            twist = Twist()
            if self.check_cancle():
                return False
            if num < 1:
                twist.linear.x = 0.2
            elif num < 2:
                twist.linear.y = -0.2
            elif num < 3:
                twist.linear.x = -0.2
            else:
                twist.linear.y = 0.2
            self.cmd_pub.publish(twist)
            num+=1/self.rate_num
            rate.sleep()
        twist = Twist()
        self.cmd_pub.publish(twist)
        return True
    def task6(self):
        """
        进电梯
        """
        rate = rospy.Rate(self.rate_num)
        num = 0
        while num < 6:
            twist = Twist()
            if self.check_cancle():
                return False
            if num < 3:
                twist.linear.x = 0.25
            elif num < 5:
                twist.linear.y = math.pi / 2
            self.cmd_pub.publish(twist)
            num+=1/self.rate_num
            rate.sleep()
        twist = Twist()
        self.cmd_pub.publish(twist)
        return True
    def task7(self):
        """
        出电梯
        """
        rate = rospy.Rate(self.rate_num)
        num = 0
        while num < 3:
            twist = Twist()
            if self.check_cancle():
                return False
            twist.linear.x = 0.25
            self.cmd_pub.publish(twist)
            num+=1/self.rate_num
            rate.sleep()
        twist = Twist()
        self.cmd_pub.publish(twist)
        return True
    def check_cancle(self):
        if self.server.is_preempt_requested():
            self.result.final_status = MoveSetResult.FAIL
            self.server.set_preempted(self.result)
            twist = Twist()
            self.cmd_pub.publish(twist)    
            return True
        return False
    
    def cb(self, goal: MoveSetGoal):
        sucess = True
        if goal.taskId == 0:
            pass
        elif goal.taskId == 1:
            sucess = self.task1()
        elif goal.taskId == 2:
            sucess = self.task2()
        elif goal.taskId == 3:
            sucess = self.task3()
        elif goal.taskId == 4:
            sucess = self.task4()
        elif goal.taskId == 5:
            sucess = self.task5()
        elif goal.taskId == 6:
            sucess = self.task6()
        elif goal.taskId == 7:
            sucess = self.task7()
        else:
            sucess = False
            self.result.final_status = MoveSetResult.FAIL
            self.server.set_aborted(self.result, text="not exit the task Id")
        if sucess:
            self.result.final_status = MoveSetResult.FINISH
            self.server.set_succeeded(self.result, text="sucess!")


if __name__ == "__main__":
    rospy.init_node('move_set_server_node')
    server = MoveSetServerNode()
    rospy.spin()
