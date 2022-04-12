#!/usr/bin/env python
from unicodedata import name
import rospy
from communicate_with_stm32.msg import MotorCmd,MotorData
import json
import sys
# import numpy as np
class ROSNode:
    def __init__(self):
        rospy.init_node("name_node")
        rospy.loginfo("Starting ROSNode as name_node.")
        self.pub = rospy.Publisher('stm32TopicCtrl',MotorCmd,queue_size=10)
        self.sub = rospy.Subscriber('motorState',MotorData,self.read_data_cv,queue_size=10)
        self.cmd_data = MotorCmd()
        self.cmd_data.cmd = 1
        self.cmd_data.isUrgent = True
        self.w = [[],[],[],[]]
        self.speed = 0
        self.delta_speed = 100
    def read_data_cv(self,msg:MotorData):
        # enc = np.array(list(msg.aveInfo.encoderData)) * 1000 / msg.aveInfo.duration
        # set_speed = np.array(list(msg.incInfo.encoderData)) * 1e-5 * 50
        for i in range(4):
            enc = msg.aveInfo.encoderData[i] * 1000 / msg.aveInfo.duration
            set_speed = msg.incInfo.encoderData[i] * 1e-5 * 100
            self.w[i].append([enc,set_speed])
    def pub_data(self):
        for i in range(4):
            self.cmd_data.data[i] = self.speed
        self.pub.publish(self.cmd_data)
        self.speed += self.delta_speed
        

if __name__ == "__main__":
    name_node = ROSNode()
    rate = rospy.Rate(1)
    times = 0
    while not rospy.is_shutdown():
        name_node.pub_data()
        if(name_node.speed >= 6000) or name_node.speed <= -6000:
            name_node.delta_speed *= -1
        if name_node.speed == 0:
            times+=1
            if(times == 10):
                with open('/home/dlr/Desktop/ros/2.json','w') as f:
                    json.dump(name_node.w,f,indent=2)
                name_node.pub_data()
                break
        rate.sleep()
    
    
        
