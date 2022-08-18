#! /usr/bin/env python

import rospy
from agv_nav.msg import useSensorStatus
import pandas as pd


class SensorStatusClass:
    def __init__(self) -> None:
        self.stat_pub = rospy.Publisher(
            'sensor_status', useSensorStatus, queue_size=5, latch=True)
        self.status_data = useSensorStatus()
        self.timer = rospy.Timer(rospy.Duration(2), self.timer_calbk)

        self.sensor_topic_name_serial = pd.Series(
            ['/PressInfo', '/batteryInfo', '/camera/left/camera_info', '/imu', '/odom', '/scan'])

    def timer_calbk(self, event):
        all_topic = list(map(lambda x:x[0],rospy.get_published_topics()))
        self.status_data.use_press = False
        sensor_isopen = self.sensor_topic_name_serial.isin(all_topic)
        self.status_data.use_battery = sensor_isopen[1]
        self.status_data.use_camera = sensor_isopen[2]
        self.status_data.use_imu = sensor_isopen[3]
        self.status_data.use_odom = sensor_isopen[4]
        self.status_data.use_laser = sensor_isopen[5]
        self.stat_pub.publish(self.status_data)


if __name__ == "__main__":
    rospy.init_node('sensor_status_node')
    node = SensorStatusClass()
    while not rospy.is_shutdown():
        rospy.spin()
