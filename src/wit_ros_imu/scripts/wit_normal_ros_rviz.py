#!/usr/bin/env python
# -*- coding:utf-8 -*-
from turtle import st
import serial
import struct
import rospy
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf


# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))


# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == int(check_data,16)


# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, acceleration, angularVelocity, angularVelocity_last, pub_flag, last_stamp, qua, qua_last
    angle_flag=False
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != '$':
        key = 0
        return
    if key > 1 and buff[1] != 'D':
        key = 0
        return
    if key < 79:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value
        if checkSum(''.join(data_buff[0:77]).encode('UTF-8'), ''.join(data_buff[78:])) or True:
            # angle_degree = [eval(''.join(data_buff[19+6*i:24+6*i])) for i in range(0, 3)]
            angle_flag = True
            acceleration = [eval(''.join(data_buff[39+6*i:44+6*i])) for i in range(0, 3)]
            angularVelocity = [eval(''.join(data_buff[59+6*i:64+6*i])) for i in range(0, 3)]
            # *32768.0*math.pi/180
        else:
            print("校验失败")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if angle_flag:                
            stamp = rospy.Time.now()
            d_stamp = stamp - last_stamp
            last_stamp = stamp
            dt = d_stamp.to_sec()
            # print(dt)

            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_footprint"

            # mag_msg.header.stamp = stamp
            # mag_msg.header.frame_id = "base_footprint"

            tfs.header.stamp = stamp
            tfs.header.frame_id = "imu_odom"
            tfs.child_frame_id = "base_footprint"

            angle_degree = [angle_degree[i]+0.5*dt*(angularVelocity[i]+angularVelocity_last[i])*1e3 for i in range(0,3)]
            # angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
            angle_radian = angle_degree
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            # 四元数迭代
            # qua[0] = qua_last[0]+500*dt*(-angularVelocity[0]*qua_last[1]-angularVelocity[1]*qua_last[2]-angularVelocity[2]*qua_last[3])
            # qua[1] = qua_last[1]+500*dt*(angularVelocity[0]*qua_last[0]+angularVelocity[2]*qua_last[2]-angularVelocity[1]*qua_last[3])
            # qua[2] = qua_last[2]+500*dt*(angularVelocity[1]*qua_last[0]-angularVelocity[2]*qua_last[1]+angularVelocity[0]*qua_last[3])
            # qua[3] = qua_last[3]+500*dt*(angularVelocity[2]*qua_last[0]+angularVelocity[1]*qua_last[1]-angularVelocity[0]*qua_last[2])

            # qua[0] = qua_last[0]+500*dt*(angularVelocity[0]*qua_last[3]+angularVelocity[2]*qua_last[1]-angularVelocity[1]*qua_last[2])
            # qua[1] = qua_last[1]+500*dt*(angularVelocity[1]*qua_last[3]-angularVelocity[2]*qua_last[0]+angularVelocity[0]*qua_last[2])
            # qua[2] = qua_last[2]+500*dt*(angularVelocity[2]*qua_last[3]+angularVelocity[1]*qua_last[0]-angularVelocity[0]*qua_last[1])
            # qua[3] = qua_last[3]+500*dt*(-angularVelocity[0]*qua_last[0]-angularVelocity[1]*qua_last[1]-angularVelocity[2]*qua_last[2])

            # qua_last = qua

            imu_msg.orientation.x = qua[0]
            imu_msg.orientation.y = qua[1]
            imu_msg.orientation.z = qua[2]
            imu_msg.orientation.w = qua[3]

            imu_msg.angular_velocity.x = angularVelocity[0]
            imu_msg.angular_velocity.y = angularVelocity[1]
            imu_msg.angular_velocity.z = angularVelocity[2]

            imu_msg.linear_acceleration.x = acceleration[0]
            imu_msg.linear_acceleration.y = acceleration[1]
            imu_msg.linear_acceleration.z = acceleration[2]

            tfs.transform.translation.x = 0.0
            tfs.transform.translation.y = 0.0
            tfs.transform.translation.z = 0.0

            tfs.transform.rotation.x = qua[0]
            tfs.transform.rotation.y = qua[1]
            tfs.transform.rotation.z = qua[2]
            tfs.transform.rotation.w = qua[3]

            # mag_msg.magnetic_field.x = magnetometer[0]
            # mag_msg.magnetic_field.y = magnetometer[1]
            # mag_msg.magnetic_field.z = magnetometer[2]

            imu_pub.publish(imu_msg)
            # mag_pub.publish(mag_msg)
            tf_pub.sendTransform(tfs)


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
angularVelocity_last = [0, 0, 0]
acceleration = [0, 0, 0]
# magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
qua = [0,0,0,1]
qua_last = [0,0,0,1]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    find_ttyUSB()
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 9600)
    print("IMU Type: Normal Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    # mag_msg = MagneticField()
    tfs = TransformStamped()

    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            rospy.loginfo("\033[32m串口打开成功...\033[0m")
        else:
            wt_imu.open()
            rospy.loginfo("\033[32m打开串口成功...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31m串口打开失败\033[0m")
        exit(0)
    else:
        imu_pub = rospy.Publisher("wit/imu", Imu, queue_size=10)
        # mag_pub = rospy.Publisher("wit/mag", MagneticField, queue_size=10)
        tf_pub = tf2_ros.TransformBroadcaster()


        while not rospy.is_shutdown():
            last_stamp = rospy.Time.now()
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu 失去连接，接触不良，或断线")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count).decode('UTF-8')
                    # print(buff_data)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])

