#!/usr/bin/env python
# -*- coding:utf-8 -*-
from nbformat import write
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
def calc_crc16(string):
    # data = bytearray.fromhex(string)
    data = string
    crc = 0xFFFF
    for pos in data:
            crc ^= pos
            for i in range(8):
                    if((crc & 1) != 0):
                            crc >>= 1
                            crc ^= 0xA001
                    else:
                            crc >>= 1

    return ((crc & 0xff) << 8) + (crc >> 8)

def checkCrc16(list_data, check_data):
    return calc_crc16(list_data) == (check_data[0]<<8)+check_data[1]

# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhh", bytearray(raw_data)))


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    angle_flag=False
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x50:
        key = 0
        return
    if key < 29:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value
        # checkCrc16(buff[:27],buff[27:])
        if True:
            acceleration = [hex_to_short(data_buff[8:2:-1])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]

            angularVelocity = [hex_to_short(data_buff[14:8:-1])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

            angle_degree = [hex_to_short(data_buff[26:20:-1])[i] / 32768.0 * 180 for i in range(0, 3)]
            angle_flag = True

            magnetometer = hex_to_short(data_buff[20:14:-1])

        else:
            rospy.loginfo("CRC校验失败！")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if angle_flag:                
            stamp = rospy.get_rostime()

            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_footprint"

            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = "base_footprint"

            tfs.header.stamp = stamp
            tfs.header.frame_id = "imu_odom"
            tfs.child_frame_id = "base_footprint"

            angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

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

            mag_msg.magnetic_field.x = magnetometer[0]
            mag_msg.magnetic_field.y = magnetometer[1]
            mag_msg.magnetic_field.z = magnetometer[2]

            tfs.transform.translation.x = 0.0
            tfs.transform.translation.y = 0.0
            tfs.transform.translation.z = 0.0

            tfs.transform.rotation.x = qua[0]
            tfs.transform.rotation.y = qua[1]
            tfs.transform.rotation.z = qua[2]
            tfs.transform.rotation.w = qua[3]

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)
            tf_pub.sendTransform(tfs)


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    find_ttyUSB()
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 9600)
    print("IMU Type: Normal Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    mag_msg = MagneticField()
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
        mag_pub = rospy.Publisher("wit/mag", MagneticField, queue_size=10)
        tf_pub = tf2_ros.TransformBroadcaster()

        while not rospy.is_shutdown():
            try:
                if wt_imu.writable():
                    # 读取三轴加速度、角速度、地磁、角度
                    wt_imu.write(b'\x50\x03\x00\x34\x00\x0C\x09\x80')
                
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu 失去连接，接触不良，或断线")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    print(buff_count)
                    print(buff_data)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])

        

