#! /usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


def cv_show(name, image):      # 展示图像
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def picture_pretreatment(image):   # 图像预处理
    image = cv2.medianBlur(image, 5)     # 中值滤波     # 效果最好
    image = cv2.boxFilter(image, -1, (3, 3), normalize=True)   # 方框滤波

    image = cv2.GaussianBlur(image, (5, 5), 1)          # 高斯滤波
    # image = cv2.blur(image, (3, 3))                        # 均值滤波
    # cv_show("滤波效果", image)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, gray_image = cv2.threshold(
        gray_image, 70, 255, cv2.THRESH_TOZERO)   # 低于 第一个数的灰度值 全都设为0 保留其他部分
    # cv_show("gray", gray_image)
    return gray_image


def histogram_equalization(image):
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(
        clipLimit=10.0, tileGridSize=(27, 27))  # 对图像进行分割，10*10
    img4 = clahe.apply(image)       # 进行直方图均衡化
    # cv_show("img4", img4)
    return img4


def mask_create(image):    # 筛选重要区域
    # image2 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    kernel = np.ones((7, 7), np.uint8)
    # 通过腐蚀操作来获取边界 当然可以通过其他方式获取边界
    gradient = cv2.morphologyEx(image, cv2.MORPH_GRADIENT, kernel)
    # cv_show('gradient', gradient)
    ret, thresh = cv2.threshold(
        gradient, 80, 255, cv2.THRESH_BINARY)   # 二值化获取边缘
    _, mask = cv2.threshold(
        gradient, 0, 0, cv2.THRESH_BINARY)          # 获取mask黑底板
    # cv_show('thresh', thresh)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # 筛选最大边界
    area = []
    for i in range(len(contours)):
        area.append(cv2.contourArea(contours[i]))
    res_max = np.argmax(np.array(area))
    # 筛选出图像的边缘
    mask = cv2.drawContours(mask, contours, res_max, [
                            255, 255, 255], cv2.FILLED)
    # cv_show("mask", mask)
    image_vital = cv2.bitwise_and(image, image, mask=mask)
    # cv_show("image_vital", image_vital)
    return image_vital


def grad_treatment(image, original_picture=None):      # 梯度处理，目测不太好用
    if not original_picture:
        original_picture = image.copy()

    laplacian = cv2.Laplacian(image, cv2.CV_64F)
    laplacian = cv2.convertScaleAbs(laplacian)
    gradX = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=3)
    gradY = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=3)
    gradX = cv2.convertScaleAbs(gradX)
    gradY = cv2.convertScaleAbs(gradY)
    gradDst = cv2.addWeighted(gradX, 0.5, gradY, 0.5, 2)
    res = np.hstack((gradX, gradY, gradDst))
    # cv_show('res', res)
    # cv_show('gradDst0', gradDst)
    # cv_show("laplacian", laplacian)
    return res, gradDst


def canny_treatment(image):  # 梯度检测
    v2 = cv2.Canny(image, 60, 100)    #
    # cv_show("canny_treatment", v2)
    return v2


def hist_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hist = cv2.calcHist([image], [0], None, [256], [0, 256])
    # plt.plot(hist)          # 折线图统计
    # plt.show()
    # plt.hist(image.ravel(), 256, [0, 256])
    # plt.show()          # 原图像灰度展示
    equ = cv2.equalizeHist(image)

    # plt.hist(equ.ravel(), 256, [0, 256])
    # plt.show()
    # cv_show("equ", equ)
    return equ


def open_treatment(image):  # 开运算 先腐蚀再膨胀
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return opening


def fincontours_surface_defect(image, original_image):
    contours, hie = cv2.findContours(
        image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # chontours 是一个数组，接下来要对这个数组进行处理，筛选有效边框

    perimeter_count = 0
    area_count = 0
    area_number = 0  # 面积计数器
    for contour in contours:
        # 对每一轮廓进行遍历
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 8000:  # 边框太大的是边界 删掉
            perimeter = 0
        perimeter_count = perimeter_count + perimeter
        # 计算轮廓总长度

        area = cv2.contourArea(contour)
        if area > 20000:  # 面积过大的是边界删掉
            area = 0
        if area > 0:
            area_number += 1
        area_count = area + area_count
        # 计算轮廓总面积

    # 记录符合筛选条件的轮廓

    # 计算封闭轮廓 平均多大
    average_area = area_count/area_number    # 平均轮廓线长度
    average_perimeter = perimeter_count/area_number

    # 创建一个数组储存符合条件边框结果
    resultContours = []
    for contour in contours:
        # 对每一轮廓进行遍历

        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        if average_area*0.9 < area < 12000 and perimeter > average_perimeter*0.8:
            resultContours.append(contour)

    cv2.drawContours(original_image, resultContours, -1, (0, 0, 255), 3)
    return original_image


class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher(
            "cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/left/image_raw", Image, self.callback)

    def callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # 在opencv的显示窗口中绘制一个圆，作为标记
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)

        # 显示Opencv格式的图像
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def detect_surface(self, original_image):
        using_image = picture_pretreatment(original_image)   # 图像预处理
        # 创建mask滤波
        image = mask_create(using_image)
        # 直方图凸显 但是黑了点
        # image = hist_image(image)
        image = histogram_equalization(image)

        # 二值化
        ret, image = cv2.threshold(image, 95, 255, cv2.THRESH_BINARY)   # 查找边缘
        # 闭运算 操作填一下空隙
        # 给一个卷积核
        kernel = np.ones((5, 5), np.uint8)
        image = cv2. morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        # cv_show("top_hat", image)

        image = canny_treatment(image)    # 边缘检测

        cv_show("1-0picture", image)
        return image

    def run_demo(self):
        original_image = cv2.imread(
            "/home/agv/Desktop/ros/ws/stm32_ws/src/agv_nav/test/item02_r.jpg")
        cv_show("original_image", original_image)

        self.detect_surface(original_image)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")   # 节点
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
