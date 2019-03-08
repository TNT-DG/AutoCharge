#!/usr/bin/env python       
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#创建一个image_converter的类
class image_converter:
    def __init__(self):     #这里用的是python中__init__的内置方法，用来储存类的实例的共有属性特征     
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)   #定义了发布者
        self.bridge = CvBridge()    #self.bridge = CvBridge()是定义了CvBridge句柄，用来调用相关的接口转换
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)  #定义了订阅者，第三个参数self.callback是调用类的方法，这里指调用图像转换的方法

        self.point_x = 0
        self.point_y = 0

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        #同时这里还需了解python异常处理中

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        #这里是opencv的简单用法

        
        # 2.获取训练好的人脸的参数数据，这里直接使用默认值
        face_cascade = cv2.CascadeClassifier('/home/sf/catkin_ws/src/ros_opencv/haarcascade_frontalface_default.xml')

        # 3.对图片灰度化处理
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 4.检测人脸，探测图片中的人脸
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.15,
            minNeighbors=5,
            minSize=(5, 5),
            # flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        # 5.标记人脸
        for(x,y,w,h) in faces:
                # 1.原始图片 2.人脸坐标原点 3.标记的高度 4，线的颜色 5，线宽
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
                self.point_x = x
                self.point_y = y
                cv2.putText(cv_image, "hello world!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        
        # # 6.显示图片
        # cv2.imshow("Find Faces!", image)


        # 在opencv的显示窗口中绘制一个圆，作为标记
        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)

        # 显示Opencv格式的图像
        # cv2.imshow("发现{0}个人脸!".format(len(faces), cv_image)
        cv2.imshow("Find Faces!", cv_image)

        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
