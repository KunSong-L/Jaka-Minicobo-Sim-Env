#!/usr/bin/python3.8
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

save_path = "/home/song/fig/"
def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 1:
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        timestr = "%.6f" %  data.header.stamp.to_sec()
              #%.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = "image2" + ".jpg"
        cv2.imwrite(save_path + image_name, cv_img)  #保存；
        cv2.imshow("frame" , cv_img)
        cv2.waitKey(0)
    else:
        pass
 
def displayWebcam():
    rospy.init_node('save_fig_node', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/d435/color/image_raw', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':
    displayWebcam()
 