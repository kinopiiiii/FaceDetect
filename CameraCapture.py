#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""画像を取得"""

import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs import msg
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os

c = 0

def color(img):
    global t, c

    #print data
    #rospy.Subscriber("chatter", String, callback)
    #d=np.ndarray(data)
    convert=CvBridge()
    cv_image = convert.imgmsg_to_cv2(img,'bgr8') #colorの場合
    #cv_image = convert.imgmsg_to_cv2(img, '32FC1') #depthの場合
    #cv_image = cv_image.astype(np.float32)#depthの場合

    #print cv_image.shape[0],cv_image.shape[1] #imageの大きさ確認 hsr: 480*640
    #print cv_image.shape[1]/2,cv_image.shape[0]/2
    #print type(cv_image)

    #cv_image = cv2.resize(cv_image,(cv_image.shape[1]/2,cv_image.shape[0]/2))
    if t == 1:
        print "\nwrite result.jpg\n"
        cv2.imwrite(str(c)+'.jpg',cv_image)
        c+=1
        t = 0
    
    # どちらでも同じ
    cv_image[240]=[255,0,0]
    cv_image[240,16]=[0,255,0]
    
    cv_image[:,320]=[255,0,0]
    
    cv2.imshow("Test", cv_image)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()
    rospy.loginfo(rospy.get_caller_id() + "I good")
    
    r = rospy.Rate(10) 
    r.sleep()


def t(string):
    #cv2.imwrite('~/Desktop/object_image_budo-/0.jpg',cv_image)
    print "\n"+string.data+"\n"
    if string.data == "s":
        print "\npublish topic(yes)\n"
        global t
        t = 1
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def Recognition():

    global c
    c = 0
    rospy.init_node('image_view', anonymous=True)
    

    rospy.Subscriber("hsrb/head_rgbd_sensor/rgb/image_color", msg.Image,color) #colorの場合
    #rospy.Subscriber("hsrb/head_rgbd_sensor/depth_registered/image_raw", msg.Image,depth) #depthの場合
    rospy.Subscriber("yes", String,t)
    #rospy.Subscriber("chatter", String, ss)

    rospy.spin()
  
if __name__ == '__main__':
    cur_dir = os.getcwd()
    os.chdir(cur_dir+"/images")
    
    Recognition()
