#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""画像を取得"""
#TODO 照明,処理速度
import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs import msg
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os
import threading
from datetime import datetime
import hsrb_interface
import sys
from hsrb_interface import geometry

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import trajectory_msgs.msg
import controller_manager_msgs.srv

import roslib
count = 0
#顔検出角度の初期化
#rad = 0.0
#顔検出フラグの初期化
face_detect_flag = False
detect_complete_flag = False
traj_flag = False
angle_person = 0
cv_image_depth = []
#ロボット動作準備
#robot = hsrb_interface.Robot()
#omni_base = robot.get('omni_base')
#whole_body = robot.get('whole_body')
#gripper = robot.get('gripper')
#tts = robot.get('default_tts')

# パブリッシャーの初期化
rospy.init_node('FaceDetect', anonymous=True)
pub_head = rospy.Publisher('/hsrb/head_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
pub_vel = rospy.Publisher('/hsrb/command_velocity',Twist,queue_size=1)
pub_say = rospy.Publisher('/f_rotation',Bool,queue_size=1)

# コントローラーとの接続待機
while pub_head.get_num_connections() == 0 and not rospy.is_shutdown():
    rospy.sleep(0.1)

#コントローラーとの接続確認
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
running = False
while running == False and not rospy.is_shutdown():
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'head_trajectory_controller' and c.state == 'running':
            running = True

#顔検出の処理及び回転、停止
class FaceThread(threading.Thread):
    def __init__(self, frame):
        super(FaceThread, self).__init__()
        global face_detect_flag,angle_person,count
        self._cascade_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"
        #画像を取得
        self._frame = frame
    def run(self):
        global face_detect_flag,angle_person,count,detect_complete_flag
        #グレースケール変換
        self._frame_gray = cv2.cvtColor(self._frame, cv2.cv.CV_BGR2GRAY)
        #カスケード分類器の特徴量を取得する
        self._cascade = cv2.CascadeClassifier(self._cascade_path)
        #物体認識（顔認識）の実行
        self._facerect = self._cascade.detectMultiScale(self._frame_gray, scaleFactor=1.2, minNeighbors=3, minSize=(10, 10))
        if len(self._facerect) > 0:
            #顔座標の検出
            face_x = self._facerect[0][0]+self._facerect[0][2]/2
            face_y = self._facerect[0][1]+self._facerect[0][2]/2
            print "(x,y) = ",face_x,face_y
            print "depth : ",cv_image_depth[face_y][face_x]
            print
            if cv_image_depth[face_y][face_x] == 0 :
                pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)))
                return
            if cv_image_depth[face_y][face_x] > 1800 :
                pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.13)))
                return
            #print '顔が検出されました。'
            #顔角度の演算
            self._angle = (float(face_x)/float(self._frame.shape[1]))
            angle_person = (self._angle-0.5)*58.0
            print "Face Angle: ",angle_person
            self._color = (255, 255, 255) #白
            for self._rect in self._facerect:
                #検出した顔を囲む矩形の作成
                cv2.rectangle(self._frame, tuple(self._rect[0:2]),tuple(self._rect[0:2] + self._rect[2:4]), self._color, thickness=2)
            
            #現在の時間を取得
            self._now = datetime.now().strftime('%Y%m%d%H%M%S')
            #認識結果の保存
            self._image_path = 'face detect ' + self._now + '.jpg'
            cv2.imwrite(self._image_path, self._frame)
            #cv2.imshow('face detect', self._frame)
            #rad = angle_person*3.14159265359/180
            #顔認識フラグを立てる
            face_detect_flag = True
            
        else :
            face_detect_flag = False
        #顔検出フラグがFalseのとき回転,Trueのとき停止
        if face_detect_flag == False:
            pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.13)))
        elif angle_person > 20:
            pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.10)))
        elif angle_person > 2.5:
            pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.03)))
        elif angle_person < -20:
            pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.10)))
        elif angle_person < -2.5:
            pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.03)))
        else:
            pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)))
            pub_say.publish(True)
            detect_complete_flag = True
            print "Face Detect"
            #elif  count > 3:
            #    pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)))
            #elif angle_person > 0:
            #    pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-0.1)))
            #elif angle_person < 0:
            #    pub_vel.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.1)))
        #カスケード分類器のパス設定

def color(img):
    global t, c,count,detect_complete_flag
    if detect_complete_flag == False :
        convert=CvBridge()
        cv_image = convert.imgmsg_to_cv2(img,'bgr8') #colorの場合
        cv_image[240]=[255,0,0]
        cv_image[240,16]=[0,255,0]
        cv_image[:,320]=[255,0,0]
        
        #cv2.imshow("Color", cv_image)
        cv2.waitKey(1)
        #cv2.destroyAllWindows()
        #rospy.loginfo(rospy.get_caller_id() + "I good")
        th = FaceThread(cv_image)
        th.start()
    r = rospy.Rate(5) 
    r.sleep()

def depth(img):
    global cv_image_depth
    convert=CvBridge()
    cv_image_depth = convert.imgmsg_to_cv2(img,'16UC1') #colorの場合
    #cv2.imshow("Depth", cv_image_depth)
    cv2.waitKey(1)
    r = rospy.Rate(5) 
    r.sleep()
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def Recognition(true):
    global traj_flag
    if traj_flag == False:
        # 首角度移動情報の確定
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        #首角度[横/縦]
        p.positions = [0.0, 0.23]
        p.velocities = [0, 0]
        #目標角に達するまでの時間
        p.time_from_start = rospy.Time(2)
        traj.points = [p]
        #目標角情報のパブリッシュ
        pub_head.publish(traj)
        traj_flag = True
    rospy.Subscriber("hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", msg.Image,depth) #depthの場合
    rospy.Subscriber("hsrb/head_rgbd_sensor/rgb/image_color", msg.Image,color) #colorの場合
    
    #rospy.Subscriber("chatter", String, ss)
    rospy.spin()
  
if __name__ == '__main__':
    cur_dir = os.getcwd()
    os.chdir(cur_dir+"/image")
    #rospy.Subscriber("s_hdetection",Bool,Recognition)
    Recognition(True)
    rospy.spin()
