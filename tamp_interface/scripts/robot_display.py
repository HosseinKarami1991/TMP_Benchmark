#!/usr/bin/env python

import signal
import sys
import time
import roslib
import rospy
import tf
import numpy as np
import rospy
from std_msgs.msg import String,Bool
import roslaunch
import subprocess  # as child
import os
import argparse
import cv2
import cv_bridge
from sensor_msgs.msg import Image as Image_Sensor_Msg
import copy as copy_module

import Image
import ImageDraw
import ImageFont
import cStringIO
from PIL import Image as ImagePIL
kinect_image = Image_Sensor_Msg
lastdata = None
index = None
topub = False

def signal_handler(signal, frame):
    print('Bye!')
    sys.exit(0)

def callbackkiectimage(data):
    #rospy.loginfo("********* Subscribing to kinect camera image color But still need permission ***** ")
    #print("now we can publish: %s", str(topub))
    if topub==True:
        #rospy.loginfo("********* Publishing kinect camera image color to display ***** ")
        pub.publish(data)


def callbackkinect(data):
    #print("Received msg to publish: %s", str(data.data))
    global topub
    topub = data.data
    if not topub:
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub.publish(msg)

    #callbackkiectimage(data,topub)








def callback2(data):
    #rospy.loginfo("*********Done action for kinestheticTeaching arrived ***** %s", data.data)
    #rospy.loginfo("items of lastdata are:")
    for item in lastdata:
        print(item)
    doneaction = data.data
    font2 = cv2.FONT_HERSHEY_PLAIN
    index_list=[]
    for i,item in enumerate(lastdata):
        if len(item.split('-'))<2:
            index_list.append(i)
        else:
            mainparam = item.split('-')[1]
            #rospy.loginfo("********  mainparam is ************ %s", mainparam)
            #rospy.loginfo("********  doneaction is ************ %s", doneaction)
            if doneaction==mainparam:
                global index
                index = i
                #rospy.loginfo("******** index is ************ %s", index)
                break
    img2 = np.copy(img)
    cv2.putText(img2, "Kinesthetic Teaching", (30,150), font2, 5.5, (255, 255, 255), 4)
    for i in range(1,len(lastdata)):
        if i<=index+1 or i in index_list:
            st = str(i) +") " + lastdata[i-1]
            cv2.putText(img2, st, (100, (i-1)*50+250), font2, 3, (0, 0, 255), 4)
        else:
            st = str(i) +") " + lastdata[i-1]
            cv2.putText(img2, st, (100, (i-1)*50+250), font2, 3, (0, 255, 0), 4)


    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img2, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.1)
    del index




def callback(data):
    #rospy.loginfo("robot display: I heard %s", data.data)
    DataList = [str(x) for x in data.data.split('+')][0:]
    img2 = np.copy(img)
    global lastdata
    #lastdata = DataList
    font2 = cv2.FONT_HERSHEY_PLAIN
    if len(DataList) > 1:
        topub = False
        lastdata = list(DataList)
        #rospy.loginfo("items of lastdata are:")
        for item in lastdata:
            print(item)
        cv2.putText(img2, "Kinesthetic Teaching", (30,150), font2, 5.5, (255, 255, 255), 4)
        for i in range(1,len(DataList)):
            st = str(i) +") " + DataList[i-1]
            cv2.putText(img2, st, (100, (i-1)*50+250), font2, 3, (0, 255, 0), 4)
            
        
    else:
        DataList = data.data
    

        #print(len(DataList[0]), ": ", DataList)
        #print("im in change")
        if DataList[1] == "Human":
            #print("im in Human")
            cv2.putText(img2, DataList[1], (400, 400), font2, 4, (0, 255, 0), 4)
            cv2.putText(img2, DataList[0], (100, 500), font2, 2, (0, 255, 0), 4)
        elif DataList[1] == "LeftArm" or DataList[1] == "RightArm" or DataList[1] == "LeftArm+RightArm" or DataList[
            1] == "RightArm+LeftArm":
            cv2.putText(img2, DataList[1], (400, 400), font2, 4, (0, 0, 255), 4)
            #print("im in Human")

            if len(DataList[0]) > 50:
                print("im in DataList[0])>50")
                cv2.putText(img2, DataList[0], (10, 500), font2, 2, (0, 0, 255), 4)
            else:
                #print("im in DataList[0])<50")
                cv2.putText(img2, DataList[0], (100, 500), font2, 2, (0, 0, 255), 2)
        else:
            #print("im in first else")
            if DataList == "not_faulty":
                #print("im in von faulty")
                cv2.putText(img2, "Non_Faulty", (70, 350), font, 5, (255, 255, 255), 10, cv2.LINE_AA)
            elif DataList == "NA":
                cv2.putText(img2, DataList, (300, 450), font, 10, (255, 255, 255), 10, cv2.LINE_AA)
            elif DataList == "faulty":
                cv2.putText(img2, "Faulty", (200, 350), font, 7, (255, 255, 255), 10, cv2.LINE_AA)
            elif DataList == "kinestheticTeaching":
                #print("im in kinestheticTeaching")
                cv2.putText(img2, "Kinesthetic", (120, 300), font, 5, (255, 255, 255), 10)
                cv2.putText(img2, "Teaching", (170, 450), font, 5, (255, 255, 255), 10)
            elif DataList == "NormalGraph":
                #print("im in NormalGraph")
                cv2.putText(img2, "Normal", (190, 300), font, 5, (255, 255, 255), 10)
                cv2.putText(img2, "Graph", (210, 450), font, 5, (255, 255, 255), 10)
            elif DataList == "previousstate":
            #print("im in NormalGraph")
                cv2.putText(img2, "Previous", (170, 300), font, 5, (0, 255, 0), 10)
                cv2.putText(img2, "State", (220, 450), font, 5, (0, 255, 0), 10)

            else:
                #print("im in else")
                cv2.putText(img2, DataList, (100, 350), font, 5, (255, 0, 0), 10)

    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img2, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.1)


def main():
    rospy.init_node('robot_display')
    global pub
    #global topub
    topub = False
    pub = rospy.Publisher('/robot/xdisplay', Image_Sensor_Msg, latch=True, queue_size=1)
    Sub = rospy.Subscriber('robotDisplayText', String, callback)
    sub2 =rospy.Subscriber('/done_actions',String,callback2)
    subkinect =rospy.Subscriber('/pub_kinect',Bool,callbackkinect)
    kinectimage =rospy.Subscriber('/camera/rgb/image_color',Image_Sensor_Msg,callbackkiectimage)

    path = '/home/hossein/catkin_ws/src/ROBOT_INTERFACE/robot_interface/files/black_Background.png'
    #global lastdata
    #lastdata = []
    global img
    #global index
    #index=0
    img = cv2.imread(path)
    global font
    font = cv2.FONT_HERSHEY_SIMPLEX

    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.1)

    rospy.spin()
    signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    main()

