#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 15:41:54 2021

@author: robot
"""
from __future__ import print_function

#import roslib
#roslib.load_manifest('beginner_tutorials')
import message_filters
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import pickle
import math

class Camera_Lidar_Calibration:
  def __init__(self):
    self.image_pub = rospy.Publisher("object_range",Image)
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/camera/color/image_raw",Image)
    self.scan_sub = message_filters.Subscriber("/scan",LaserScan)
    self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.scan_sub], 10, 0.1, allow_headerless=True)
    self.ts.registerCallback(self.callback)
    with open('/home/robot/calibration/src/calibration/scripts/camera_lidar_calibration.p', "rb") as file:
        self.rvec = pickle.load(file)
        self.mtx = pickle.load(file)
        self.tvec = pickle.load(file)
    with open("/home/robot/drive/src/followingdrive/scripts/coco.names", "r") as f:
        self.classes = [line.strip() for line in f.readlines()]
    
  def callback(self, image, laser_scan):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
      lidar_camera = laser_scan.ranges[304:424]
#      
    except CvBridgeError as e:
      print(e)
#
    len_point = len(lidar_camera)
    point_lidar = []
    lidar_round = [0 for i in range(len_point)]
    
    for i in range(len_point):
        if lidar_camera[i] == float("inf"):
            lidar_round[i] = 40
        else:
            lidar = round(lidar_camera[i],3)
            lidar_round[i] = lidar
      
    ang = 152
    
    for i in range(0, len_point):
        x = round(lidar_round[i] * math.cos(ang*math.pi/180),3)
        y = round(lidar_round[i] * math.sin(ang*math.pi/180),3)
        z = 0
        xr = round(x,3)
        yr = round(y,3)
        ang = ang + 0.5
        point = [xr, yr, z]
        point_lidar.append(point)

    lidar_objs = np.array(point_lidar, dtype = "double")
    
    lidpoint, _ = cv2.projectPoints(lidar_objs, self.rvec, self.tvec, self.mtx, np.zeros(5))
    
    for i in range(0, len(lidpoint)):
        x = int(lidpoint[i][0][0])
        y = int(lidpoint[i][0][1])
        distance = lidar_round[i]
        if distance == 40:
          continue
        elif distance < 3:
          color = (0,0,255)
        elif distance < 5:
          color = (255, 0, 0)
        else:
          color = (0, 255, 0)
        cv2.circle(cv_image, (x, y), 1, color, 3)
 
    net = cv2.dnn.readNet("/home/robot/drive/src/followingdrive/scripts/yolov3-tiny.weights", "/home/robot/drive/src/followingdrive/scripts/yolov3-tiny.cfg")
    classes = []
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        
    height, width, channels = cv_image.shape
       
    # Detecting objects
    blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    #정보를 화면에 표시
    class_ids = []
    confidences = []
    boxes = []
    
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                # 좌표
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
            
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.45, 0.4) 
    #0.5 = confidence threshold o.4=NMS threshold
    box_distance = []
    x_2d = 0
    y_2d = 0
    
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            x_2d = x
            y_2d = y
            label = str(self.classes[class_ids[i]])
            score = confidences[i]
        # 경계상자와 클래스 정보 투영
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 4)
            cv2.circle(cv_image, (x+(w/2), y+(h/2)), 1, (0,0,255), 3)
            print('==================================')
            print('box_center', x+(w/2), y+(h/2))
            print('------')
            for k in range(0, len(lidpoint)):
                x_p = int(lidpoint[k][0][0])
                y_p = int(lidpoint[k][0][1])
                distance = lidar_round[k]
                if x_p > x and x_p < x+(w/2):
                    if distance != 40:
                        obj_distance = [x_p, y_p, distance]
                        box_distance.append(obj_distance)
    
    min_distance = 40
    num = 304
    min_angle = 0
    
    for i in range(0,len(box_distance)):            
        l = box_distance[i][2]
        if  l<min_distance:
            min_distance = l
            min_angle_lidar = (360-num)
            print(num)
            print(min_angle_lidar)
            min_angle = min_angle_lidar*-0.5
        num = num+1
        print('point : ', box_distance[i])  
    print('angle : ', min_angle)
    print('min_distance : ', min_distance)              
    print('end')
    print('==================================')
                
    cv2.imshow("Image window", cv_image) 
    cv2.waitKey(3)
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
        
def main(args):
  print(cv2.__version__)
  rospy.init_node('CL_Calibration', anonymous=True)  
  CLC = Camera_Lidar_Calibration()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
