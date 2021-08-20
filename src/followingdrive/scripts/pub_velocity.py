#!/usr/bin/env python
#-*-coding: utf-8-*-
from __future__ import print_function

#import roslib
#roslib.load_manifest('beginner_tutorials')
import sys
import rospy
import cv2
from followingdrive.msg import angle_distance
from geometry_msgs.msg import Twist
import time
import numpy as np

class Velocity:

  def __init__(self):
    self.twist_pub = rospy.Publisher("cmd_cel",Twist, queue size = 1)
    self.ad_msg = angle_distance()
    self.angle_sub = rospy.Subscriber("angle_distance",angle_distance,self.callback)
    self.last_time = rospy.Time.now()
    self.send_time = rospy.Time.now()
    
  def callback(self,data):
    try:
      distance = self.ad_msg.distance
      angle = self.ad_msg.angle
      
    except:
      print(error)
    if dis == 40 and ang ==0:
      return # unknown key.
  
    between = 4 - dis
    linear_ = btween
    angular_vel = ang
    vels = [linear_vel, angular_vel]
    g_target_twist.angular.z = vels[0]
    g_target_twist.linear.x  = vels[1]
    
    try:
      self.twist_pub.publish(g_target_twist)
    except  as e:
      print(e)
      
def main(args):
  rospy.init_node('to_twist', anonymous=True)  
  Vel = Velocity()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__': 
    main(sys.argv)
