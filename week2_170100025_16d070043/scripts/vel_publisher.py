#!/usr/bin/env python
# trunc8 did this

import math
import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from week2_170100025_16d070043.msg import pose_error

class Velocity_Publisher:
  def __init__(self):
    self.rate=rospy.Rate(10)
    self.E_pos = 0
    self.E_theta = 0
    self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    self.sub = rospy.Subscriber('/pose_error', pose_error, self.poseErrorCallback)

  def poseErrorCallback(self, msg):
    self.E_pos = msg.E_pos
    self.E_theta = msg.E_theta

  def controlLoop(self):
    while not rospy.is_shutdown():
      velocity_msg = Twist()
      max_angular_vel = 0.5
      max_linear_vel = 0.5
      K1 = 0.5
      K2 = 0.5
      if (np.abs(self.E_theta) > 0.1):
        velocity_msg.linear.x = 0.02*self.E_pos
        velocity_msg.angular.z = K1*self.E_theta
        rospy.loginfo("Orienting")
      elif (self.E_pos > 0.1):
        velocity_msg.linear.x = K2*self.E_pos
        velocity_msg.angular.z = 0.01*self.E_theta
        rospy.loginfo("Chasing")
      else:
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        rospy.loginfo("Your destination has arrived!")

      # K1 = 0.2
      # K2 = -0.2
      # velocity_msg.linear.x = K1*self.E_pos
      # velocity_msg.angular.z = -K2*self.E_theta
      
      velocity_msg.linear.x = min(max_linear_vel, velocity_msg.linear.x)
      velocity_msg.angular.z = np.clip(velocity_msg.angular.z, -max_angular_vel, max_angular_vel)
      self.pub.publish(velocity_msg)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('vel_publisher_node')
  rospy.loginfo("vel_publisher_node created")
  try:
    velocity_publisher = Velocity_Publisher()
    while (rospy.get_time()==0): # Wait until node has loaded completely
      pass
    rospy.loginfo("Beginning control loop...")
    velocity_publisher.controlLoop()

  except rospy.ROSInterruptException:
    pass