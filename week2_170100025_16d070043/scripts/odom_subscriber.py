#!/usr/bin/env python
# trunc8 did this

import csv
import math
import numpy as np
import os, sys
import rospy
import tf

from nav_msgs.msg import Odometry
from week2_170100025_16d070043.msg import pose_error

class Odom_Subscriber:
  def __init__(self):
    self.waypoints = []
    self.index = 0
    self.current_x = 0
    self.current_y = 0
    self.current_theta = 0
    self.rate = rospy.Rate(10)
    self.pub = rospy.Publisher('/pose_error', pose_error, queue_size=10)
    self.sub = rospy.Subscriber('/odom', Odometry, self.odomCallback)

  def odomCallback(self, msg):
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    quaternion = [x,y,z,w]    

    self.current_x = msg.pose.pose.position.x
    self.current_y = msg.pose.pose.position.y
    self.current_theta = tf.transformations.euler_from_quaternion(quaternion)[2]

  def publishError(self):
    while not rospy.is_shutdown():
      if (self.index >= len(self.waypoints)):
        rospy.signal_shutdown("Destination reached!")
        return

      target_x = float(self.waypoints[self.index][0])
      target_y = float(self.waypoints[self.index][1])
      print("{:.3f},{:.3f}\tTarget:{:.3f},{:.3f}".format(self.current_x, self.current_y, target_x, target_y))

      error_x = target_x - self.current_x
      error_y = target_y - self.current_y
      E_pos = np.linalg.norm([error_x, error_y])
      desired_theta = math.atan2(error_y, error_x)
      E_theta = desired_theta - self.current_theta
      if (E_theta > math.pi):
        E_theta -= 2*math.pi
      elif (E_theta < -math.pi):
        E_theta += 2*math.pi

      msg = pose_error()
      msg.E_pos = E_pos
      msg.E_theta = E_theta

      if (E_pos < 0.1):
        self.index += 1

      self.pub.publish(msg)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('odom_subscriber_node')
  rospy.loginfo("odom_subscriber_node created")
  try:
    odom_subscriber = Odom_Subscriber()
    
    ## Read all waypoints
    csv_reader = csv.reader(open(os.path.join(sys.path[0], 'waypoints.txt')))
    for line in csv_reader:
      odom_subscriber.waypoints.append([line[0],line[1]])
    
    ## Wait until node has loaded completely
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("Beginning to publish error...")
    odom_subscriber.publishError()

  except rospy.ROSInterruptException:
    pass