#!/usr/bin/env python
# trunc8 did this

import rospy

from geometry_msgs.msg import Twist


class Velocity_Publisher:
  def __init__(self):
    self.rate=rospy.Rate(20)
    self.pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=10)
    self.v = 0.11
    self.w = 0.8


  def publishConstTwist(self):
    while not rospy.is_shutdown():
      velocity_msg = Twist()
      velocity_msg.linear.x = self.v
      velocity_msg.angular.z = self.w
      self.pub.publish(velocity_msg)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('const_vel_publisher_node')
  rospy.loginfo("const_vel_publisher_node created")
  try:
    velocity_publisher = Velocity_Publisher()
    while (rospy.get_time()==0): # Wait until node has loaded completely
      pass
    rospy.loginfo("Beginning to publish...")
    velocity_publisher.publishConstTwist()

  except rospy.ROSInterruptException:
    pass