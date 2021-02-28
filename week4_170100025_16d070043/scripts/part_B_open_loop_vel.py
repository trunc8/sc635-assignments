#!/usr/bin/env python
# trunc8 did this

import rospy

from geometry_msgs.msg import Twist


class Velocity_Publisher:
  def __init__(self):
    '''
    Reason for choosing Rate = 1Hz -->
    Each pub command times out in 0.6s
    Thus for open loop control, we must publish at a rate
    slower than (1/0.6s) = 1.67Hz to ensure that each
    velocity command executes completely
    '''
    self.rate=rospy.Rate(1)
    self.pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=10)
    self.num_of_steps = 20
    self.v = 0.5/(0.6*self.num_of_steps) # ~0.8m/s
    self.w = 0


  def publishOpenLoopTwist(self):
    counter = 0
    while not rospy.is_shutdown():
      if counter >= self.num_of_steps:
        return
      counter += 1
      
      velocity_msg = Twist()
      velocity_msg.linear.x = self.v
      velocity_msg.angular.z = self.w
      
      self.pub.publish(velocity_msg)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('open_loop_vel_publisher_node')
  rospy.loginfo("open_loop_vel_publisher_node created")
  try:
    velocity_publisher = Velocity_Publisher()
    while (rospy.get_time()==0): # Wait until node has loaded completely
      pass
    rospy.sleep(5.)
    rospy.loginfo("Beginning to publish...")
    velocity_publisher.publishOpenLoopTwist()

  except rospy.ROSInterruptException:
    pass