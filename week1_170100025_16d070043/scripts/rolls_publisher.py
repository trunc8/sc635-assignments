#! /usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node("roll_publisher_node")
rospy.loginfo("Publisher node created")

pub = rospy.Publisher("rolls_topic", String, queue_size=10)
rospy.loginfo("Publisher object created")

# Create variable to store names
names = String()
names.data = "170100025_16d070043"

# Publish rate of 2 Hz
rate = rospy.Rate(2)

rospy.loginfo("Beginning to publish...")
while not rospy.is_shutdown():
  pub.publish(names)
  rate.sleep()