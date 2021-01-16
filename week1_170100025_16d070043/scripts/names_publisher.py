#! /usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node("names_publisher_node")
rospy.loginfo("Node created")

pub = rospy.Publisher("names_topic", String, queue_size=10)
rospy.loginfo("Publisher object created")

# Create variable to store names
names = String()
names.data = "Siddharth Saha_Kumar Ashutosh"

# Publish rate of 2 Hz
rate = rospy.Rate(2)

rospy.loginfo("Beginning to publish...")
while not rospy.is_shutdown():
  pub.publish(names)
  rate.sleep()