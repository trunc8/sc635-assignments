#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import message_filters

def callback(names_msg, rolls_msg):
  names = names_msg.data
  name1, name2 = names.split("_")
  rolls = rolls_msg.data
  roll1, roll2 = rolls.split("_")
  print("Student %s has roll : %s \nStudent %s has roll : %s" % (name1, roll1, name2, roll2))

rospy.init_node("joint_subscriber_node")
rospy.loginfo("Subscriber created")

names_sub = message_filters.Subscriber("names_topic", String)
rolls_sub = message_filters.Subscriber("rolls_topic", String)
ts = message_filters.ApproximateTimeSynchronizer([names_sub, rolls_sub], queue_size=10, slop=0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.loginfo("Time synchronized subscriber object created")

rospy.loginfo("Beginning subscription...")
rospy.spin()