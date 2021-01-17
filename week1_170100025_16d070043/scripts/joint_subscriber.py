#! /usr/bin/env python

import rospy
from std_msgs.msg import String

class Team:
  def __init__(self):
    self.names = None
    self.rolls = None

  def namesCallback(self, msg):
    self.names = msg.data
    self.getMembers()

  def rollsCallback(self, msg):
    self.rolls = msg.data
    self.getMembers()

  def getMembers(self):
    if self.names!=None and self.rolls!=None:
      name1, name2 = self.names.split("_")
      roll1, roll2 = self.rolls.split("_")
      print("Student %s has roll : %s \nStudent %s has roll : %s" % (name1, roll1, name2, roll2))

rospy.init_node("joint_subscriber_node")
rospy.loginfo("Subscriber created")

team = Team()

while (rospy.get_time()==0): # Wait until node has loaded completely
  pass

rospy.Subscriber("names_topic", String, team.namesCallback)
rospy.Subscriber("rolls_topic", String, team.rollsCallback)

rospy.loginfo("Beginning subscription...")
rospy.spin()