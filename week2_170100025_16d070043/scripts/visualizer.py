#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
import os
import rospy
import sys
import tf

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class Visualizer:
  def __init__(self):
    self.waypoints = []
    self.tracking_points = []
    self.index = 0
    self.current_x = 0
    self.current_y = 0
    self.current_theta = 0
    # self.sub = rospy.Subscriber('/odom', Odometry, self.odomCallback)
    self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb)


  def cb(self, msg):
    # x = msg.pose.pose.orientation.x
    # y = msg.pose.pose.orientation.y
    # z = msg.pose.pose.orientation.z
    # w = msg.pose.pose.orientation.w
    # quaternion = [x,y,z,w]    

    self.current_x = msg.pose[1].position.x
    self.current_y = msg.pose[1].position.y
    # self.current_theta = tf.transformations.euler_from_quaternion(quaternion)[2]
    self.tracking_points.append([self.current_x, self.current_y])


if __name__ == '__main__':
  rospy.init_node('visualizer_node')
  rospy.loginfo("visualizer_node created")
  vis = Visualizer()
  try:    
    ## Read all waypoints
    csv_reader = csv.reader(open(os.path.join(sys.path[0], 'waypoints.txt')))
    for line in csv_reader:
      vis.waypoints.append([line[0],line[1]])
    
    ## Wait until node has loaded completely
    while (rospy.get_time()==0): 
      pass
    
    while not rospy.is_shutdown():
      rospy.spin()

  except rospy.ROSInterruptException:
    pass

  ## Plot all waypoints and track points together
  finally:    
    fig, ax = plt.subplots(1)
    for w in vis.waypoints:
      ax.plot(w[0], w[1], 'b.', ms=12, label='Waypoints')
    X=[]
    Y=[]
    for pt in vis.tracking_points:
      X.append(pt[0])
      Y.append(pt[1])
    ax.plot(X, Y, 'c', lw=3, label='Track points')
    plt.xlim([-5, 5])
    plt.ylim([-5, 5])
    plt.xlabel('X (in meters)')
    plt.ylabel('Y (in meters)')
    plt.title('Robot Motion')
    plt.savefig(os.path.join(sys.path[0],'../images/Robot_Motion.png'))
