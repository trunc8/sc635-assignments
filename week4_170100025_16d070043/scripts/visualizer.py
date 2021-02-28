#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
import math
import os
import rospy
import sys
import tf

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class Visualizer:
  def __init__(self):
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

    self.current_x = msg.pose[-1].position.x
    self.current_y = msg.pose[-1].position.y
    # self.current_theta = tf.transformations.euler_from_quaternion(quaternion)[2]
    self.tracking_points.append([self.current_x, self.current_y])


if __name__ == '__main__':
  rospy.init_node('visualizer_node')
  rospy.loginfo("visualizer_node created")
  vis = Visualizer()
  curr_seconds = 0
  try:
    ## Wait until node has loaded completely
    while (curr_seconds==0): 
      curr_seconds = rospy.get_time()

    while not rospy.is_shutdown():
      rospy.spin()

  except rospy.ROSInterruptException:
    pass

  ## Plot all waypoints and track points together
  finally:    
    fig, ax = plt.subplots(1)
    X=[]
    Y=[]
    for pt in vis.tracking_points:
      X.append(pt[0])
      Y.append(pt[1])
    ax.plot(X, Y, 'c', lw=3, label='Track points')
    plt.axis('equal')
    plt.xlim([-1, 1])
    plt.ylim([-1, 1])
    plt.xlabel('X (in meters)')
    plt.ylabel('Y (in meters)')
    plt.title('Robot Motion')
    plt.savefig(os.path.join(sys.path[0],'../images/Open_Loop_Robot_Motion.png'))

    # Time elapsed
    elapsed = rospy.get_time()-curr_seconds
    print("Elapsed time: {:.2f} seconds".format(elapsed))
