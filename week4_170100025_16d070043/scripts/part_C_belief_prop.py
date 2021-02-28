#!/usr/bin/env python
# trunc8 did this

import rospy
import itertools
import matplotlib.pyplot as plt
import numpy as np
import os, sys

from geometry_msgs.msg import Twist


GRID_SIZE = 200       # Number of divisions of 2 meters
THETA_DIVISIONS = 360 # Number of divisions of 360 degrees

class Bel_Propagation:
  def propagateBelief(self, bel_p, u):
    global GRID_SIZE, THETA_DIVISIONS
    '''
    Inputs:
    bel_p= older belief (2D numpy array across positions in 200x200 grid)
    u= control input (v and w)
    
    Returns:
    bel= updated belief

    Notation:
    bel_p = bel'
    row_p, col_p, theta_p = row', col', theta'
    '''
    normalizer = 0
    bel = np.zeros(bel_p.shape)
    
    # Ignoring gamma error
    for row, col in itertools.product(range(GRID_SIZE), range(GRID_SIZE)):
      # (row, column) = (100-y, x)
      x = col
      y = GRID_SIZE//2-row
      pose = [x,y]
      for row_p, col_p in itertools.product(range(GRID_SIZE), range(GRID_SIZE)):
        # Use np.where()
        if (bel_p[row_p, col_p]!=0):
          theta_prob_sum = 0
          for theta_p in range(0, 360, THETA_DIVISIONS):
            x_p = col_p
            y_p = GRID_SIZE//2 - row_p
            pose_p = [x_p, y_p, theta_p]
            theta_prob_sum += self.getMotionModelProbability(pose, u, pose_p)
          bel[row, col] = theta_prob_sum*bel_p[row_p, col_p]
    return bel

    

  def getMotionModelProbability(self, pose, u, pose_p):
    return 0


class Velocity_Publisher:
  def __init__(self):
    global GRID_SIZE
    '''
    start_time = store the first rospy.get_time() as reference
    u = control input (v and w)
    bel = belief distribution across all the grid cells
    '''
    # ALERT: Take care if I'm running into VELOCITY_TIMEOUT at the end
    self.rate=rospy.Rate(20)
    self.pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=10)
    self.start_time = 0
    self.u = [0,0]

    self.t1 = 2     # seconds
    self.v1 = 0.1   # m/s
    self.w1 = 0     # rad/s

    self.t2 = 2     # seconds
    self.v2 = 0.1   # m/s
    self.w2 = 0.1   # rad/s

    self.bel_propagator = Bel_Propagation()
    self.bel = np.zeros([GRID_SIZE,GRID_SIZE])
    self.bel[GRID_SIZE//2,0] = 1

  def plotBelief(self, time):
    '''
    Plots current value of robot's state belief
    '''
    plt.imshow(self.bel, interpolation='none')
    plt.colorbar(label="Probability")
    plt.clim(vmin=0, vmax=1) # Ensure that colorbar limits are 0 and 1
    plt.title('Robot Belief (t={})'.format(time))
    plt.savefig(os.path.join(sys.path[0],
                '../images/Robot_Belief_t={}.png'.format(time)))
    plt.close("all")

  def publishTwist(self):
    '''
    First carries out linear motion for time t1
    Then performs arc motion for time t2
    Finally stops
    '''
    t1_completed = False
    t2_completed = False
    self.plotBelief("0")

    ## Enter linear mode
    velocity_msg = Twist()
    self.u = [self.v1, self.w1]
    velocity_msg.linear.x = self.u[0]
    velocity_msg.angular.z = self.u[1]

    while not rospy.is_shutdown():
      if (not t1_completed) and ((rospy.get_time()-self.start_time) >=
                                 self.t1):
        # Runs only once when timer hits t=t1
        ## Switches from linear mode to arc mode
        t1_completed = True
        self.plotBelief("t1")
        self.u = [self.v2, self.w2]
        velocity_msg.linear.x = self.u[0]
        velocity_msg.angular.z = self.u[1]
      
      elif (not t2_completed) and ((rospy.get_time()-self.start_time) >=
                                   self.t1+self.t2):
        # Runs only once when timer hits t=t1+t2
        ## Switches from arc mode to stop mode
        t2_completed = True
        self.plotBelief("t1+t2")
        self.u = [0, 0]
        velocity_msg.linear.x = self.u[0]
        velocity_msg.angular.z = self.u[1]
        return
      
      self.pub.publish(velocity_msg)
      rospy.loginfo("published")
      self.bel = self.bel_propagator.propagateBelief(self.bel, self.u)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('bel_propagator_node')
  rospy.loginfo("bel_propagator_node created")
  
  try:
    velocity_publisher = Velocity_Publisher()
    rospy.sleep(3)
    
    while (velocity_publisher.start_time == 0):
      # Wait until node has loaded completely
      velocity_publisher.start_time = rospy.get_time()
    
    rospy.loginfo("Beginning to publish...")
    velocity_publisher.publishTwist()

  except rospy.ROSInterruptException:
    pass