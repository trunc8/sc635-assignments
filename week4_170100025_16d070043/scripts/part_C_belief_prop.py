#!/usr/bin/env python
# trunc8 did this

from __future__ import print_function

import rospy
import itertools
import matplotlib.pyplot as plt
import numpy as np
import os, sys

from geometry_msgs.msg import Twist
from week5.msg import Pose


GRID_SIZE = 200       # Number of divisions of 2 meters
# THETA_DIVISIONS = 360 # Number of divisions of 360 degrees

class Bel_Propagation:
  def __init__(self):
    self.prev_time = 0
    self.del_time = 0

  def propagateBelief(self, bel_p, u, theta):
    global GRID_SIZE
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
    rospy.loginfo("STARTED >>>>>>>>>>>>>>>>>>>>>>")
    bel = np.zeros(bel_p.shape)
    nonzero_row, nonzero_col = np.nonzero(bel_p)
    if len(nonzero_row)==0:
      rospy.logerr("All zero alert")

    curr_time = rospy.get_time()
    self.del_time = curr_time - self.prev_time
    self.prev_time = curr_time
    
    '''
    Note: We overlook gamma error calculation to reduce computation.
    We are using theta obtained from /manual_odom topic.
    '''
    print("Progress percent:")
    for row, col in itertools.product(range(GRID_SIZE), range(GRID_SIZE)):
      # (row, column) = (100-Y, X)
      x = 2.0*col/GRID_SIZE                 # Range: [0,2]
      y = (GRID_SIZE//2-row)*1.0/GRID_SIZE  # Range: [-1,1]
      pose = [x,y,theta]
      if col%GRID_SIZE == 0 and row % 20 == 0:
        print("{:.2f}%...\t ".format(100*row/GRID_SIZE), end='')
      
      for row_p, col_p in zip(nonzero_row, nonzero_col):
          x_p = 2*col_p/GRID_SIZE
          y_p = (GRID_SIZE//2 - row_p)/GRID_SIZE
          # ASSUMPTION: We're passing the same theta to reduce computation
          pose_p = [x_p, y_p, theta]
          motion_prob = self.getMotionModelProbability(pose, u, pose_p)
          bel[row, col] = motion_prob*bel_p[row_p, col_p]
    # Normalizing since belief over entire space should sum to 1
    bel = bel/np.sum(bel)
    print()
    rospy.loginfo("<<<<<<<<<<<<<<<<<<<<< FINISHED")
    return bel
      

  def getMotionModelProbability(self, pose, u, pose_p):
    x, y, theta = pose
    x_p, y_p, theta_p = pose_p
    
    EPS = 1e-4
    if abs((y-y_p)*np.cos(theta)-(x-x_p)*np.sin(theta)) > EPS:
      # if angular velocity is non-zero
      mu = 0.5*((x-x_p)*np.cos(theta)+(y-y_p)*np.sin(theta)) / (
                (y-y_p)*np.cos(theta)-(x-x_p)*np.sin(theta))
      x_center = 0.5*(x+x_p) + mu*(y-y_p)
      y_center = 0.5*(y+y_p) + mu*(x_p-x)
      radius = np.sqrt((x-x_center)**2 + (y-y_center)**2)
      del_theta = np.arctan2(y_p-y_center,x_p-x_center) - np.arctan2(y-y_center,x-x_center)
      v_hat = radius*abs(del_theta)/self.del_time
      w_hat = del_theta/self.del_time
      gamma_hat = (theta_p-theta)/self.del_time - w_hat
    else:
      # else pure translation
      v_hat = np.linalg.norm([x-x_p,y-y_p])/self.del_time
      w_hat = 0
      gamma_hat = (theta_p-theta)/self.del_time - w_hat

    alp1 = alp2 = alp3 = alp4 = alp5 = alp6 = 0.5
    P1 = self.prob(u[0]-v_hat, alp1*abs(u[0])+alp2*abs(u[1]))
    P2 = self.prob(u[1]-w_hat, alp3*abs(u[0])+alp4*abs(u[1]))
    P3 = self.prob(gamma_hat, alp5*abs(u[0])+alp6*abs(u[1]))
    return P1*P2*P3


  def prob(self, a,b):
    '''
    Triangular Distribution
    Notation: b2 = b^2 (variance)
    '''
    b2 = b**2
    return max(0, 1/np.sqrt(6*b2) - abs(a)/(6*b2))


class Velocity_Publisher:
  def __init__(self):
    global GRID_SIZE
    '''
    start_time = store the first rospy.get_time() as reference
    u = control input (v and w)
    bel = belief distribution across all the grid cells
    '''
    self.rate=rospy.Rate(10)
    self.pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=10)
    self.start_time = 0
    self.u = [0,0]

    self.t1 = 2     # seconds
    self.v1 = 0.1   # m/s
    self.w1 = 0     # rad/s

    self.t2 = 2     # seconds
    self.v2 = 0.1   # m/s
    self.w2 = 0.1   # rad/s

    # An object of class Bel_Propagation will be contained in 
    # Velocity_Publisher object
    self.bel_propagator = Bel_Propagation()
    self.bel = np.zeros([GRID_SIZE,GRID_SIZE])
    self.bel[GRID_SIZE//2,0] = 1

    self.sub = rospy.Subscriber('/manual_odom', Pose, self.updateTheta)
    self.theta = 0

  def updateTheta(self, msg):
    self.theta = msg.theta

  def plotBelief(self, time):
    '''
    Plots current value of robot's state belief
    '''
    plt.imshow(self.bel, interpolation='none')
    plt.colorbar(label="Probability")
    # plt.clim(vmin=0, vmax=1) # Ensure that colorbar limits are 0 and 1
    plt.title('Robot Belief (t={})'.format(time))
    plt.savefig(os.path.join(sys.path[0],
                '../images/Robot_Belief_t={}.png'.format(time)))
    plt.close("all")
    rospy.loginfo("Plot Ready! (written in images directory)")

  def publishTwist(self):
    '''
    First carries out linear motion for time t1
    Then performs arc motion for time t2
    Finally stops
    '''
    t1_completed = False
    t2_completed = False
    self.plotBelief("0")
    self.bel_propagator.prev_time = rospy.get_time()

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
        self.bel = self.bel_propagator.propagateBelief(self.bel,
                                                       self.u,
                                                       self.theta)
        self.plotBelief("t1")
        self.u = [self.v2, self.w2]
        velocity_msg.linear.x = self.u[0]
        velocity_msg.angular.z = self.u[1]
        self.start_time = rospy.get_time() # Reset timer

        # In order to ease computation,
        # we will carry forward only top N belief points
        N = 10
        # Convert to 1D array, pick top N
        # Reconvert to 2D array and store indices
        idx = np.unravel_index(np.argpartition(
                self.bel.ravel(),-N)[-N:], self.bel.shape)
        bel_copy = np.zeros(self.bel.shape)
        bel_copy[idx] = self.bel[idx]
        # Normalize and re-assign to variable 'bel'
        self.bel = bel_copy/np.sum(bel_copy)
        self.plotBelief("t1_after_filtering")
      
      elif (not t2_completed) and ((rospy.get_time()-self.start_time) >=
                                   self.t2):
        # Runs only once when timer hits t=t1+t2
        ## Switches from arc mode to stop mode
        t2_completed = True
        self.bel = self.bel_propagator.propagateBelief(self.bel,
                                                       self.u,
                                                       self.theta)
        self.plotBelief("t1+t2")
        self.u = [0, 0]
        velocity_msg.linear.x = self.u[0]
        velocity_msg.angular.z = self.u[1]
        return
      
      self.pub.publish(velocity_msg)
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




# def propagateBelief(self, bel_p, u):
  #   global GRID_SIZE, THETA_DIVISIONS
  #   '''
  #   Inputs:
  #   bel_p= older belief (2D numpy array across positions in 200x200 grid)
  #   u= control input (v and w)
    
  #   Returns:
  #   bel= updated belief

  #   Notation:
  #   bel_p = bel'
  #   row_p, col_p, theta_p = row', col', theta'
  #   '''
  #   normalizer = 0
  #   bel = np.zeros(bel_p.shape)
    
  #   # Ignoring gamma error
  #   for row, col in itertools.product(range(GRID_SIZE), range(GRID_SIZE)):
  #     # (row, column) = (100-y, x)
  #     x = col
  #     y = GRID_SIZE//2-row
  #     pose = [x,y]
  #     for row_p, col_p in itertools.product(range(GRID_SIZE), range(GRID_SIZE)):
  #       # Use np.where()
  #       if (bel_p[row_p, col_p]!=0):
  #         theta_prob_sum = 0
  #         for theta_p in range(0, 360, THETA_DIVISIONS):
  #           x_p = col_p
  #           y_p = GRID_SIZE//2 - row_p
  #           pose_p = [x_p, y_p, theta_p]
  #           theta_prob_sum += self.getMotionModelProbability(pose, u, pose_p)
  #         bel[row, col] = theta_prob_sum*bel_p[row_p, col_p]
  #   return bel