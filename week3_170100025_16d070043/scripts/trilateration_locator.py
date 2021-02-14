#!/usr/bin/env python
# trunc8 did this

# import csv
import math
import numpy as np
import rospy

from week3.msg import Trilateration
from week3_170100025_16d070043.msg import robot_pose


class Locator:
  def __init__(self):
    ## Below 4 variables will contain the 3 landmarks data
    self.land_x = None
    self.land_y = None
    self.land_dist = None
    self.land_var = None

    self.prev_pose = np.array([0.,0.])
    self.curr_pose = np.array([0.,0.])
    self.yaw = 0.
    self.prev_yaw = 0.

    ## Below 3 variables combat the Gaussian noise
    self.cum_curr_pose = [] # List of noisy poses from each trilateration message
    self.sample_index = 0
    self.sample_size = 10 # Average over 'n' consecutive messages

    self.rate = rospy.Rate(10)
    self.pub = rospy.Publisher('/robot_pose', robot_pose, queue_size=10)
    self.sub = rospy.Subscriber('/trilateration_data', Trilateration, self.trilaterateCallback)


  def trilaterateCallback(self, msg):
    '''
    Update landmark distances whenever subscriber receives message
    '''
    self.land_x = [msg.landmarkA.x, msg.landmarkB.x, msg.landmarkC.x]
    self.land_y = [msg.landmarkA.y, msg.landmarkB.y, msg.landmarkC.y]
    self.land_dist = [msg.landmarkA.distance, msg.landmarkB.distance, msg.landmarkC.distance]
    self.land_var = [msg.landmarkA.variance, msg.landmarkB.variance, msg.landmarkC.variance]

    self.sample_index += 1
    if (self.sample_index == self.sample_size):
      self.prev_pose[:] = self.curr_pose
      self.prev_yaw = self.yaw
      self.curr_pose = sum(self.cum_curr_pose)/len(self.cum_curr_pose)
      x_old, y_old = self.prev_pose
      x_new, y_new = self.curr_pose
      if (self.prev_pose == self.curr_pose).all():
        self.yaw = self.prev_yaw
      else:
        self.yaw = math.atan2(y_new-y_old, x_new-x_old)
      self.cum_curr_pose = []
      self.sample_index = 0
    else:
      ret, pose = self.getBestIntersection(
        self.land_x[0], self.land_y[0], self.land_dist[0],
        self.land_x[1], self.land_y[1], self.land_dist[1],
        self.land_x[2], self.land_y[2], self.land_dist[2]
      )
      if ret:
        self.cum_curr_pose.append(pose)


  def solveIntersection(self, x0,y0,r0, x1,y1,r1, X,Y,R):
    '''
    Solve for intersection points of circles 1 and 2 and return point lying closer to circle 3
    circle 1: (x0, y0), radius r0
    circle 2: (x1, y1), radius r1
    circle 3: (X,Y), radius R
    Reference for formulae: http://paulbourke.net/geometry/circlesphere/
    '''
    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
      rospy.logwarn("non intersecting")
      return 0, None
    # One circle within other
    if d < abs(r0-r1):
      rospy.logwarn("One circle within other")
      return 0, None
    # coincident circles
    if d == 0 and r0 == r1:
      rospy.logwarn("coincident circles")
      return 0, None
    else:
      a=(r0**2-r1**2+d**2)/(2*d)
      h=math.sqrt(r0**2-a**2)
      x2=x0+a*(x1-x0)/d   
      y2=y0+a*(y1-y0)/d   
      x3=x2+h*(y1-y0)/d     
      y3=y2-h*(x1-x0)/d 

      x4=x2-h*(y1-y0)/d
      y4=y2+h*(x1-x0)/d

    ## Checking which point is closer to circle 3
    if abs((x3-X)**2+(y3-Y)**2-R**2) < abs((x4-X)**2+(y4-Y)**2-R**2):
      return 1, np.array([round(x3,3),round(y3,3)])
    else:
      return 1, np.array([round(x4,3),round(y4,3)])


  def getBestIntersection(self, x0,y0,r0, x1,y1,r1, x2,y2,r2):
    '''
    The 3 circles won't exactly intersect due to noisy range data.
    Get average of the closest intersection points
    '''
    pts = []
    
    # ret: return status (success/fail)
    ret, pt = self.solveIntersection(x0,y0,r0, x1,y1,r1, x2,y2,r2)
    if ret:
      pts.append(pt)
    ret, pt = self.solveIntersection(x2,y2,r2, x0,y0,r0, x1,y1,r1)
    if ret:
      pts.append(pt)
    ret, pt = self.solveIntersection(x1,y1,r1, x2,y2,r2, x0,y0,r0)
    if ret:
      pts.append(pt)
    
    if len(pts)!=0:
      return 1, sum(pts)/len(pts)
    else:
      return 0, None


  def beginLocating(self):
    '''
    Compute robot pose using positions and distances of 3 landmarks
    '''
    while self.land_x == None: # Ensure that we have subscribed at least once
      pass

    while not rospy.is_shutdown():
      pose_msg = robot_pose()
      pose_msg.x, pose_msg.y = self.curr_pose
      pose_msg.yaw = self.yaw
      self.pub.publish(pose_msg)
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node('trilateration_locator_node')
  rospy.loginfo("trilateration_locator_node created")
  try:
    locator = Locator()
    while (rospy.get_time()==0): # Wait until node has loaded completely
      pass

    rospy.loginfo("Beginning the locator routine...")
    locator.beginLocating()

  except rospy.ROSInterruptException:
    pass