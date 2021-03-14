#!/usr/bin/env python
from __future__ import division
import csv, os, sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from math import atan, atan2, pi
import tf
import matplotlib.pyplot as plt

from week5.msg import Trilateration
from trilateration import varA, varB, varC

## Define global variables
pose = [0, 0, 0]
prev_pose = [0, 0, 0]
v, w = 0.0, 0.0 # Linear and angular velocity control inputs

## System noise related
noisy_pose = [0.0, 0.0, 0.0]
varTHETA, varX, varY = 0.01, 0.05, 0.05

Q = np.diag([varA, varB, varC]) ## Imported from trilateration
R = np.diag([varX, varY, varTHETA]) 
E = np.diag([0.5, 0.5, 0.5])  ## Some reasonable initial values

cov_1 = []
cov_2 = []
cov_3 = []

F = np.eye(3)                   ## System matrix for discretized unicycle is Identity 
H = np.zeros((3,2))             #get_current_H(pose, landmarkA, landmarkB, landmarkC)

distanceLandmarkA = 10.0
distanceLandmarkB = 10.0
distanceLandmarkC = 10.0

FILTER_ORDER= 10     ## Change filter settings
filter_a=[0 for i in range(FILTER_ORDER)]
filter_b=[0 for i in range(FILTER_ORDER)]
filter_c=[0 for i in range(FILTER_ORDER)]

idxA = 0
idxB = 0
idxC = 0

# positions of landmarks
landmark_A, landmark_B, landmark_C = [], [], []

theta = 0

odoo = Odometry()

def get_current_H(pose, lA, lB, lC):
  ### Calculate the linearized measurement matrix H(k+1|k) at the current robot pose ==> Done
  ## x and y co-ordinates of landmarks
  global landmark_A, landmark_B, landmark_C

  H = np.zeros((3,3))
  H[0,:2] = (pose[:2] - landmark_A)/dist(pose[:2], landmark_A)
  H[1,:2] = (pose[:2] - landmark_B)/dist(pose[:2], landmark_B)
  H[2,:2] = (pose[:2] - landmark_C)/dist(pose[:2], landmark_C)
  return H


def dist(p1, p2):
  ### Given a pair of points the function returns euclidean distance
  return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)


def predict_state(current_pose, current_v, current_w):
  ### System evolution 
  return noisy_pose


def predict_measurement(predicted_pose):
  ### Predicts the measurement (d1, d2, d3) given the current position of the robot
  global landmark_A, landmark_B, landmark_C
  d1 = dist(predicted_pose, landmark_A)
  d2 = dist(predicted_pose, landmark_B)
  d3 = dist(predicted_pose, landmark_C)
  measurement = [d1, d2, d3]
  measurement = np.array(measurement).reshape(3,1)
  return measurement


def callback2(data):
  global noisy_pose, varX, varY, varTHETA
  x  = data.pose.pose.orientation.x;
  y  = data.pose.pose.orientation.y;
  z = data.pose.pose.orientation.z;
  w = data.pose.pose.orientation.w;
  noise = [np.random.normal(0,varX), np.random.normal(0,varY), np.random.normal(0,varTHETA)]
  noisy_pose = np.array([data.pose.pose.position.x + noise[0], data.pose.pose.position.y + noise[1], tf.transformations.euler_from_quaternion([x,y,z,w])[2] + noise[2]]).reshape(3,1)
  return


def callback(data):
  global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
  global idxA, idxB, idxC
  global filter_a, filter_b, filter_c
  global landmark_A, landmark_B, landmark_C
  global prev_pose, theta, pose
  global E, Q, R, F, H
  global v, w

  lA = data.landmarkA
  lB = data.landmarkB
  lC = data.landmarkC

  ### Locations of landmarks
  landmark_A = [lA.x, lA.y]
  landmark_B = [lB.x, lB.y]
  landmark_C = [lC.x, lC.y]

  #######################################################
  ### FILTERING VALUES
  #######################################################
  ### Add value into r buffer at indices idxA, idxB, idxC
  filter_a[idxA] = lA.distance
  filter_b[idxB] = lB.distance
  filter_c[idxC] = lC.distance

  ### Increment indexes
  idxA += 1
  idxB += 1
  idxC += 1

  ### wrap around the indices if buffer full
  if idxA>=FILTER_ORDER:
    idxA=0
  if idxB>=FILTER_ORDER:
    idxB=0
  if idxC>=FILTER_ORDER:
    idxC=0

  ## Calculate filtered measurements (d1, d2, d3)  
  d1, d2, d3 = sum(filter_a)/FILTER_ORDER, sum(filter_b)/FILTER_ORDER, sum(filter_c)/FILTER_ORDER 

  Y_measured = np.matrix([[d1],[d2],[d3]])

  #### EXTENDED KALMAN FILTER CODE GOES BELOW

  ## Prediction: 
  #    we use the function 'predict_state()' defined above as a substitute for prediction
  # x(k+1|k)
  state_p = predict_state(prev_pose, v, w)

  ## Covariance update: 
  #    we use the linearized state space matrix F to calculate the predicted covariance matrix
  # P(k+1|k)
  E_p = np.dot(np.dot(F,E),F.transpose()) + R

  ## Get measurement residual: 
  #    difference between y_measured and y_predicted of the (k+1)^th time instant
  # print(state_p.reshape(-1))
  Y_residual = Y_measured - predict_measurement(state_p[:2])


  ## Kalman gain calculation: 
  #    we use the linearized measurement matrix 'H(k+1|k)' to compute the kalman gain
  # W(k+1) 
  H = get_current_H(state_p.transpose().reshape(-1), lA, lB, lC)
  # print(H)
  W = np.dot(np.dot(E_p,H),
         np.linalg.inv(np.dot(np.dot(H,E_p), H.transpose()) + Q))
  
  ## Update state with kalman gain: 
  #    we correct the predicted state using the measurement residual 
  # x(k+1|k+1)
  state = state_p + np.dot(W, Y_residual)

  ## Update covariance matrix
  #    
  # P(k+1|k+1)
  E = np.dot((np.eye(3) - np.dot(W,H)), E_p)

  cov_1.append(E[0][0])
  cov_2.append(E[1][1])
  cov_3.append(E[2][2])

  file_1 = open('covariance_1.txt', 'a+')
  file_1.write(str(E[0][0]))
  file_1.close()

  file_2 = open('covariance_2.txt', 'a+')
  file_2.write(str(E[1][1]))
  file_2.close()

  file_3 = open('covariance_3.txt', 'a+')
  file_3.write(str(E[2][2]))
  file_3.close()

  pose = np.array(state.copy())
  pose = pose.reshape(-1)
  prev_pose = pose.copy()
  return


# global variables
index = 0
waypoints = []

def poseError(current_pose):
  global index, waypoints
  if index >= len(waypoints):
    rospy.signal_shutdown("Destination reached!")
    plt.plot(*range(len(cov_1)), cov_1)
    plt.plot(*range(len(cov_2)), cov_2)
    plt.plot(*range(len(cov_3)), cov_3)
    plt.legend(["E[0][0", "E[1][1]", "E[2][2]"])
    plt.show()
    return
  
  target_x = float(waypoints[index][0])
  target_y = float(waypoints[index][1])
  print("{:.2f}, {:.2f}, {:.2f}\tTarget:{:.2f}, {:.2f}".format(current_pose[0], current_pose[1], current_pose[2]*(180/pi), target_x, target_y))

  error_x = target_x - current_pose[0]
  error_y = target_y - current_pose[1]
  E_pos = np.linalg.norm([error_x, error_y])
  desired_theta = math.atan2(error_y, error_x)
  E_theta = desired_theta - current_pose[2]
  if (E_theta > math.pi):
    E_theta -= 2*math.pi
  elif (E_theta < -math.pi):
    E_theta += 2*math.pi

  if E_pos < 0.2:
    index += 1

  return E_pos, E_theta


def velocityControl(E_pos, E_theta):
  global v, w
  velocity_msg = Twist()
  max_angular_vel = 0.1
  max_linear_vel = 0.1
  K1 = 0.4
  K2 = 0.6
  if (np.abs(E_theta) > 0.1):
    velocity_msg.linear.x = 0.02*E_pos
    velocity_msg.angular.z = K1*E_theta
    # rospy.loginfo("Orienting")
  elif (E_pos > 0.1):
    velocity_msg.linear.x = K2*E_pos
    velocity_msg.angular.z = 0.01*E_theta
    # rospy.loginfo("Chasing")
  else:
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    rospy.loginfo("Your destination has arrived!")

  velocity_msg.linear.x = min(max_linear_vel, velocity_msg.linear.x)
  velocity_msg.angular.z = np.clip(velocity_msg.angular.z, -max_angular_vel, max_angular_vel)
  # Share with global variables
  v = velocity_msg.linear.x
  w = velocity_msg.linear.z

  return velocity_msg


## Trajectory tracking business
def control_loop():
  global v,w
  global pose, waypoints
  
  rospy.init_node('controller_node')
  pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
  rospy.Subscriber('/trilateration_data', Trilateration, callback)
  rospy.Subscriber('/odom', Odometry, callback2)
    
  ## Setting the rate for loop execution
  rate = rospy.Rate(5)     
  
  ## Read all waypoints
  csv_reader = csv.reader(open(os.path.join(sys.path[0], 'waypoints.txt')))
  for line in csv_reader:
    waypoints.append([line[0],line[1]])
  
  rospy.sleep(2.)
  ## Twist values to move the robot
  while not rospy.is_shutdown():
    #### If robot has reached the current waypoint read next waypoint
    E_pos, E_theta = poseError(pose)
    #### Apply proportional control to reach the waypoint
    velocity_msg = velocityControl(E_pos, E_theta)
    pub.publish(velocity_msg)
    
    rate.sleep()


if __name__ == '__main__':
  try:
    control_loop()
  except rospy.ROSInterruptException:
    pass
