#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy as np

from week5.msg import Trilateration, Landmark



## quaternion to euler
def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)
########################


## Globals
pose = [0.0, 0.0, 0.0]

landmarkA = [ -5,  7]
varA = 0.020
landmarkB = [  5,  7]
varB = 0.015
landmarkC = [  0, -7] 
varC = 0.010
########################


## Odometry callback
def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
########################


## Euclidean Distance
def dist(p1, p2):
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)
########################


## Node 
def trilateration_pub():
    global landmarkA, landmarkB, landmarkC, varA, varB, varC
    
    rospy.init_node('Trilateration_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback)
    
    pub = rospy.Publisher('trilateration_data', Trilateration, queue_size=10)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        lA = Landmark(landmarkA[0], landmarkA[1], dist(pose, landmarkA)+np.random.normal(0,varA), varA)
        lB = Landmark(landmarkB[0], landmarkB[1], dist(pose, landmarkB)+np.random.normal(0,varB), varB)
        lC = Landmark(landmarkC[0], landmarkC[1], dist(pose, landmarkC)+np.random.normal(0,varC), varC)
        t = Trilateration(lA, lB, lC)
        rospy.loginfo("Sent :\n{}".format(t))
        pub.publish(t)
        rate.sleep()
########################

if __name__ == '__main__':
    try:
        trilateration_pub()
    except rospy.ROSInterruptException:
        pass
