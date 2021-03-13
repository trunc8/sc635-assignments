#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from week5.msg import Pose
from math import sin, cos

## Constant
V_TIMEOUT = 0.6

## Globals
v_last = 0.0
w_last = 0.0

time_last = 0.0
time_last_command = 0.0

robot_pose = [0.0, 0.0, 0.0]
pub = None
twist_msg = Twist()

## Condition check flag
initialization_flag = False

## Iteration 
v_actual = 0.0
w_actual = 0.0


## Dead reckoning
def update_pose():
    '''
    This function is called at two places
     1) Periodically at 20 Hz
     2) Whenever new control (v,w) is issued
    '''
    global pub, twist_msg, robot_pose, v_last, w_last, time_last, V_TIMEOUT, time_last_command
    
    # how much time has elapsed ?
    time_now = rospy.Time.now()

    time_diff = time_now - time_last
    time_diff2 = time_now - time_last_command


    v = v_last
    w = w_last


    if time_diff2 > V_TIMEOUT:
        ## Time elapsed since lase command was issued is greater than timeout
        time_diff = time_diff - (time_diff2 - V_TIMEOUT)
        v_last = 0
        w_last = 0
    
    # Euler integration
    dX = v * cos(robot_pose[2]) * time_diff.to_sec()
    dY = v * sin(robot_pose[2]) * time_diff.to_sec()
    dT = w * time_diff.to_sec()
    
    robot_pose[0] += dX
    robot_pose[1] += dY
    robot_pose[2] += dT

    print("Debug :: time_diff:{}, pose : {}".format(time_diff, robot_pose))
    
    ## Reset the time base    # critical
    time_last = time_now
        
    

## Receive Twist() message and pass it on to the robot
def callback(odom_msg):
    global pub, v_last, w_last, time_last_command

    ## we received a new velocity command.
    ## Let us update the pose from last command
    update_pose()

    ## pass on the received (v,w) messages
    pub.publish(odom_msg)

    ## register the time
    time_last_command = rospy.Time.now()

    ## Record the commands being issued
    v_last = odom_msg.linear.x
    w_last = odom_msg.angular.z



def control_loop():
    global robot_pose, pub, V_TIMEOUT, time_last, time_last_command
    rospy.init_node('manual_odom_compute')

    ## accept v,w at this topic
    rospy.Subscriber('odometry_tracker', Twist, callback)

    ## relay the v,w to robot 
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    ## manually computed odometry will be available at this topic
    pub2 = rospy.Publisher('/manual_odom', Pose, queue_size=10)

    V_TIMEOUT = rospy.Duration(V_TIMEOUT)


    time_last = rospy.Time.now()
    time_last_command = rospy.Time.now()
    
    tick = 0 
    rate = rospy.Rate(15) ## Higher rate increases accuracy of integration approximation
    while not rospy.is_shutdown():
        update_pose()     
        tick += 1
        if (tick % 3 == 0): ## publish at 1/4 th frequency of Rate()
            current_pose = Pose(robot_pose[0], robot_pose[1], robot_pose[2])
            pub2.publish(current_pose)
        rate.sleep()



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
