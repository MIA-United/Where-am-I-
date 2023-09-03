#!/usr/bin/env python
import numpy as np
import rospy
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

## These are the constrains of our robot
## It is useful to store them in variables instead
## of directly in the matrix so we can fine tune them

dis_cen = 0.275         # Distance from the robot's center to the wheel
wheel_radius = 0.05     # the radius of the wheel


## This is the robot's inverse kinematics matrix
## To learn more about it and how it was derived
## Check out this Paper: https://shorturl.at/lS578

Inverse_Kinematic_Matrix = (wheel_radius * (np.sqrt(2)/4)) * np.array([
            [  1,  -1,  -1,   1],
            [  1,   1,  -1,  -1],
            [1/(np.sqrt(2)*dis_cen), 1/(np.sqrt(2)*dis_cen), 1/(np.sqrt(2)*dis_cen), 1/(np.sqrt(2)*dis_cen)]
        ])

## This is a rotation matrix, it is used to make sure the 
## X and Y coordinates are not affected by the robots rotation

rot_matrix = np.array([[np.cos(heading),-np.sin(heading),0],
                      [np.sin(heading),np.cos(heading),0],
                      [0,0,1]])

last_time=time.time()

odom_quat=[0.0,0.0,0.0,0.0]

x_pos=0.0
y_pos=0.0

x_pos_abs=0
y_pos_abs=0


#############################################################################################
########################## PART ONE, CALCULATE THE CHANGE IN HEADING#########################

heading=0.0
last_heading=0
delta_heading=0

def IMU_Callback_Function(msg):
    global heading, last_heading,delta_heading
    ### YOUR CODE GOES HERE ###
    


    ############################


#############################################################################################
########################## PART TWO, GET THE ROBOT'S VELOCITIES #############################

robot_velocities = [0,0,0,0,0,0]
robot_velocities_package = [0,0,0,0,0,0]
enc_velocities = [0,0,0,0,0,0]

def Encoders_Callback_Function(msg):
    global robot_velocities, enc_velocities,robot_velocities_package
    ### YOUR CODE GOES HERE ###
    


    ############################

#############################################################################################

if __name__ == '__main__':
    rospy.init_node('odom_node')

    Encoder_Speed_Subscriber  = rospy.Subscriber('feedback_s', Float32MultiArray, Encoders_Callback_Function)
    sub_angle_imu   = rospy.Subscriber('IMU_DATA_deg', Float32, IMU_Callback_Function)
    
    Odomotery_Publisher  = rospy.Publisher('odomomotery', Odometry,queue_size=100)
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id  = "base_link"
    last_send=0
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.get_rostime()
        now=time.time()
        ### YOUR CODE GOES HERE ###
        
        ### Calculate the x_pos and the y_pos
    


        ############################
        
        cy = np.cos(heading  * 0.5)
        sy = np.sin(heading  * 0.5)
        cp = 1.0
        sp = 0
        cr = 1.0
        sr = 0

        odom.pose.pose.position.x = x_pos
        odom.pose.pose.position.y = y_pos
        odom.pose.pose.position.z =0.0

        odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        odom.twist.twist.linear.y  =  robot_velocities[1]
        odom.twist.twist.linear.x  =  robot_velocities[0] 

        odom.twist.twist.linear.z   = 0
        odom.twist.twist.angular.x  = 0
        odom.twist.twist.angular.y  = 0

        odom.twist.twist.angular.z   = robot_velocities[2]

        pub.publish(odom)
            
