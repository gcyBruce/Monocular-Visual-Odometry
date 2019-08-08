#!/usr/bin/python2
"""
Created on Mon Oct  1 14:24:26 2018

@author: Shelton
"""

import rospy, time
from geometry_msgs.msg import Twist
from create_node.msg import TurtlebotSensorState
from cylinder.msg import cylDataArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import sys, select, termios, tty
import os
from std_msgs.msg import Float64
import math
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class POSE(object):
    
    def __init__(self,robot_abs=np.array([0,0,0]),cylin_abs=np.array([0,0,0])):
        self.sub_odom=rospy.Subscriber('/odom',Odometry,self.odom_callback)
        self.sub_cyl=rospy.Subscriber("/cylinderTopic", cylDataArray, self.cyl_callback)
        self.sub_cmd=rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist,self.cmd_callback)
        self.robot_abs=robot_abs
        self.cylin_abs=cylin_abs
        self.cyl_dict={}
        self.cyl_pose={}
        self.pose_hist=[]
        self.cyl_hist=[]
        self.cur_time=0
        self.pre_time=0
        self.robot_pose=[]

    def cyl_callback(self,msg):
     	if (len(msg.cylinders)>0):
             for i,data in enumerate(msg.cylinders):
                 conv=data.covariance
		 
                 self.cyl_dict[data.label]=[data.Zrobot,data.Xrobot,[conv[0]]+list(conv[2:4])]
                 #print(self.cyl_dict[data.label])
                 self.cyl_pose[data.label]=cylin_pose_predict(self.pose,self.cyl_dict[data.label])
                 
             self.cyl_hist.append(self.cyl_dict)
             
             f=open('/home/turtlebot/catkin_ws/src/group6/src/measurement3.txt','a')
             f.write(str(['cyl_mea',self.cyl_dict])+'\n')
             f.close()
             
             f=open('/home/turtlebot/catkin_ws/src/group6/src/pose3.txt','a')
             f.write(str(['cyl_pose',self.cyl_pose])+'\n')
             f.close()
             
             

     		#print Zrobot, Xrobot, label
     	else:
     		print 'no cylinder'

    def cmd_callback(self,msg):
        self.cur_time=time.time()
        vx=msg.linear.x
        theta=msg.angular.z
        dur=self.cur_time-self.pre_time
        self.u=np.array([vx,0,theta])*dur
        self.pre_time=self.cur_time

              
    def odom_callback(self,msg):
        pose_id=msg.header.seq
        x = msg.pose.pose.position.x  # get linear velocity x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        theta = yaw
        conv=msg.pose.covariance
        sig_xx=conv[0]
        sig_xy=conv[1]
        sig_xtheta=conv[5]
        sig_yy=conv[7]
        sig_ytheta=conv[11]
        sig_thetatheta=conv[-1]
        convariance=[sig_xx,sig_xy,sig_xtheta,sig_yy,sig_ytheta,sig_thetatheta]
        
        self.pose=[x,y,theta,convariance]
        self.pose_hist.append(self.pose)
        
        f=open('/home/turtlebot/catkin_ws/src/group6/src/measurement3.txt','a')
        f.write(str(['odom',self.pose])+'\n')
        f.close()
        
        self.robot_pose=robot_pose_predict(self.pose,self.u)
        f=open('/home/turtlebot/catkin_ws/src/group6/src/pose3.txt','a')
        f.write(str(['robot_pose',self.robot_pose])+'\n')
        f.close()
    
def robot_pose_predict(robot_abs,u):
    x1 = robot_abs[0]
    y1 = robot_abs[1]
    theta1 = robot_abs[2]
    dx = u[0]
    dy = u[1]
    dtheta = u[2]
    
    #R is the transition matrix of robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -dx*np.sin(theta1)-dy*np.cos(theta1)],
          [0, 1,  dx*np.cos(theta1)-dy*np.sin(theta1)],
          [0, 0, 1]]
         
    #Calculate Jacobian H2 with respect to u
    H2 = [[np.cos(theta1), -np.sin(theta1), 0],
          [np.sin(theta1), np.cos(theta1), 0],
          [0, 0, 1]]
    
    #print(robot_abs)
    print( np.dot(R,u))
    next_robot_abs = np.dot(R,u) + np.array(robot_abs[:3])
    
    return [next_robot_abs, H1, H2]
    
def cylin_pose_predict(robot_abs,landmark_meas_xy):

    '''
    Calcute Landmark's absolute coordinate
    Input: robot's absolute coordinate [x,y,theta]
           landmark's measurment with repect to robot frame [x,y]
    Output: landmarks's absolute coordinate  [x,y]
    '''
    
    x1 = robot_abs[0]
    y1 = robot_abs[1]
    theta1 = robot_abs[2]
    x2 = landmark_meas_xy[0]    
    y2 = landmark_meas_xy[1]
    
    landmark_meas = [[x2],
                     [y2],
                     [1]]
    
    #R is the transition matrix to robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -x2*np.sin(theta1)-y2*np.cos(theta1)],
          [0, 1,  x2*np.cos(theta1)-y2*np.sin(theta1)]]
         
    #Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), -np.sin(theta1)],
          [np.sin(theta1),  np.cos(theta1)]]
     
    landmark_abs = np.dot(R,landmark_meas) + np.array(robot_abs[:3]) 
    
    return [[landmark_abs[0][0],landmark_abs[1][0]], H1, H2]
       
if __name__ == "__main__":
    rospy.init_node('lab4_task11')
    pose=POSE()
    
    
    #rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist,cml_callback)

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

        
        
        
        
    
    
    
    
    
             
