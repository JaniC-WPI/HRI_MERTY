#!/usr/bin/env python3
# license removed for brevity
import math
import random
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from numpy.linalg import multi_dot
from geometry_msgs.msg import Twist
#from sympy import *

#Jpinv = []
def callback_vk(dt):
    global Jpinv, q1, q2
    q1 = dt.position[0]
    q2 = dt.position[1]
    
    #dt = Twist()
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    c12 = np.cos(q1+q2)
    s12 = np.sin(q1+q2)
        
    Jp = np.array([[-(1.1*s12) - (1.1*s1), -(1.1*s12)],[(1.1*c12) + (1.1*c1),  (1.1*c12)]]) #Position Jacobian derived from matlab
    
    
    Jpinv = np.linalg.inv(Jp)       
        
        #xdot = dt.linear.x [linear velocity in case we need to subscribe from Twist]]
        #ydot = dt.linear.y [same as above]
        
    '''xdot = np.float64(input("Enter x_velocity: "))
    ydot = np.float64(input("Enter y_velocity: "))
        #zdot = dt.linear.z
    
    lin_vel = np.array([[xdot],[ydot]])

    Theta_dot = np.matmul(Jpinv, lin_vel)    
        
    q1_dot = Theta_dot[0]
    q2_dot = Theta_dot[1]
        
    #lin_vel_test = np.dot(Jp, Theta_dot)        
       
                
    #print(q1, q2)
    print(Jpinv)       
    print(Theta_dot)
     
    pub1.publish(q1_dot)
    pub2.publish(q2_dot)'''
    #pub3.publish(T0ee)
        
if __name__ == "__main__":
    rospy.init_node('end_effector', anonymous=True)
    pub1 = rospy.Publisher('/rrbot/joint1_velocity_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/rrbot/joint2_velocity_controller/command', Float64, queue_size=1)
    sub_vel = rospy.Subscriber('/rrbot/joint_states', JointState, callback_vk)
    
    xdot = np.float64(input("Enter x_velocity: "))
    ydot = np.float64(input("Enter y_velocity: "))
        #zdot = dt.linear.z
    
    lin_vel = np.array([[xdot],[ydot]])

    Theta_dot = np.matmul(Jpinv, lin_vel)    
        
    q1_dot = Theta_dot[0]
    q2_dot = Theta_dot[1]
        
    #lin_vel_test = np.dot(Jp, Theta_dot)        
       
                
    print(q1, q2)
    print(Jpinv)       
    print(Theta_dot)
     
    pub1.publish(q1_dot)
    pub2.publish(q2_dot)
    
    r= rospy.Rate(1)
     
while not rospy.is_shutdown():
	r.sleep()