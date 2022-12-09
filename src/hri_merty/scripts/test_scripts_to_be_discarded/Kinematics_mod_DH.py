"""
Worcester Polytechnic Institute, MA, USA
Code developed by : Abhay Karade
Email             : akarade@wpi.edu
"""

import math
import numpy as np


class Kinematics():
    """
    Input: 7 Joint angles 

    """

    def __init__(self, joint_angles):
        self.joint_angles = joint_angles[1:7]

        # modified DH parameters
        #                       a(m) d(m)           alpha(rad)      theta(rad)
        DH_params = {"joint1": [0, 0.333,             0,   joint_angles[0]],
                     "joint2": [0,     0,  -(math.pi/2),   joint_angles[1]],
                     "joint3": [0, 0.316,   (math.pi/2),   joint_angles[2]],
                     "joint4": [0.0825,     0,   (math.pi/2),   joint_angles[3]],
                     "joint5": [-0.0825, 0.384,  -(math.pi/2),   joint_angles[4]],
                     "joint6": [0,     0,   (math.pi/2),   joint_angles[5]],
                     "joint7": [0.088,     0,   (math.pi/2),   joint_angles[6]] }

        self.DH_params = DH_params

    def forward(self):
        joints = ["joint1", "joint2", "joint3",
                  "joint4", "joint5", "joint6", "joint7"]

        T_old = np.eye(4)

        """
        T_cumulative
        ex: 5 th matrix in T_cumulative is transformation from link5 to link 0   

        T_individual 
        ex: 5 th matrix in T_individual  is transformation from link5 to link 4 
        """

        T_cumulative = []
        T_individual = []

        for joint in joints:
            T_new = self.t_matrix(joint)
            T_individual.append(T_new)
            T = T_old @ T_new
            T_cumulative.append(T)
            T_old = T

        return T_cumulative

    def t_matrix(self, joint):
        a, d, alpha, theta = self.DH_params[joint]
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        """
        for classical DH parameters
        """

        # T = np.array([[ct,   -st*ca,  st*sa,  a*ct],
        #               [st,    ct*ca, -ct*sa,  a*st],
        #               [0,       sa,     ca,     d],
        #               [0,        0,      0,     1]])

        '''
        For modified DH parameters
        to calculate transformation matrix from DH parameters 
        '''

        T = np.array([[ct,       -st,   0,      a],
                      [st*ca,  ct*ca, -sa,  -d*sa],
                      [st*sa,  ct*sa,  ca,   d*ca],
                      [0,          0,   0,      1]])
        
        return T


# if __name__ == '__main__':
#     # random joint values from Gazebo
#     joint_angles = [0.03999850283282958, 0.03906658922185621, -0.00044750571679763596, -
#                     0.035452213051629045, -0.0007943739502156433, -0.5683594221923984, 0.0017109621550908116]

    # forward = Kinematics(joint_angles)
    # forward.forward()

    # print(np.around(forward.forward()[1], decimals=3))
