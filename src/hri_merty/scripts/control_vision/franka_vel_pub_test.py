#!/usr/bin/env python3
# license removed for brevity
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, Float64, Bool

# Declaring cvBridge for cv to ros conversion and vice versa
control_flag = None


def start_vel(msg):
    global control_flag
    print("start vel getting called")
    control_flag = msg.data

def main():
    # Initialize the node
    rospy.init_node('pos_control_dream_depth_ds_gen')
    flag_sub = rospy.Subscriber("/franka/control_flag", Bool, start_vel, queue_size=1)

    pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
    stop_pub = rospy.Publisher('/node/stop', Float64, queue_size=1)

    print("velocity control getting called")

    joint_vel_00 = Float64MultiArray()
    joint_vel_01 = Float64MultiArray()
    joint_vel_02 = Float64MultiArray()
    joint_vel_03 = Float64MultiArray()
    joint_vel_04 = Float64MultiArray()
    joint_vel_05 = Float64MultiArray()
    joint_vel_06 = Float64MultiArray()
    joint_vel_07 = Float64MultiArray()
    joint_vel_08 = Float64MultiArray()

    status = Float64()
    status.data = 0.0

    joint_vel_01.data = [0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0] 
    # joint_vel_02.data = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_vel_02.data = [0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0]
    joint_vel_03.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.002, 0.0]
    # joint_vel_04.data = [0.0, -0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    # joint_vel_04.data = [-0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # # joint_vel_05.data = [0.0, 0.05, 0.0, 0.05, 0.0, 0.0, 0.0]
    # joint_vel_05.data = [0.05, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    # # joint_vel_06.data = [0.0, -0.05, 0.0, -0.025, 0.0, 0.0, 0.0] 
    # joint_vel_06.data = [-0.05, 0.0, 0.0, -0.025, 0.0, 0.0, 0.0] 
    # # joint_vel_07.data = [0.0, 0.05, 0.0, -0.05, 0.0, 0.0, 0.0]
    # joint_vel_07.data = [0.05, 0.0, 0.0, -0.05, 0.0, 0.0, 0.0]
    # # joint_vel_08.data = [0.0, -0.05, 0.0, 0.025, 0.0, 0.0, 0.0]
    # joint_vel_08.data = [-0.05, 0.05, 0.0, 0.025, 0.0, 0.0, 0.0]

    joint_vel_00.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    # put the msgs in matrix

    #rostopic pub /vel/status std_msgs/Float64 "data: 0.0"
    # while not rospy.is_shutdown():
    count = 1
    dur = 1.5

    # Latest changes

    while not rospy.is_shutdown():
        print(control_flag)
        if control_flag == True:
            start_time = rospy.Time.now()
            to_wait = rospy.Duration(count*dur)
            end_time_1 = start_time + to_wait    
            while rospy.Time.now() < end_time_1:
                # print("We need to check the first one")
                pub.publish(joint_vel_01)
            count+=1
            start_time = rospy.Time.now()
            to_wait = rospy.Duration(count*dur)
            end_time_2 = start_time + to_wait    
            while end_time_1 < rospy.Time.now() < end_time_2:
                pub.publish(joint_vel_02)
            count+=1
            start_time = rospy.Time.now()
            to_wait = rospy.Duration(count*dur)
            end_time_3 = start_time + to_wait    
            while end_time_2 < rospy.Time.now() < end_time_3:
                pub.publish(joint_vel_03)
            # count+=1
            # start_time = rospy.Time.now()
            # to_wait = rospy.Duration(count*dur)
            # end_time_4 = start_time + to_wait    
            # while end_time_3 < rospy.Time.now() < end_time_4:
            #     pub.publish(joint_vel_04)
            # count+=1
            # start_time = rospy.Time.now()
            # to_wait = rospy.Duration(count*dur)
            # end_time_5 = start_time + to_wait    
            # while end_time_4 < rospy.Time.now() < end_time_5:
            #     pub.publish(joint_vel_05)
            # count+=1
            # start_time = rospy.Time.now()
            # to_wait = rospy.Duration(count*dur)
            # end_time_6 = start_time + to_wait    
            # while end_time_5 < rospy.Time.now() < end_time_6:
            #     pub.publish(joint_vel_06)
            # count+=1
            # start_time = rospy.Time.now()
            # to_wait = rospy.Duration(count*dur)
            # end_time_7 = start_time + to_wait    
            # while end_time_6 < rospy.Time.now() < end_time_7:
            #     pub.publish(joint_vel_07)
            # count+=1
            # start_time = rospy.Time.now()
            # to_wait = rospy.Duration(count*dur)
            # end_time_8 = start_time + to_wait    
            # while end_time_7 < rospy.Time.now() < end_time_8:
            #     pub.publish(joint_vel_08)
            # start_time = rospy.Time.now()
            # to_wait = rospy.Duration(count*dur)
            # end_time_8 = start_time + to_wait    
            while end_time_3 < rospy.Time.now():
                pub.publish(joint_vel_00)
                status = 0.0
                stop_pub.publish(status)
            print("we have checked enough")
            

    rospy.spin()


if __name__ == '__main__':
    main()