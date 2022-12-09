#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import String
from franka_gripper.msg import GraspActionGoal, StopActionGoal

grasp_data = None

grasp_stop = rospy.Publisher('/franka_gripper/stop/goal', StopActionGoal, queue_size=1) 

n = 1

def pick_place_callback(msg):
    
    user_input = msg.data

    grasp_stop_data = StopActionGoal()

    if user_input == 'pick':
      grasp_data.goal_id.id = str(n)
      grasp_data.goal.width = 0.0
      grasp_data.goal.force = 0.7
      grasp_data.goal.speed = 2.0
      grasp_data.goal.epsilon.inner = 0.25
      grasp_data.goal.epsilon.outer = 0.25
      print("pick is chosen")
      
    elif user_input == 'open' :
        grasp_data.goal.width = 1.0
        grasp_data.goal.force = -1
        grasp_data.goal.speed = 2.0
        grasp_data.goal.epsilon.inner = 0.5
        grasp_data.goal.epsilon.outer = 0.5
        print("gripper opened")

    elif user_input == 'place' :
        grasp_data.goal_id.id = str(n+1)
        grasp_data.goal.width = 0.5
        grasp_data.goal.force = -1
        grasp_data.goal.speed = 2.0
        grasp_data.goal.epsilon.inner = 0.5
        grasp_data.goal.epsilon.outer = 0.5
        print("object placed")
        
      # grasp_data.goal.force = -1.0
      # grasp_data.goal.speed = 2.0
      # grasp_data.goal.epsilon.inner = 0.5
      # grasp_data.goal.epsilon.outer = 0.5

    else:
      grasp_stop_data.goal_id.id = '1'
      grasp_stop.publish(grasp_stop_data)
     


def main():
    global grasp_data, grasp_stop_data
    # initialize the node
    rospy.init_node('franka_grasp_node')

    # Initialize user input subscriber for pick and place
    pick_place_sub = rospy.Subscriber("vsbot/pick_place", String, pick_place_callback, queue_size=1)

    # Publisher for grasping
    grasp_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1) 
    # grasp_stop = rospy.Publisher('/franka_gripper/stop/goal', StopActionGoal, queue_size=1) 
    
    grasp_data = GraspActionGoal()    
    # grasp_stop_data = StopActionGoal()
    
    # rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      # Publish goal image at the loop rate
      grasp_pub.publish(grasp_data)
      # rate.sleep() 

if __name__ == '__main__':
    main()
