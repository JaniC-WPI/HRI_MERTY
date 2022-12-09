#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray, Int64, String
import sys

bridge = CvBridge()
ros_img = None
cv_img = None
red_center = []
blue_center = [] 
green_center = []
yellow_center = []
red_index = None
blue_index = None
green_index = None
yellow_index = None
red_corners_list = []
blue_corners_list = []
green_corners_list = []
yellow_corners_list = []
marker_flag = None
color_flag = None
i = 0
# Subscriber callback
def marker_pose(img_msg):
    global bridge, ros_img, red_center, blue_center, green_center, yellow_center, red_index, blue_index, green_index, yellow_index, marker_flag, red_corners_list, \
            blue_corners_list, green_corners_list, yellow_corners_list, cv_img
    ros_img = img_msg
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY) 
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) 
    arucoParameters = aruco.DetectorParameters_create() 
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)

    print(ids)

    # # print("Id generated", ids) #Remove
    id_list = []
    id_list.clear()
    for i in ids:
        if int(i) == 501:
            id_list.append(int(i))
        elif int(i) == 620:
            id_list.append(int(i))
        elif int(i) == 411:                    
            id_list.append(int(i))
        elif int(i) == 504:
            id_list.append(int(i))
    # Aruco ID for each marker
    red_id = 501
    blue_id = 620
    green_id = 411
    yellow_id = 504

    print(id_list)
    
   
    # check if one of the colors found and get the center for the same

    if red_id in id_list:
        red_index = id_list.index(red_id)
        red_corners_list = corners[red_index].reshape(4,2)
        # Averaging corner co-ordinates to obtain marker center
        red_center_x = (red_corners_list[0][0] + red_corners_list[1][0] + red_corners_list[2][0] + red_corners_list[3][0])/4
        red_center_y = (red_corners_list[0][1] + red_corners_list[1][1] + red_corners_list[2][1] + red_corners_list[3][1])/4
        red_center = [red_center_x, red_center_y]
        print("red id", red_id)

    if blue_id in id_list:
        blue_index = id_list.index(blue_id)
        blue_corners_list = corners[blue_index].reshape(4,2)
        # Averaging corner co-ordinates to obtain marker center
        blue_center_x = (blue_corners_list[0][0] + blue_corners_list[1][0] + blue_corners_list[2][0] + blue_corners_list[3][0])/4
        blue_center_y = (blue_corners_list[0][1] + blue_corners_list[1][1] + blue_corners_list[2][1] + blue_corners_list[3][1])/4
        blue_center = [blue_center_x, blue_center_y]

    if green_id in id_list:
        green_index = id_list.index(green_id)
        green_corners_list = corners[green_index].reshape(4,2)
        # Averaging corner co-ordinates to obtain marker center
        green_center_x = (green_corners_list[0][0] + green_corners_list[1][0] + green_corners_list[2][0] + green_corners_list[3][0])/4
        green_center_y = (green_corners_list[0][1] + green_corners_list[1][1] + green_corners_list[2][1] + green_corners_list[3][1])/4
        green_center = [green_center_x, green_center_y]

    if yellow_id in id_list:
        yellow_index = id_list.index(yellow_id)
        yellow_corners_list = corners[yellow_index].reshape(4,2)
        # Averaging corner co-ordinates to obtain marker center
        yellow_center_x = (yellow_corners_list[0][0] + yellow_corners_list[1][0] + yellow_corners_list[2][0] + yellow_corners_list[3][0])/4
        yellow_center_y = (yellow_corners_list[0][1] + yellow_corners_list[1][1] + yellow_corners_list[2][1] + yellow_corners_list[3][1])/4
        yellow_center = [yellow_center_x, yellow_center_y]       
         

def flag_cb(msg):
    global color_flag, error_vect, error_x, error_y, i
    print("flag call back getting called")
    color_flag = msg.data
    print(color_flag)
    if (color_flag == 'blue') and (ros_img is not None):      
        target = blue_center 
        cv2.circle(cv_img, (np.int64(target[0]), np.int64(target[1])), 5, (0, 0, 255), -1) 
        cv2.circle(cv_img, (640, 360), 5, (0, 0, 255), -1) 
        cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/blue/image_capture_' + str(i) + '.jpg', cv_img)
        # print(target)
    elif (color_flag == 'red') and (ros_img is not None):      
        print("red center", red_center)
        target = red_center   
        x = target[0] - 640
        y = target[1] - 480
        cv2.circle(cv_img, (np.int64(target[0]), np.int64(target[1])), 5, (0, 0, 255), -1) 
        cv2.circle(cv_img, (640, 360), 5, (0, 0, 255), -1) 
        cv2.putText(cv_img, str(x), (200, 50),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.9, (0, 255, 0), 2)
        cv2.putText(cv_img, str(y), (400, 100),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.9, (0, 255, 0), 2)
        cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/red/image_capture_' + str(i) + '.jpg', cv_img)
        # print(target)
    elif (color_flag == 'green') and (ros_img is not None):       
        target = green_center   
        cv2.circle(cv_img, (np.int64(target[0]), np.int64(target[1])), 5, (0, 0, 255), -1) 
        cv2.circle(cv_img, (640, 360), 5, (0, 0, 255), -1) 
        cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/green/image_capture_' + str(i) + '.jpg', cv_img)
        # print(target)
    elif (color_flag == 'yellow') and (ros_img is not None):
        target = yellow_center
        cv2.circle(cv_img, (np.int64(target[0]), np.int64(target[1])), 5, (0, 0, 255), -1) 
        cv2.circle(cv_img, (640, 360), 5, (0, 0, 255), -1) 
        cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/yellow/image_capture_' + str(i) + '.jpg', cv_img)
        # print(target)
    else:
        target = [640, 360]


    print(target)
    error_x = target[0] - 640    
    error_y = target[1] - 360

    # print(error_x, error_y)
    error_vect = Float64MultiArray()  
    error_vect.data = [error_x, error_y]    
    print(error_vect.data)

    i = i+1

def main():
  rospy.init_node('image_converter', anonymous=True)

  image_sub = rospy.Subscriber("/camera/color/image_raw",Image, marker_pose)
  flag_sub = rospy.Subscriber("/color/flag", String, flag_cb)
  voice_fb = rospy.Publisher("/voice/feedback", Int64, queue_size=1)
  vel_pub = rospy.Publisher("/velocity/vect", Float64MultiArray, queue_size=1)

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    if color_flag is not None:
      print(color_flag)
      vel_pub.publish(error_vect)

    rate.sleep()

  rospy.spin()

  
if __name__ == '__main__':
    main()
