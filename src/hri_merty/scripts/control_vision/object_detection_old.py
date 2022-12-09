#!/usr/bin/env python3

import rospy
import signal
import sys
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int64, Float64MultiArray, String
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

kernel = np.ones((5 ,5), np.uint8)
img = None
ros_img = None
color_flag = None
red_center = []
blue_center = []
green_center = []
red_mask = None
blue_mask = None
green_mask = None
marker_center = None
error_x = 0
error_y = 0
error_vect = []

i = 0

def image_callback(img_msg):
    global bridge, i,img, ros_img, kernel, red_mask, blue_mask, green_mask
    print("is image callbacl getting called")
    ros_img = img_msg
    #convert ros_image into an opencv-compatible image
    if ros_img is not None:
      img = bridge.imgmsg_to_cv2(ros_img, "bgr8")

      hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      red_lower = np.array([0, 50, 50], np.uint8) #[17, 15, 100]
      red_upper = np.array([10, 255, 255], np.uint8) #[180, 255, 255][50, 56, 200],
      red_mask = cv2.inRange(hsvimg, red_lower, red_upper) 

      green_lower = np.array([25, 52, 72], np.uint8) 
      green_upper = np.array([102, 255, 255], np.uint8) 
      green_mask = cv2.inRange(hsvimg, green_lower, green_upper)

      blue_lower = np.array([94, 80, 2], np.uint8) 
      blue_upper = np.array([120, 255, 255], np.uint8) 
      blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper) 

#     For red color 
      red_mask = cv2.dilate(red_mask, kernel) 
      res_red = cv2.bitwise_and(img, img,  
                                  mask = red_mask) 

#     For green color 
      green_mask = cv2.dilate(green_mask, kernel) 
      res_green = cv2.bitwise_and(img, img, 
                                   mask = green_mask) 

#     For blue color 
      blue_mask = cv2.dilate(blue_mask, kernel) 
      res_blue = cv2.bitwise_and(img, img, 
                                   mask = blue_mask) 


def id_red(red_mask):
    global img, i, red_center
    opening = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)    
    x, y, w, h = cv2.boundingRect(opening)
    red_center = [x, y]
    cv2.rectangle(img, (x, y), (x+w, y + h), (0, 255, 0), 3)
    cv2.circle(img, (np.int64(x+w/2), np.int64(y+h/2)), 5, (0, 0, 255), -1) 
    cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/red/image_capture_' + str(i) + '.jpg', img)
    i = i+1

    return red_center

def id_blue(blue_mask):
    global img, i, blue_center, img
    print("is it even getting caled")
    opening = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)    
    x, y, w, h = cv2.boundingRect(opening)
    blue_center = [x, y]
    cv2.rectangle(img, (x, y), (x+w, y + h), (0, 255, 0), 3)
    cv2.circle(img, (np.int64(x+w/2), np.int64(y+h/2)), 5, (0, 0, 255), -1) 
    cv2.circle(img, (640, 360), 5, (255, 0, 0), -1)
    print(error_x, error_y)
    cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/blue/image_capture_' + str(i) + '.jpg', img)
    i = i+1

    return blue_center

def id_green(green_mask):          
    global img, i, green_center
    opening = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    x, y, w, h = cv2.boundingRect(opening)
    green_center = [x,y]
    cv2.rectangle(img, (x, y), (x+w, y + h), (0, 255, 0), 3)
    cv2.circle(img, (np.int64(x+w/2), np.int64(y+h/2)), 5, (0, 0, 255), -1) 
    cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/green/image_capture_' + str(i) + '.jpg', img)
    i = i+1

    return green_center


def flag_cb(msg):
    global color_flag, error_vect, error_x, error_y
    print("flag call back getting called")
    color_flag = msg.data
    print(color_flag)
    if color_flag == 'blue' and ros_img is not None:      
        target = id_blue(blue_mask)    
        # print(target)
    elif color_flag == 'red' and ros_img is not None:      
        target = id_red(red_mask)        
    elif color_flag == 'green' and ros_img is not None:       
        target = id_green(green_mask)     
    else:
        target = [640, 360]

    # print(target)
    error_x = target[0] - 640    
    error_y = target[1] - 360

    # print(error_x, error_y)
    error_vect = Float64MultiArray()  
    error_vect.data = [error_x, error_y]    
    # print(error_vect.data)

def main():
  rospy.init_node('image_converter', anonymous=True)

  image_sub = rospy.Subscriber("/camera/color/image_raw",Image, image_callback)
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
