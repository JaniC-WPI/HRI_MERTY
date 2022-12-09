#!/usr/bin/env python3

import rospy
import signal
import sys
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int64, Float64MultiArray
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

#pub1 = rospy.Publisher("publishXGreen", Int64, queue_size=10)
#pub2 = rospy.Publisher("publishYGreen", Int64, queue_size=10)

pub1 = rospy.Publisher("publishError", Float64MultiArray, queue_size=0.1)
kernel = np.ones((5 ,5), np.uint8)


i = 0

def image_callback(ros_image):
  print('got an image')
  global bridge, i,img
  global gnCx, gnCy, rCx, rCy, blCx, blCy, grCx, grCy, green_center, img_grn, grn_area
  #convert ros_image into an opencv-compatible image
  try:
    img = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)      
  #cv_image = cv2.imread(cv_image) 
  
  #from now on, you can work exactly like with opencv
  '''(rows,cols,channels) = cv_image.shape
  if cols > 200 and rows > 200 :
      cv2.circle(cv_image, (100,100),90, 255)
  font = cv2.FONT_HERSHEY_SIMPLEX
  cv2.putText(cv_image,'Webcam Activated with ROS & OpenCV!',(10,350), font, 1,(255,255,255),2,cv2.LINE_AA)'''
  
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

    
  kernel = np.ones((5, 5), "uint8")

# For red color 
  red_mask = cv2.dilate(red_mask, kernel) 
  res_red = cv2.bitwise_and(img, img,  
                              mask = red_mask) 
      
# For green color 
  green_mask = cv2.dilate(green_mask, kernel) 
  res_green = cv2.bitwise_and(img, img, 
                               mask = green_mask) 
      
# For blue color 
  blue_mask = cv2.dilate(blue_mask, kernel) 
  res_blue = cv2.bitwise_and(img, img, 
                               mask = blue_mask) 

  id_blue(blue_mask)
  # id_red(red_mask)

def id_red(red_mask):
    contours, hierarchy = cv2.findContours(red_mask, 
                                       cv2.RETR_TREE, 
                                       cv2.CHAIN_APPROX_SIMPLE)   
    for contour in contours:    
      area = cv2.contourArea(contour)
      #print(area)                    
      if(area > 300): 
          x,y,w,h = cv2.boundingRect(contour)
          red_frame = cv2.rectangle(img,(x,y),(x+w,x+h),(0,0,255),5)
          print(red_frame)           
          # (x,y),radius = cv2.minEnclosingCircle(contour)
          red_center = (int(x),int(y))
          rCx = red_center[0]
          rCy = red_center[1]
          print("The center for the red circle is: ", red_center)
          # radius = int(radius)
          # img_red = cv2.circle(img,red_center,radius,(0,0,255),2)
          
          img = red_frame
        
          cv2.putText(img, "Red", (int(x), int(y)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                    (0, 0, 255))    
    cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/image_capture_' + str(i) + '.jpg', img)
    i = i+1

def id_blue(blue_mask):
    global img, i
    opening = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    # Creating contour to track blue color 
    # contours, hierarchy = cv2.findContours(blue_mask, 
    #                                          cv2.RETR_TREE, 
    #                                          cv2.CHAIN_APPROX_SIMPLE) 


    x, y, w, h = cv2.boundingRect(opening)
 
    cv2.rectangle(img, (x, y), (x+w, y + h), (0, 255, 0), 3)
    cv2.circle(img, int((x+w/2), int(y+h/2)), 5, (0, 0, 255), -1) 

    # for contour in contours: 
    #         area = cv2.contourArea(contour) 
    #         if(area > 300): 
    #             x,y,w,h = cv2.boundingRect(contour)
    #             blue_frame = cv2.rectangle(img,(x,y),(x+w,x+h),(255,0,0),5)
    #             print(blue_frame)           
    #             # (x,y),radius = cv2.minEnclosingCircle(contour)
    #             blue_center = (int(x),int(y))
    #             blCx = blue_center[0]
    #             blCy = blue_center[1]  
    #             print("The center for the blue circle is: ", blue_center)
    #             # radius = int(radius)
    #             # img_blue = cv2.circle(img,blue_center,radius,(255,0,0),2)
    #             img = blue_frame
    #             cv2.putText(img, "Blue", (int(x), int(y)), 
    #                       cv2.FONT_HERSHEY_SIMPLEX, 
    #                      1.0, (255, 0, 0)) 

    cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/image_capture_' + str(i) + '.jpg', img)
    i = i+1

def id_green(green_mask):          
    # Creating contour to track green color 
    contours, hierarchy = cv2.findContours(green_mask, 
                                             cv2.RETR_TREE, 
                                             cv2.CHAIN_APPROX_SIMPLE) 

    for contour in contours: 
            grn_area = cv2.contourArea(contour)                     
            if (grn_area > 300):             
                (x,y),radius = cv2.minEnclosingCircle(contour)
                green_center = (int(x),int(y))
                gnCx = green_center[0]
                gnCy = green_center[1]
                print("The center for green circle is: ", green_center)
                radius = int(radius)
                img_grn = cv2.circle(img,green_center,radius,(0,255,0),2)            
                img = img_grn

                cv2.putText(img, "Green", (int(x), int(y)), 
                          cv2.FONT_HERSHEY_SIMPLEX,  
                          1.0, (0, 255, 0))

def marker_cb(msg):
    ee_center = msg.data
    ee_x = ee_center[0]
    ee_y = ee_center[0]

    # if len(blue_frame)

    # cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/image_capture_' + str(i) + '.jpg', img)
  
  # gnCxError = 399 - gnCx 
  # gnCyError = 399 - gnCy
  # ErrVect = Float64MultiArray()  # the data to be sent, initialise the array
  # ErrVect.data = [gnCxError, gnCyError] # assign the array with the value you want to send
  # pub1.publish(ErrVect)
  # print(ErrVect)
  
  # if (gnCxError == 0 and gnCyError == 0):
  #   rospy.loginfo(green_center)
  #   rospy.signal_shutdown('need to stop')    
  # else:
  #   rospy.loginfo("Nothing to see here")

 
    
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber("/camera/color/image_raw",Image, image_callback)
  # marker_sub = rospy.subscriber("aruco/Pose", Float32MultiArray, marker_cb)
  
  #pub2 = rospy.Publisher("publishYGreen", Int64, queue_size=0.1)

  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  

if __name__ == '__main__':
    main(sys.argv)
