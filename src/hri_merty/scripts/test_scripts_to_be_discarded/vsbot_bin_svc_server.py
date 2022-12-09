#!/usr/bin/env python3
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from encoderless_vs.srv import bin_img, bin_imgResponse

# Define CvBridge for ROS
bridge = CvBridge()
# image to be used in binary service

current_ros_image = None

# flag to control the control point service request
cp_flag = False

#global variable to publish the flag for a new frame
flag_pub = None

# publish every frame to check the rate with which each image is processed
# frame_pub = None 

# counter to get frame number
i = 0
# the following callback function receives a request from the control points service 
# and responds with a binary image
def binary_image_service(msg):
  global bridge, i
  # Convert ROS image to cv image
  current_cv_image = bridge.imgmsg_to_cv2(current_ros_image, "bgr8")
  # cv2.imshow("CV_Image", cv_image)
  # cv2.waitKey(1) 
  # print("Frame is", i)

  # Visualizing each frame with frame number
  # cv2.putText(cv_image, str(i), (200, 200), 
  #                 cv2.FONT_HERSHEY_SIMPLEX, 
  #                   1.0, (255, 255, 255)) 
  # # cv2.imwrite("/home/janch-ros/Documents/pics/image_" + str(i) + ".jpg", cv_image)
  # cv2.imwrite("image_" + str(i) + ".jpg", cv_image)
  
  i = i+1

  # # publish frame to get frame rate
  # frame = bridge.cv2_to_imgmsg(cv_image, "bgr8")
  # frame_pub.publish(frame)

 
 # converting to hsv
  hsvimg = cv2.cvtColor(current_cv_image, cv2.COLOR_BGR2HSV)
  # Define image color bounds 
  orange_lower = np.array([10, 100, 20], np.uint8) 
  orange_upper = np.array([25, 255, 255], np.uint8) 
  white_lower = np.array([0, 0, 200], np.uint8)
  white_upper = np.array([145, 60, 255], np.uint8) 
  blue_lower = np.array([78,158,124])
  blue_upper = np.array([138,255,255])
	
  # Binarizing individual colors & combining
  orange_mask = cv2.inRange(hsvimg, orange_lower, orange_upper)
  blue_mask = cv2.inRange(hsvimg, blue_lower, blue_upper)
  white_mask = cv2.inRange(hsvimg, white_lower, white_upper)
		
  cv_binary = orange_mask + blue_mask + white_mask

  kernel = np.ones((5, 5), np.uint8)
  cv_binary = cv2.dilate(cv_binary, kernel, iterations=1)

  binary_image = bridge.cv2_to_imgmsg(cv_binary, "mono8")
  
  return bin_imgResponse(binary_image)

def image_callback(ros_image):
  global current_ros_image
  current_ros_image = ros_image
  if current_ros_image is not None:    
    cp_flag = True    
  # if cp_flag == True:  
  flag_pub.publish(cp_flag)   

def main(args):  
  # Initialize ROS
  global flag_pub, frame_pub, binary_vis_pub
  rospy.init_node('image_segmentation')  
  # Declare subcscribers
  # cam sub
  image_sub = rospy.Subscriber("/vsbot/camera1/image_raw",Image,image_callback,queue_size = 1)

  # service declaration to receive the binary image
  bin_img_service = rospy.Service("binary_image_output", bin_img, binary_image_service )
  
  # publisher to publish flag to start control points svc
  flag_pub = rospy.Publisher("/vsbot/control_flag", Bool, queue_size = 1)
  frame_pub = rospy.Publisher("/vsbot/frame_rate", Image, queue_size = 1)

  # while not rospy.is_shutdown():
  #   flag_pub.publish(cp_flag)

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)