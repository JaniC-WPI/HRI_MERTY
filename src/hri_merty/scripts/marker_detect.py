#!/usr/bin/env python3

import rospy
# import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from std_msgs.msg import Float64MultiArray
import sys

bridge = CvBridge()
ros_img = None
# base_marker_center = []
# ee_marker_center = []
ee_center = []
base_index = None
ee_index = None
marker_flag = False
ee_corners_list = []


# Subscriber callback
def marker_pose(img_msg):
    global bridge, ros_img, ee_center, base_index, ee_index, marker_flag, ee_corners_list
    cv_img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    if ros_img is not None:

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY) 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) 

        arucoParameters = aruco.DetectorParameters_create() 

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)

        # print("Id generated", ids) #Remove
        id_list = []

        id_list.clear()
        for i in ids:
            if int(i) == 32:
                id_list.append(int(i))
            elif int(i) == 34:
                id_list.append(int(i))

        # Aruco ID for each marker
        base_id = 34
        ee_id = 32

        # # Check if both markers are found
        # try:    
        #     base_index = id_list.index(base_id)
        #     marker_flag = True
        # except:
        #     marker_flag = False
        try:
            ee_index = id_list.index(ee_id)
            marker_flag = True
        except:
            marker_flag = False

        # if not marker_flag:
        #     print("MARKER NOT FOUND")
        # Separating the corner pixel co-ordinates for each marker
        if marker_flag:
            print("Marker flag found")
            # base_corners_list = corners[base_index].reshape(4,2)   # [[x1, y1],[x2, y2],[x3, y3],[x4, y4]]
            ee_corners_list = corners[ee_index].reshape(4,2)
            # print("latest ee corner list", ee_corners_list) #Remove


            # print("ee corner list", ee_corners_list) #Remove

            # Averaging base corner co-ordinates to obtain marker center
            # base_center_x = (base_corners_list[0][0] + base_corners_list[1][0] + base_corners_list[2][0] + base_corners_list[3][0])/4
            # base_center_y = (base_corners_list[0][1] + base_corners_list[1][1] + base_corners_list[2][1] + base_corners_list[3][1])/4

            # base_center = [base_center_x, base_center_y]

            # Averaging ee corner co-ordinates to obtain marker center
            ee_center_x = (ee_corners_list[0][0] + ee_corners_list[1][0] + ee_corners_list[2][0] + ee_corners_list[3][0])/4
            ee_center_y = (ee_corners_list[0][1] + ee_corners_list[1][1] + ee_corners_list[2][1] + ee_corners_list[3][1])/4

            ee_center = [ee_center_x, ee_center_y]

            print(ee_center)

            # Compute ee_center_co-ordinate w.r.t base marker
            # ee_center = [ee_center_x - base_center_x, ee_center_y - base_center_y]
            # Draw a box on detected markers
            cv_img = aruco.drawDetectedMarkers(cv_img, corners)
            # Draw the centers
            # cv2.circle(cv_img, (int(base_center_x), int(base_center_y)), 4, [0, 255, 255], -1)
            cv2.circle(cv_img, (int(ee_center_x), int(ee_center_y)), 4, [255, 255, 0], -1)
            # Convert back to ros img to publish
            cv2.imwrite( cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/image_capture_' + str(i) + '.jpg', cv_img))

            i = i+1

def main(args):
    # Initialize ROS
    rospy.init_node('marker_detect')
    pub_flag = True
    # Listening to the start flag

    # Subscribers
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, marker_pose, queue_size=1)

    # Publishers
    marker_detect_pub = rospy.Publisher("aruco/result", Image, queue_size=1)
    marker_pose_pub = rospy.Publisher("/aruco/Pose", Float64MultiArray, queue_size=1)

    # Rate Loop to publish
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        # Convert pose to rosmsg
        ee_marker = Float64MultiArray()
        ee_marker.data = ee_center

        # Publish marker pose
        if ee_center != [] and pub_flag == True:
            marker_pose_pub.publish(ee_marker)
            pub_flag = False

        # Publish image
        if ros_img:
            marker_detect_pub.publish(ros_img)

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
