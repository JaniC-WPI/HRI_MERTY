# Importing Libraries
import cv2
import mediapipe as mp
import numpy as np
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
 
# Used to convert protobuf message to a dictionary.
from google.protobuf.json_format import MessageToDict

bridge = CvBridge()
ros_img = None
text2speech = None
speech_pub = None

def img_callback(msg):
    global ros_img, text2speech
    ros_img = msg
    # Initializing the Model
    mpHands = mp.solutions.hands
    hands = mpHands.Hands(static_image_mode=False, model_complexity=1, min_detection_confidence=0.75, min_tracking_confidence=0.75, max_num_hands=2)
    # print(ros_img)

    if ros_img is not None:
        control_flag = True
    if control_flag == True:
        cv_img = bridge.imgmsg_to_cv2(ros_img, "rgb8")

        cv_img = cv2.flip(cv_img, 1)
        # Process the RGB image
        results = hands.process(cv_img)
    
        # If hands are present in image(frame)
        # If hands are present in image(frame)
        if results.multi_hand_landmarks:
            print("is there a hand? ", results.multi_hand_landmarks)

            for hand_landmarks in results.multi_hand_landmarks:
                pinky_tip_x = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_DIP].x
                thumb_tip_x = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_TIP].x
                wrist_tip_x = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_MCP].x*1280
                wrist_tip_y = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_MCP].y*720
                mid_fing_x = hand_landmarks.landmark[mpHands.HandLandmark.MIDDLE_FINGER_MCP].x*1280
                mid_fing_y = hand_landmarks.landmark[mpHands.HandLandmark.MIDDLE_FINGER_MCP].y*720
                print("mid_fing", mid_fing_x, mid_fing_y)
                print("wrist", wrist_tip_x, wrist_tip_y )

                # mid_fing 599.7006607055664 435.47019481658936
                # 524.8818969726562 453.1249666213989



            # Both Hands are present in image(frame)
            if len(results.multi_handedness) == 2:
                    # Display 'Both Hands' on the image
                cv2.putText(cv_img, 'Both Hands', (250, 50),
                            cv2.FONT_HERSHEY_COMPLEX,
                            0.9, (0, 255, 0), 2)

                text2speech = "I can see both hands. Please put out your hand correctly"
                speech_pub.publish(text2speech)
                print(text2speech)


            # If any hand present
            else:
                for i in results.multi_handedness:
                
                    # Return whether it is Right or Left Hand
                    label = MessageToDict(i)['classification'][0]['label']
                    if (label == 'Left') and (pinky_tip_x < thumb_tip_x):
                        print("Palm Closed")

                        # Display 'Left Hand' on
                        # left side of window
                        # cv2.putText(img, label+' Hand',
                        #             (20, 50),
                        #             cv2.FONT_HERSHEY_COMPLEX,
                        #             0.9, (0, 255, 0), 2)

                        cv2.putText(cv_img, 'closed palm',
                                    (20, 50),
                                    cv2.FONT_HERSHEY_COMPLEX,
                                    0.9, (0, 255, 0), 2)

                        text2speech = "Please put out your hand correctly"
                        speech_pub.publish(text2speech)

                    elif (label == 'Left') and (pinky_tip_x > thumb_tip_x):
                        print("Palm Open")

                        # Display 'Left Hand' on
                        # left side of window
                        # cv2.putText(img, label+' Hand',
                        #             (20, 50),
                        #             cv2.FONT_HERSHEY_COMPLEX,
                        #             0.9, (0, 255, 0), 2)

                        cv2.putText(cv_img, 'open palm',
                                    (20, 50),
                                    cv2.FONT_HERSHEY_COMPLEX,
                                    0.9, (0, 255, 0), 2)

                        if (515 <=mid_fing_x <=535) and (443 <=mid_fing_y <=463):
                            text2speech = "Thank you. Now I am going to place the object on your palm"
                            speech_pub.publish(text2speech)

                    elif (label == 'Right') and (pinky_tip_x < thumb_tip_x):
                        print("Palm Open")

                        # Display 'Left Hand' on
                        # left side of window
                        # cv2.putText(img, label+' Hand',
                        #             (20, 50),
                        #             cv2.FONT_HERSHEY_COMPLEX,
                        #             0.9, (0, 255, 0), 2)

                        cv2.putText(cv_img, 'open palm',
                                    (20, 50),
                                    cv2.FONT_HERSHEY_COMPLEX,
                                    0.9, (0, 255, 0), 2)
                        if (590 <=mid_fing_x <=610) and (425 <=mid_fing_y <=445):
                            text2speech = "Thank you. Now I am going to place the object on your palm"
                            speech_pub.publish(text2speech)


                    elif (label == 'Right') and (pinky_tip_x > thumb_tip_x):
                        print("Palm Closed")

                        # Display 'Left Hand' on
                        # left side of window
                        # cv2.putText(img, label+' Hand',
                        #             (20, 50),
                        #             cv2.FONT_HERSHEY_COMPLEX,
                        #             0.9, (0, 255, 0), 2)

                        cv2.putText(cv_img, 'closed palm',
                                    (20, 50),
                                    cv2.FONT_HERSHEY_COMPLEX,
                                    0.9, (0, 255, 0), 2)

                        text2speech = "Please put out your hand correctly"
                        speech_pub.publish(text2speech)

        else:
            text2speech = "I cannot see any hands. Please put out your hand correctly" 
            speech_pub.publish(text2speech)
            print(text2speech)
        
                    


        # Display Video and when 'q'
        # is entered, destroy the window
        cv2.imwrite('/home/jc-merlab/Pictures/Merty/saved_images/hands/hand.jpg', cv_img)
            # i = i+1


def main():
    global speech_pub, text2speech
    # Initialize the node
    rospy.init_node('palm_identifier')
    print("is main getting called")
    # subscriber for rgb image to detect markers
    image_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, img_callback, queue_size=1)
    
    # status_sub = rospy.Subscriber("/node/stop", Float64, stop_cb, queue_size=1)
    # publish the text message to be converted to speech

    speech_pub = rospy.Publisher('/hri/text2speech/', String, queue_size=1)
    # text2speech = String
    # publisher to publish flag to start control points svc
    # publish_string = rospy.Publisher("/franka/control_flag", Bool, queue_size = 1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("running while loop")
        # speech_pub.publish(text2speech)
    #     print("Camera K", camera_K)
    #     if camera_K is not None and control_flag == True:
    #         camera_ext = transform(tvec, quat)
    #         print("call image_pix")
    #         # print("camera_ext", camera_ext)
    #         image_pix = image_pixels(camera_ext, world_coords)
    #         print(image_pix)
    #         print("call kp_gen")
    #         kp_gen(control_flag, ros_img)  

    #     if status == 0.0:
    #         rospy.signal_shutdown("I have good reason!")
        rate.sleep()
    #     # print(image_pix)  

    rospy.spin()


if __name__ == "__main__":
    main()
        
