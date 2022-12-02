# Importing Libraries
import cv2
import mediapipe as mp
 
# Used to convert protobuf message to a dictionary.
from google.protobuf.json_format import MessageToDict
 
# Initializing the Model
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False, model_complexity=1, min_detection_confidence=0.75, min_tracking_confidence=0.75, max_num_hands=2)
 
# Start capturing video from webcam
cap = cv2.VideoCapture(0)
 
while True:
    # Read video frame by frame
    success, img = cap.read()
 
    # Flip the image(frame)
    # img = cv2.flip(img, 1)
 
    # Convert BGR image to RGB image
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
 
    # Process the RGB image
    results = hands.process(imgRGB)
 
    # If hands are present in image(frame)
    # If hands are present in image(frame)
    if results.multi_hand_landmarks:

        for hand_landmarks in results.multi_hand_landmarks:
            pinky_tip_x = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_DIP].x
            thumb_tip_x = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_TIP].x
            print(pinky_tip_x)


        # for hand_no, hand_landmarks in enumerate(results.multi_hand_landmarks):
        #     # print("Hand Number", hand_no)
        #     # print("Hand Landmarks", (hand_landmarks))
        #     for i in range(20):
        #         print(mpHands.HandLandmark(i).name)
        #         print(hand_landmarks.landmark[mpHands.HandLandmark(i).value])

        #         if mpHands.HandLandmark(i).name == 'PINKY_DIP':
        #             # pinky_tip_x = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_DIP].x
        #             print('PINKY TIP x', hand_landmarks.landmark[mpHands.HandLandmark(i).value].x)




        # Both Hands are present in image(frame)
        if len(results.multi_handedness) == 2:
                # Display 'Both Hands' on the image
            cv2.putText(img, 'Both Hands', (250, 50),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.9, (0, 255, 0), 2)

        # If any hand present
        else:
            for i in results.multi_handedness:
            
                # Return whether it is Right or Left Hand
                label = MessageToDict(i)['classification'][0]['label']

                if label == 'Left' and (pinky_tip_x < thumb_tip_x):
                    print("Palm Open")

                    # Display 'Left Hand' on
                    # left side of window
                    # cv2.putText(img, label+' Hand',
                    #             (20, 50),
                    #             cv2.FONT_HERSHEY_COMPLEX,
                    #             0.9, (0, 255, 0), 2)

                    cv2.putText(img, 'open palm',
                                (20, 50),
                                cv2.FONT_HERSHEY_COMPLEX,
                                0.9, (0, 255, 0), 2)



                elif label == 'Left' and (pinky_tip_x > thumb_tip_x):
                    print("Palm Closed")

                    # Display 'Left Hand' on
                    # left side of window
                    # cv2.putText(img, label+' Hand',
                    #             (20, 50),
                    #             cv2.FONT_HERSHEY_COMPLEX,
                    #             0.9, (0, 255, 0), 2)

                    cv2.putText(img, 'closed palm',
                                (20, 50),
                                cv2.FONT_HERSHEY_COMPLEX,
                                0.9, (0, 255, 0), 2)

                elif label == 'Right' and (pinky_tip_x > thumb_tip_x):
                    print("Palm Open")

                    # Display 'Left Hand' on
                    # left side of window
                    # cv2.putText(img, label+' Hand',
                    #             (20, 50),
                    #             cv2.FONT_HERSHEY_COMPLEX,
                    #             0.9, (0, 255, 0), 2)

                    cv2.putText(img, 'open palm',
                                (20, 50),
                                cv2.FONT_HERSHEY_COMPLEX,
                                0.9, (0, 255, 0), 2)


                elif label == 'Right' and (pinky_tip_x < thumb_tip_x):
                    print("Palm Closed")

                    # Display 'Left Hand' on
                    # left side of window
                    # cv2.putText(img, label+' Hand',
                    #             (20, 50),
                    #             cv2.FONT_HERSHEY_COMPLEX,
                    #             0.9, (0, 255, 0), 2)

                    cv2.putText(img, 'closed palm',
                                (20, 50),
                                cv2.FONT_HERSHEY_COMPLEX,
                                0.9, (0, 255, 0), 2)



        # Display Video and when 'q'
        # is entered, destroy the window
        cv2.imshow('Image', img)
        cv2.waitKey(10)
        cv2.imwrite('/home/jc-merlab/HRI_MERTY/images/hand_02.jpg', img)

    # cv2.imwrite('/images/hand', img)
