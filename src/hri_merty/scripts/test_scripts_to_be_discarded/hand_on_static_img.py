# Importing Libraries
import cv2
import mediapipe as mp
 
# Used to convert protobuf message to a dictionary.
from google.protobuf.json_format import MessageToDict
 
# Initializing the Model
mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=True, model_complexity=1, min_detection_confidence=0.75, min_tracking_confidence=0.75, max_num_hands=2)

img = cv2.imread('/home/jc-merlab/HRI_MERTY/images/Hand_0000002.jpg')

# Convert BGR image to RGB image
imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Process the RGB image
results = hands.process(imgRGB)

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

            if label == 'Left' and (pinky_tip_x > thumb_tip_x):
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

                

            elif label == 'Left' and (pinky_tip_x < thumb_tip_x):
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

            elif label == 'Right' and (pinky_tip_x < thumb_tip_x):
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

            
            elif label == 'Right' and (pinky_tip_x > thumb_tip_x):
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
    cv2.waitKey(100)
    cv2.imwrite('/home/jc-merlab/HRI_MERTY/images/hand_01.jpg', img)

    # you can obtain which hand you're seeing from MULTI_HANDEDNESS label. It will return either "Left" or "Right". Depending on the hand, you can compare landmark locations. For example for the left hand, if THUMB_TIP (index 4)'s x value is smaller than PINKY_TIP (index 20)'s x value it means left hand palm, otherwise it's left hand back. Same thing but reverse calculation applies to right hand.


