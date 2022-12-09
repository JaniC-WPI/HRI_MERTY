#!/usr/bin/env python

# https://pyttsx3.readthedocs.io/en/latest/engine.html#examples 
# https://github.com/D4rkdev1987/virtual-assistant/blob/master/virtualassistant.py 

# python TTS and STT import
import speech_recognition as sr
import pyttsx3
import time

from sympy import vfield

# ROS node imports
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64

# begin by setting up the STT speech recognizer:
r = sr.Recognizer()
wait_time = 5 # 5 seconds of verbal command recording

# then set up the TTS engine
def onStart(name): # using these just for debugging atm
    print("speaking now:")
def onEnd(name, completed):
    print("done speaking") 

# desired block color command flag message definition:
cmd_flag = String()

# voice feedback callback definition
vF_req = 0 # default voice feedback condition is none
vF_ = "" # default voice feedback is none
global fbRec_
fbRec_ = False # default feedback requested is none
def objNotFoundCallback(data):
    vF_req = data.data

def voiceFeedbackCallback(data):
    vF_ = data.data
    fbRec_ = True
    # engine.say(data.data)
    # engine.runAndWait()

# set up speech recording function definition: 
def decipher_speech(r, wait_time):
    with sr.Microphone() as source:
        print("Please say your command:")
        # read the audio data from the default microphone
        audio_data = r.record(source, duration=wait_time)
        print("Recognizing...")
        # convert speech to text
        try:
            text = r.recognize_google(audio_data)
            print(text)
            if which_obj(text):
                return True
            else: # try again
                # decipher_speech(r,wait_time)
                return False
        #   if relevant command is not found, ask user again
        except sr.UnknownValueError as e:
            engine.say("Error: I didn't understand that, could you please repeat yourself?")
            engine.runAndWait()
            # decipher_speech(r,wait_time)
            return False
# set up desired object determining function:
# search user voice input for the relevant command information (RED, YELLOW, BLUE, GREEN)
def which_obj(user_input):
    if "red" in user_input:
        engine.say("picking up the red object")
        engine.runAndWait()
        cmd_flag.data = "red"
        flag_pub.publish(cmd_flag)
        return True
    elif "green" in user_input:
        engine.say("picking up the green object")
        engine.runAndWait()
        cmd_flag.data = "green"
        flag_pub.publish(cmd_flag)
        return True
    elif "blue" in user_input:
        engine.say("picking up the blue object")
        engine.runAndWait()
        cmd_flag.data = "blue"
        flag_pub.publish(cmd_flag)
        return True
    else:
        engine.say("invalid request, please ask for a valid-colored object on the table")
        engine.runAndWait()
        return False

def voice_state_machine(state):
    global fbRec_
    if(state == -1): # first desired object query
        engine.say("which block would you like me to pick up?")
        engine.runAndWait()
        command_found_ = decipher_speech(r,wait_time)
        if command_found_:
            vF_req = 0
            return 
        else:
            vF_req = -1 # just in case, ensure stays in state until valid object request is found
            return
    elif(state == 0): # IDLE state, waiting for any feedbacks
        print("publishing flag: ", cmd_flag.data)
        flag_pub.publish(cmd_flag)
        if fbRec_:
            engine.say(vF_)
            engine.runAndWait()
            fbRec_ = False
    elif(state == 1): # RED object not found
        engine.say("Could not find the red object")
        engine.runAndWait()
        vF_req = -1 # send back to the query state
    elif(state == 2): # BLUE object not found
        engine.say("Could not find the blue object")
        engine.runAndWait()
        vF_req = -1 # send back to the query state
    elif(state == 3): # GREEN object not found
        engine.say("Could not find the green object")
        engine.runAndWait()
        vF_req = -1 # send back to the query state

# initialize ros node and main loop
if __name__ == '__main__':
    rospy.init_node('voiceFeedbackNode',anonymous=True)
    
    # set up ROS publisher for sending color flags
    flag_pub = rospy.Publisher("/color/flag",String,queue_size = 1)
    # set up ROS subscriber for voice feedback requests
    rospy.Subscriber("/voice/feedback",Int64,objNotFoundCallback)
    rospy.Subscriber("/hri/text2speech",String,voiceFeedbackCallback)
    # set up a reasonable rate for the rospy loop for handling everything
    rate = rospy.Rate(10) # 10Hz to start
    vF_req = -1 # case for initial ask to the user
    vF_ = "" # there is no initial voice feedback
    fbRec_ = False # there is no initial feedback request

    engine = pyttsx3.init()
    # these connections are only for debugging, use if wondering why speech hangs
    # engine.connect('started-utterance', onStart)
    # engine.connect('finished-utterance', onEnd)
    # adjust speed of TTS:
    engine.setProperty("rate",151) # rate is in words per minute 
    voices = engine.getProperty("voice")
    engine.setProperty("voice", voices[2].id) # voices are different per computer

    while not rospy.is_shutdown():
        voice_state_machine(vF_req)
        rate.sleep()

    rospy.spin()


