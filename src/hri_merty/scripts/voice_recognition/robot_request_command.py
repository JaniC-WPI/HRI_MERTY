#!/usr/bin/env python

# https://pyttsx3.readthedocs.io/en/latest/engine.html#examples 
# https://github.com/D4rkdev1987/virtual-assistant/blob/master/virtualassistant.py 

from multiprocessing.connection import wait
import speech_recognition as sr
import pyttsx3
import time

# begin by setting up the STT speech recognizer:
r = sr.Recognizer()
wait_time = 5 # 5 seconds of verbal command recording

# then set up the TTS engine
def onStart(name): # using these just for debugging atm
    print("speaking now:")
# def onWord(name, location, length):
#     print("why is this happening to me")
def onEnd(name, completed):
    print("done speaking") 
engine = pyttsx3.init()
engine.connect('started-utterance', onStart)
# engine.connect('started-word', onWord)       
engine.connect('finished-utterance', onEnd)

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
            which_obj(text)
            return text
        except sr.UnknownValueError as e:
            # print(("Error: I didn't understand that, could you please repeat yourself by pressing 'space' again?"))
            engine.say("Error: I didn't understand that, could you please repeat yourself?")
            engine.runAndWait()
            decipher_speech(r,wait_time)
# set up desired object determining function:
def which_obj(user_input):
    if "red" in user_input:
        engine.say("red")
        engine.runAndWait()
    elif "green" in user_input:
        engine.say("green")
        engine.runAndWait()
    elif "blue" in user_input:
        engine.say("blue")
        engine.runAndWait()
    elif "yellow" in user_input:
        engine.say("yellow")
        engine.runAndWait()
    else:
        engine.say("no color found")
        engine.runAndWait()
# now can set up the initial action: Robot asks user which block to pick up:
engine.say("Which block would you like me to pick up?")
engine.runAndWait()
# record user voice input (somewhere between 5 to 10 seconds of recording)
uin = decipher_speech(r, wait_time)
# search user voice input for the relevant command information (RED, YELLOW, BLUE, GREEN)
#   if relevant command is not found, ask user again (formulate this as a try catch)

#   if relevant command is found, say which object will be picked up

# end test case code