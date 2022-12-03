#!/usr/bin/env python

# https://pyttsx3.readthedocs.io/en/latest/engine.html#examples 

import speech_recognition as sr
import keyboard
import pyttsx3

# set up speech recognition object
r = sr.Recognizer()
# set up text-to-speech engine
# set up wait time (s) for command audio recording duration
wait_time = 5

# set up triggers for text to speech engine (to avoid hangups)
def onStart(name):
    print("speaking now:")
def onWord(name, location, length):
    print("why is this happening to me")
def onEnd(name, completed):
    print("done speaking") 
engine = pyttsx3.init()
engine.connect('started-utterance', onStart)
engine.connect('started-word', onWord)       
engine.connect('finished-utterance', onEnd)

# sets up the audio recording command, prints the text to the terminal window
def decipher_speech(r, wait_time):
    with sr.Microphone() as source:
        print("Please say your command:")
        # engine.say("Please say your command:")
        # engine.runAndWait()
        # read the audio data from the default microphone
        audio_data = r.record(source, duration=wait_time)
        print("Recognizing...")
        # convert speech to text
        try:
            text = r.recognize_google(audio_data)
            print(text)
        except sr.UnknownValueError as e:
            print(("Error: I didn't understand that, could you please repeat yourself by pressing 'space' again?"))
            # engine.say("Error: I didn't understand that, could you please repeat yourself by pressing 'space' again?")
            # engine.runAndWait()

        

# set up keyboard hotkey for push-to-talk, triggering the speech recognition
keyboard.add_hotkey('space', lambda: decipher_speech(r,wait_time))
# remind user of the hotkey definition
print("Please press the spacebar to talk, max message time:", wait_time, "seconds")  
# loop repeatedly, waiting for keypress
keyboard.wait()