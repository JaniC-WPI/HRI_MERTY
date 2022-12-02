#!/usr/bin/env python

import speech_recognition as sr
import keyboard

# set up speech recognition object
r = sr.Recognizer()
# set up wait time (s) for command audio recording duration
wait_time = 5

# sets up the audio recording command, prints the text to the terminal window
def decipher_speech(r, wait_time):
    with sr.Microphone() as source:
        # read the audio data from the default microphone
        audio_data = r.record(source, duration=wait_time)
        print("Recognizing...")
        # convert speech to text
        try:
            text = r.recognize_google(audio_data)
            print(text)
        except sr.UnknownValueError as e:
            print("Error: I didn't understand that, could you please repeat yourself by pressing 'space' again?")
        

# set up keyboard hotkey for push-to-talk, triggering the speech recognition
keyboard.add_hotkey('space', lambda: decipher_speech(r,wait_time))
# remind user of the hotkey definition
print("Please press the spacebar to talk, max message time:", wait_time, "seconds")  
# loop repeatedly, waiting for keypress
keyboard.wait()