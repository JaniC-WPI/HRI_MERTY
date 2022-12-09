#!/usr/bin/env python
# tutorial: https://www.geeksforgeeks.org/python-text-to-speech-pyttsx-module/ 
# how to use pyttsx3: https://pyttsx3.readthedocs.io/en/latest/engine.html#examples 

# must run pip install pyttsx3 before using this code:
import pyttsx3

print("Initializing text-to-speech engine")
engine = pyttsx3.init()
voices = engine.getProperty('voices')
engine.setProperty("voice",voices[2].id)
engine.setProperty("rate",151)
print("finished initializing text-to-speech engine")

engine.say("my first text-to-speech")
print("saying command")
engine.runAndWait()
print("run and wait finished")
