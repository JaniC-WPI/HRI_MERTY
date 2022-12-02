#!/usr/bin/env python

# must run pip install pyttsx3 before using this code:
import pyttsx3

print("Initializing text-to-speech engine")
engine = pyttsx3.init()
print("finished initializing text-to-speech engine")

engine.say("my first text-to-speech")
print("saying command")
engine.runAndWait()
print("run and wait finished")