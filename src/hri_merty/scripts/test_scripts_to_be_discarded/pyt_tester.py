#!/usr/bin/env python
# https://pyttsx3.readthedocs.io/en/latest/engine.html#examples 

import pyttsx3
def onStart(name):
   print('starting', name)
def onWord(name, location, length):
   print ('word', name, location, length)
def onEnd(name, completed):
   print ('finishing', name, completed)
engine = pyttsx3.init()
engine.connect('started-utterance', onStart)
engine.connect('started-word', onWord)
engine.connect('finished-utterance', onEnd)
for i in range(2):
   engine.say('The quick brown fox jumped over the lazy dog.')
   engine.runAndWait()