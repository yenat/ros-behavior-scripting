#!/usr/bin/env python
from collections import deque
import rospy
import math
import time
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from netcat import netcat
from opencog.atomspace import *

'''
   This is a sample subscriber written to check the class of AudioInput.
   Classification is as Quiet, Normal Conversation, Loud conversation and
   critical sound.
'''

class AudioStrength:
  d = 0
  Decibel = None

  def __init__(self):
    self.hostname = "localhost"
    self.port = 17020
    self.loop = 0
    rospy.Subscriber("/opencog/AudioFeature", String, self.GetAudioClass)
    rospy.Subscriber("/opencog/suddenchange", String, self.GetSuddenClass)

  def AudioEnergy(self, value):
    deci = "(EvaluationLink (PredicateNode \"Decibel value\") " + \
		       "(ListLink (NumberNode \"" + str(value) + "\")))\n"

    netcat(self.hostname, self.port, deci + "\n")

    # TODO: Convert each if/elif clause into a psi-rule.
    if value < 35:
        print "very low sound", value
        x = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"happy\") (NumberNode 3) (NumberNode 0.5))))\n"


        netcat(self.hostname, self.port, x + "\n")

        return 'Quiet Whisper'

    elif value < 65:
        print "Normal conversation:", value

        y = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"amused\") (NumberNode 3) (NumberNode 0.5))))\n"

        netcat(self.hostname, self.port, y + "\n")

        m = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
       "(ListLink (ConceptNode \"nod-1\") (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"

        netcat(self.hostname, self.port, m + "\n")

        return 'Normal Conversation'

    elif value < 90:
        print "high sound:", value
        z = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"irritated\") (NumberNode 3) (NumberNode 0.5))))\n"
        netcat(self.hostname, self.port, z + "\n")

        n = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show gesture\")" + \
       "(ListLink (ConceptNode \"think-browsDown.001\") (NumberNode 0.2) (NumberNode 2) (NumberNode 0.8))))\n"

        netcat(self.hostname, self.port, n + "\n")
        return 'Loud: Shouted Conversation'
    else:
        print 'critical:', value
        t = "(cog-evaluate! (EvaluationLink  (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"afraid\") (NumberNode 3) (NumberNode 0.5))))\n"
        netcat(self.hostname, self.port, t + "\n")
        return 'Loud: Critical'

  def GetAudioClass(self, data):
    try:
        features = (data.data).split("_")
        self.Decibel = float(features[0])
        Frequency = float(features[1])

        if self.loop <=2:
            d.append(self.Decibel)

            self.loop += 1
        else:
            d.popleft();d.append(self.Decibel)
            self.loop += 1

    except ArithmeticError as e:
        print(e)
    return self.Decibel

  def GetSuddenClass(self, msg):
    try:
        change = msg.data

        if int(change) > 1:
            print 'sudden change'
            l = "(cog-evaluate! (EvaluationLink (DefinedPredicateNode \"Show expression\")" + \
       "(ListLink (ConceptNode \"surprised\") (NumberNode 2) (NumberNode 0.5))))\n"

            netcat(self.hostname, self.port, l + "\n")

        else:
            self.AudioEnergy(self.Decibel)

    except ArithmeticError as e:
        print(e)

if __name__ == '__main__':
    global d
    d =deque()
    try:
        rospy.init_node('AudioClass', anonymous=True)
        AudioStrength()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
