#!/usr/bin/env python

#
# ROS showcase node.
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2014 Manos Tsardoulias, CERTH/ITI
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the CERTH/ITI nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import actionlib

from std_msgs.msg import (
  String, 
  Int32
)

from nao_msgs.msg import (
  Bumper,
  TactileTouch,
  SpeechWithFeedbackAction,
  SpeechWithFeedbackGoal
)

#~ nao_driver must be activated
class TouchAndTalk():
  def __init__(self, nodeName):
    
    # ROS node initialization
    rospy.init_node(nodeName)
    
    #~ ROS initializations
    rospy.Subscriber("/bumper", Bumper, self.bumperSubscriber)
    rospy.Subscriber("/tactile_touch", TactileTouch, self.tactileSubscriber)
    self.speechClient = actionlib.SimpleActionClient('speech_action', SpeechWithFeedbackAction)
    
    self.speakGoal = SpeechWithFeedbackGoal()
    
    self.bumper_lock = False;
    self.tactile_lock = False;
              
  def bumperSubscriber(self, values):
      
    if(self.bumper_lock or values.state != 1):
        return
        
    self.bumper_lock = True
    
    self.speakGoal.say = "You have touched my "
    if(values.bumper == 0):
        self.speakGoal.say += "right"
    else:
        self.speakGoal.say += "left"
    self.speakGoal.say += " foot"
    
    self.speechClient.send_goal(self.speakGoal)
    self.speechClient.wait_for_result()
    self.bumper_lock = False;
      
  def tactileSubscriber(self, values):
    
    if(self.tactile_lock or values.state != 1):
        return
        
    self.tactile_lock = True
    
    self.speakGoal.say = "You have touched my "
    if(values.button == 1):
        self.speakGoal.say += "front"
    elif (values.button == 2):
        self.speakGoal.say += "middle"
    elif (values.button == 3):
        self.speakGoal.say += "rear"
    self.speakGoal.say += " tactile"
    
    self.speechClient.send_goal(self.speakGoal)
    self.speechClient.wait_for_result()
    self.tactile_lock = False;
      

if __name__ == '__main__':

  ShowcaseNode = TouchAndTalk("nao_touch_and_talk")
  rospy.loginfo("nao_touch_and_talk started")
  rospy.spin()
  exit(0)
