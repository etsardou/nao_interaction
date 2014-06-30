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
  TactileTouch,
  SpeechWithFeedbackAction,
  SpeechWithFeedbackGoal
)

from nao_interaction_msgs.msg import(
  FaceDetected
)
  
from std_srvs.srv import (
  Empty,
  EmptyResponse
)
#~ You must run:
#~  roslaunch nao_driver nao_driver.launch
#~  roslaunch nao_interaction_launchers nao_vision_interface.launch
#~ before this node
class TrackFace():
  def __init__(self, nodeName):
    
    # ROS node initialization
    rospy.init_node(nodeName)
    
    #~ ROS initializations
    self.stiffnessOnService = rospy.ServiceProxy('body_stiffness/enable', Empty)
    self.stiffnessOffService = rospy.ServiceProxy('body_stiffness/disable', Empty)
    self.faceDetectionOnService = rospy.ServiceProxy('nao_vision/face_detection/enable', Empty)
    self.faceDetectionOffService = rospy.ServiceProxy('nao_vision/face_detection/disable', Empty)
    rospy.Subscriber("/tactile_touch", TactileTouch, self.tactileSubscriber)
    rospy.Subscriber("/nao_vision/faces_detected", FaceDetected, self.faceDetected)
    self.speechClient = actionlib.SimpleActionClient('speech_action', SpeechWithFeedbackAction)
    self.speakGoal = SpeechWithFeedbackGoal()
    
    #~ Variables / Initializations
    self.tactile_lock = False
    self.track_mode = False
    
  def tactileSubscriber(self, values):
        
    if(self.tactile_lock or values.state != 1):
        return
        
    self.tactile_lock = True
    if(self.track_mode == False):
      self.speakGoal.say = "Tracking starting"
      self.speechClient.send_goal(self.speakGoal)
      self.speechClient.wait_for_result()
      self.track_mode = True
      self.faceDetectionOnService()
      self.stiffnessOnService()
    else:
      self.track_mode = False
      self.stiffnessOffService()
      self.faceDetectionOffService()
      self.speakGoal.say = "Tracking stopped"
      self.speechClient.send_goal(self.speakGoal)
      self.speechClient.wait_for_result()
      
    self.tactile_lock = False
  
  def faceDetected(self, values):
    print "aaaa"

if __name__ == '__main__':

  ShowcaseNode = TrackFace("nao_track_face")
  rospy.loginfo("nao_track_face started")
  rospy.spin()
  exit(0)
