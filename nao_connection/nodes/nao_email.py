#!/usr/bin/env python

#
# ROS node to serve NAOqi Email functionalities
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
import naoqi

import smtplib
from email.mime.text import MIMEText

from naoqi import ( 
    ALModule, 
    ALBroker, 
    ALProxy)
    
from nao_driver import NaoNode

#~ ROS msgs
from std_msgs.msg import (
    String, 
    Float32, 
    Int32)
    
from std_srvs.srv import (
    EmptyResponse,
    Empty)
    
#~ nao-ros msgs
#~ from nao_interaction_msgs.msg import ()
    
#~ from nao_interaction_msgs.srv import ()

class Constants:
    NODE_NAME = "nao_email_interface"

class NaoEMailInterface(ALModule, NaoNode):
    "sss"
    def __init__(self, moduleName):
        # ROS initialization
        NaoNode.__init__(self)
        rospy.init_node( Constants.NODE_NAME )
        
        # NAOQi initialization
        self.ip = ""
        self.port = 10603
        self.moduleName = moduleName
        self.init_almodule()

        self.subscribe()
        
        #~ msg = MIMEText("test")
        #~ msg['Subject'] = "test subject"
        #~ msg['From'] = "etsardou@gmail.com"
        #~ msg['To'] = "etsardou@gmail.com"
        #~ 
        #~ s = smtplib.SMTP('localhost')
        #~ s.sendmail(msg['From'], [msg['To']], msg.as_string())
        #~ s.quit()
        
        rospy.loginfo("nao_email_interface initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)
        
        self.memProxy = ALProxy("ALMemory",self.pip,self.pport)
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)

    def shutdown(self): 
        self.unsubscribe()

    def subscribe(self):
        pass

    def unsubscribe(self):
        pass
        
if __name__ == '__main__':
  
    ROSNaoMailModule = NaoEMailInterface("ROSNaoMailModule")
    rospy.spin()

    rospy.loginfo("Stopping ROSNaoMailModule ...")
    ROSNaoMailModule.shutdown();        
    rospy.loginfo("ROSNaoMailModule stopped.")
    exit(0)
