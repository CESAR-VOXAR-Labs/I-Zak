#!/usr/bin/env python

"""
recognizer.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
  publications:
    ~output (std_msgs/String) - text output
  services:
    ~start (std_srvs/Empty) - start speech recognition
    ~stop (std_srvs/Empty) - stop speech recognition
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy

import time

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

import subprocess

from std_msgs.msg import String
from std_srvs.srv import *

class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):

        """ Initialize the speech pipeline components. """
        rospy.init_node('recognizer')
        self.pub = rospy.Publisher('~output',String)
        rospy.on_shutdown(self.shutdown)

        # services to start/stop recognition
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)

#        self.p1 = subprocess.Popen(["/usr/bin/jackd", "-r", "-dalsa","-p1024","-n2","-r44100","-dhw:0"])


#        time.sleep(2)

        # configure pipeline
        self.pipeline = gst.parse_launch('jackaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! filesink name=filesink')


        filesink = self.pipeline.get_by_name('filesink')
        filesink.set_property('location', "/home/robot/output.wav")


        print "READY"
        

        self.start(None)
        gtk.main()
        
    def shutdown(self):
        """ Shutdown the GTK thread. """
#        self.p1.kill()
        gtk.main_quit()

    def start(self, msg):
        self.pipeline.set_state(gst.STATE_PLAYING)
        return EmptyResponse()

    def stop(self):
        self.pipeline.set_state(gst.STATE_PAUSED)
        #vader = self.pipeline.get_by_name('vad')
        #vader.set_property('silent', True)
        return EmptyResponse()



if __name__=="__main__":
    r = recognizer()

