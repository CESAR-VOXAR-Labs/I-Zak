#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from std_srvs.srv import *

from subprocess import call

def chat_callback(mydata):
    global restart, recstop

    text = mydata.data

    args = "--host -vdfki-poppy " + text

    # mute the mic
    call(["pacmd","set-source-mute 1 1"])
    try:
        recstop
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # speak the utterance
    call(["maryspeak",args])

    # unmute the mic
    call(["pacmd","set-source-mute 1 0"])
    try:
        recstart
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def speechsynth():
    global recstart, recstop

    rospy.init_node('speechsynth', anonymous=True)

    rospy.Subscriber("/chatter", String, chat_callback)

    rospy.wait_for_service('/recognizer/start')
    rospy.wait_for_service('/recognizer/stop')

    try:
        recstart = rospy.ServiceProxy('/recognizer/start', Empty)
        recstop = rospy.ServiceProxy('/recognizer/stop', Empty)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    try:
        recstop
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # mute / unmute to set volume
    call(["pacmd","set-source-mute 1 1"])
    call(["pacmd","set-source-mute 1 0"])

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#    while not rospy.is_shutdown():

#        time.sleep(0.01)



if __name__ == '__main__':
    try:
        speechsynth()
    except rospy.ROSInterruptException: pass



