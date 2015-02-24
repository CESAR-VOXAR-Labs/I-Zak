#!/usr/bin/env python

# (C) CESAR 2014 
# BSD license


# pseudo-state gatherer for robot; subscribes to hw controller topics

# can send a bang and it will publish the whole state


import rospy

from std_msgs.msg import String
from std_srvs.srv import *



def armstate_callback(data):
    global arx, ary, arz, gr, gp, gval

    # got message from the arm controller
    # msg format is: x,y,z (for cartesian)
    # gripper rot, gripper pitch, gripper open
    # msg count

    df = data.data.split(' ')

    arx = df[0]
    ary = df[1]
    arz = df[2]
    gr = df[3]
    gp = df[4]
    gval = df[5]




def wheelstate_callback(data):
    global xpos, ypos, apos, xvel, yvel, avel

    # got message from the wheel controller
    # msg format is: x,y pos (MAP coordinates, origin at launch)
    # x, y vel (x is current fwd of robot)
    # ang pos, ang vel (radians, MAP coordinates)
    # msg count

    df = data.data.split(' ')

    xpos = df[0]
    ypos = df[1]
    apos = df[2]
    xvel = df[3]
    yvel = df[4]
    avel = df[5]



def combine_status():
    global xpos, ypos, xvel, yvel, apos, avel, arx, ary, arz, gr, gp, gval

    arx = str(-int(arx)) # reverse sense of armx

    # combine states from various controllers to create status message
    rospy.loginfo("ROS robot state got a bang !")

    # swap arm y / z
    msg = "%s %s %s %s %s %s %s %s %s %s %s %s" % (xpos, ypos, apos, xvel, yvel, avel, arx, arz, ary, gr, gp, gval)
    return msg



def bang_msg(data):
    # publish combined status
    msg = combine_status()
    status.publish(msg)
    rospy.loginfo("ROS robot state published status: %s" % msg)
    return EmptyResponse()


if __name__=="__main__":
    global status

    rospy.init_node("robotstate")
    rospy.loginfo("ROS robot state starting")

    xpos = ypos = apos = xvel = yvel = avel = arx = ary = arz = gr = gp = gval = 0.


    # subscribe to output from arm
    rospy.Subscriber("/armcontrol/state", String, armstate_callback)

    # subscribe to wheel odometry
    rospy.Subscriber("/wheelstate", String, wheelstate_callback)


    # combined state
    status = rospy.Publisher("/robotstatus/output", String, queue_size=10)

    # add the 'bang' service
    rospy.Service("/robotstatus/bang", Empty, bang_msg)

    rospy.spin()
    
    
