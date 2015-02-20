#!/usr/bin/env python

# (C) CESAR 2014 - 2015
# BSD license

# original authors gf@cesar.org.br and fala@cesar.org.br

import roslib
roslib.load_manifest('cesar_robot_arm')

import rospy
import serial
import struct
import time
import math
from enum import Enum

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from cesar_robot_arm.srv import *

#TEST = 1
TEST = 0
CLAUSTRUM = 1

class ARM_MODES(Enum):
    BACKHOE = 0
    CARTESIAN0 = 1

ARM_MODE = ARM_MODES.BACKHOE

UP_OFFS = 0.05 #0.14 # distance arm base -> base joint
FWD_OFFS = 0.03


ANG1_OFFS = 0.4 # compensate for off-centre joint

BASE_CORRECTION = 0.19 # correction to add to base to allow for off center arm

PI = 3.1415926
TWO_PI = PI * 2

L1 = 0.151 
L2 = 0.144 
L3 = 0.160 
    
#    L1 = 0.151 # base2 -> shoulder
#    L2 = 0.144 # shoulder -> elbow
#    L3 = 0.160 # elbow -> gripper point


def packshort(indata):
    #serialize a short BE
    return struct.pack('>h',indata)


def calc_checksum(msg):
    tot = 0

    for i in xrange(1, 16):
        tot += ord(msg[i])

    tot = tot % 256

    csum = 255 - tot

    return chr(csum)



def prep_msg(x, y, z, gp, gr, gripval, delta):

    # after DELTA is button and extended byte

    if ARM_MODE == ARM_MODES.CARTESIAN0:
        ROTRAT = 310. / 90.
        msg = '\xFF' + packshort(x + 512) + packshort(y) + packshort(z) + packshort(gp + 90.) + packshort(gr * ROTRAT + 512) + packshort(gripval) + chr(delta) + '\x00' + '\x00'

    elif ARM_MODE == ARM_MODES.BACKHOE:
        msg = '\xFF' + packshort(x) + packshort(y) + packshort(z) + packshort(gp) + packshort(gr) + packshort(gripval) + chr(delta) + '\x00' + '\x00'


    msg = msg + calc_checksum(msg)

    rospy.loginfo("msg is %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x" % (ord(msg[0]),ord(msg[1]),ord(msg[2]),ord(msg[3])
                                          ,ord(msg[4])
                                          ,ord(msg[5])
                                          ,ord(msg[6])
                                          ,ord(msg[7])
                                          ,ord(msg[8])
                                          ,ord(msg[9])
                                          ,ord(msg[10])
                                          ,ord(msg[11])
                                          ,ord(msg[12])
                                          ,ord(msg[13])
                                          ,ord(msg[14])
                                          ,ord(msg[15])
                                          ,ord(msg[16])
                                      ))


    return msg



def calc_joint_vals(theta0, theta1, theta2, theta3, tgr, tgripval):
    jvals = [0, 0, 0, 0, 0, 0] 

    jvals[0] = int((theta0 + BASE_CORRECTION) / PI * 2048. + 2048.)
    if (jvals[0] < 0): jvals[0] = 0
    if (jvals[0] > 4095): jvals[0] = 4095

    jvals[1] = int(theta1 / PI * 2048. + 2048.)
    if (jvals[1] < 1024): jvals[1] = 1024
    if (jvals[1] > 3072): jvals[1] = 3072

    jvals[2] = int(theta2 / PI * 2048. + 2048.)
    if (jvals[2] < 1024): jvals[2] = 1024
    if (jvals[2] > 3072): jvals[2] = 3072

    jvals[3] = int(theta3 / PI * 2048. + 2048.)
    if (jvals[3] < 1024): jvals[3] = 1024
    if (jvals[3] > 3072): jvals[3] = 3072

    jvals[4] = int(tgr / PI * 512. + 512.)
    if (jvals[4] < 0): jvals[4] = 0
    if (jvals[4] > 1023): jvals[4] = 1023

    jvals[5] = int(tgripval)
    if (jvals[5] < 0): jvals[4] = 0
    if (jvals[5] > 512): jvals[4] = 512

    rospy.loginfo("Sending jvals %d %d %d %d %d %d" % (jvals[0], jvals[1], jvals[2], jvals[3], jvals[4], jvals[5]))

    return jvals


def control_callback(data):
    real_control_callback(data.data)

def service_control_callback(data):
    return real_control_callback(data.query)

def real_control_callback(strdata):
    global tx, ty, tz, tgp

    type = 0 # move

    rospy.loginfo("got arm ctrl msg")

    delta = -100

    vals = strdata.split(" ")
    if (len(vals) < 5): return

    if CLAUSTRUM:
        # in this mode, first value is 0 = move, 1 = test
        type = int(vals[0])

        if type == 2:
            # just check if we are moving
            return send_plausible(True)

        vals = vals[1:]

    tx = float(vals[0])

    if ARM_MODE != ARM_MODES.BACKHOE:
        if (tx < -.3): tx = -.3
        if (tx > .3): tx = .3

    ty = float(vals[1])

    if ARM_MODE != ARM_MODES.BACKHOE:
        if (ty < .05): ty = .05
        if (ty > .4): ty = .4

    tz = float(vals[2]) - UP_OFFS

    if ARM_MODE != ARM_MODES.BACKHOE:
        if (tz < .020): tz = .020
        if (tz > .350): tz = .350


    tgp = float(vals[3])

    if ARM_MODE != ARM_MODES.BACKHOE:
        tgp = int(tgp / TWO_PI * 360.)

        if (tgp < -30): tgp = -30
        if (tgp > 30): tgp = 30


    if type == 0:

        if ARM_MODE != ARM_MODES.BACKHOE:
            tgr = int(float(vals[4]) / TWO_PI * 360.)

            if (tgr < -145): tgr = 145
            if (tgr > 145): tgr =145
        else:
            tgr = float(vals[4]) * 180. / 150.


        tgripval = int(float(vals[5]) * 512.)
        rospy.loginfo("atarget is %f %f %f %f\n" % (tx,ty,tz,tgp))

        msg = ""
# optional

    if (len(vals) > 6):
        delta = int(vals[6])

    if (delta == -1):
        # pause
        msg = '\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF\x00\xA0'
        msg = msg + calc_checksum(msg)
        rospy.loginfo("send pause arm ctrl msg")

    elif (delta == -2):
        #resume
        msg = '\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF\x00\xA1'
        msg = msg + calc_checksum(msg)
        rospy.loginfo("send resume arm ctrl msg")

    else:

        if (delta < 0 or delta > 255): delta = 255

        if (ARM_MODE == ARM_MODES.BACKHOE):
            rospy.loginfo("target is %f %f %f %f\n" % (tx,ty,tz,tgp))

#            theta = inverse_kinematics(ty, tx, tz, tgp, tgr) # catch error if impossible
#            confirm_DK(theta[0], theta[1], theta[2], theta[3])
#            msg = prep_msg(theta[0], theta[1], theta[2], theta[3], tgr, tgripval, delta)

            res = IK_alt(tx, ty, tz, tgp)
            if res:
                if confirm_DK(theta0, theta1, theta2, theta3, True):

                    if type == 1:
                        return send_plausible(res)

                    joint_vals = calc_joint_vals(theta0, theta1, theta2, theta3, tgr, tgripval)
                    msg = prep_msg(joint_vals[0], joint_vals[1], joint_vals[2], joint_vals[3], joint_vals[4], joint_vals[5], delta)
            else:
                if type == 1:
                    return send_plausible(res)

        elif (ARM_MODE == ARM_MODES.CARTESIAN0):
            msg = prep_msg(tx, ty, tz, tgp, tgr, tgripval, delta)


    if type == 0:
        rospy.loginfo("send arm ctrl msg")

        if not TEST:
            if msg != "": ser.write(msg)



def arm_state_to_string():
    global count, x, y, z, gr, gp, gripval
    data = "%f %f %f %f %f %f %d" % (x, y, z, gr, gp, gripval, count)
    count = count + 1
    return data

# j0 = Joint base_arm_to_base2
# j1 = Joint base_to_shoulder
# j2 = Joint shoulder_to_elbow
# j3 = Joint elbow_to_wrist
# j4 = Joint wrist_to_grip_base
# j5 = Joint grip_base_to_right_finger        [axis = +y]


def send_plausible(res):
    global mov, paused, possible
    # we ran a check to see if the position was achievable, now we send the result

    # send 3 values
    # a) 0 = achievable, 1 = impossible
    # b) 1 = in movement, 0 = finished movement
    # c) 0 = not paused, 1 = paused

    if res:
        imp = 0
    else:
        imp = 1

    msg = "%d %d %d" % (imp, mov, paused)

    return msg



def confirm_DK(theta0, theta1, theta2, theta3, check):
    global x, y, z, tx, ty, tz, tgp

    distthresh = .000001
    gpthresh = 0.1

    up = 0
    fwd = 0

    ang = theta1 + ANG1_OFFS

    up += L1 * math.cos(ang)
    fwd += L1 * math.sin(ang)

    rospy.loginfo("p1 is %f %f up %f fwd" % (ang,up,fwd))

    ang += PI / 2. - theta2 - ANG1_OFFS

    up += L2 * math.cos(ang)
    fwd += L2 * math.sin(ang)

    rospy.loginfo("p2 is %f %f up %f fwd" % (ang,up,fwd))

    u0 = up

    ang -= theta3

    up += L3 * math.cos(ang)
    fwd += L3 * math.sin(ang)

    rospy.loginfo("p3 is %f %f up %f fwd" % (ang,up,fwd))

    z = up # + UP_OFFS
    fwd -= FWD_OFFS
    y = fwd * math.cos(-theta0)
    x = fwd * math.sin(-theta0)

    rospy.loginfo("DK position confirming --->  x: %f , y: %f , z: %f" % (x, y, z) )

    hor_angle = math.asin((up - u0)/L3)

    rospy.loginfo("Gripper pitch confirming ->  gp: %f " % hor_angle )
    
    if check:

        dist = (z - tz) * (z - tz) + (y - ty) * (y - ty) + (x - tx) * (x - tx)
        if dist > distthresh:
            rospy.loginfo("UNATTAINABLE POSITION")
            return False

        if math.fabs(hor_angle - tgp) > gpthresh:
            rospy.loginfo("UNATTAINABLE GRIPPER POSITION")
            return False

        return True


def IK_alt(x, y, p3up, theta4):
    global theta0, theta1, theta2, theta3

    p3fwd = math.sqrt(x * x + y * y) + FWD_OFFS
    theta0 = -math.atan(x/y)

    steps = 10000
    p0min = -PI/2. + ANG1_OFFS
    p0max = PI/2. + ANG1_OFFS

    # position of elbow
    p2up = p3up - L3 * math.sin(theta4)
    p2fwd = p3fwd - L3 * math.cos(theta4)

    if (p2fwd < 0): p2fwd = 0

    distar = L2 * L2
    best = -1000.
    minv = .0001

    rangv = (p0max - p0min) / steps
    ang = p0min - rangv

    for i in xrange(0, steps):
        ang += rangv

        p1up = L1 * math.cos(ang)
        p1fwd = L1 * math.sin(ang)

        dist = (p1up - p2up) * (p1up - p2up) + (p1fwd - p2fwd) * (p1fwd - p2fwd)

        diff = math.fabs(distar - dist)

        if diff <= minv:
            theta1 = ang - ANG1_OFFS

            p1up = L1 * math.cos(ang)
            p1fwd = L1 * math.sin(ang)
            try:
                horz = math.asin((p2up - p1up)/L2)
            except ValueError:
                continue

            theta2 = horz + theta1
            if theta2 < - PI/2. or theta2 > PI/2.:
                continue

            theta3 = theta4 - theta2 + theta1
            if theta3 < - PI/2. or theta3 > PI/2.:
                continue

            minv = diff
            best = ang


    if (best == -1000.):
        rospy.loginfo("IMPOSSIBLE REQUEST")
        return False

    theta1 = best - ANG1_OFFS

    p1up = L1 * math.cos(best)
    p1fwd = L1 * math.sin(best)

    try:
        horz = math.asin((p2up - p1up)/L2)
    except ValueError:
        print ("IMPOSSIBLE\n")
        return False

    theta2 = horz + theta1

    theta3 = theta4 - theta2 + theta1

    #print("vals %f : %f %f  -  %f : %f %f  %f\n" % (theta0, p1up, p1fwd, theta1, p2up, p2fwd, theta2))

    return confirm_DK(theta0, theta1, theta2, theta3, True)



def inverse_kinematics(x, y, z, gp, gr):

    theta = [0, 0, 0, 0, 0] 
    # theta eh o vetor de juntas de DH. No fim do codigo ele eh convertido para que o centro seja 0.

    theta[0] = math.atan2(y, x) + PI  + ANG1_OFFS

    C1 = math.cos(theta[0])
    S1 = math.sin(theta[0])

    Rx = (x + L3*C1*math.cos(gp))/C1
    Ry = (y + L3*S1*math.cos(gp))/S1
    # Rx = Ry (distancia horizontal da base ao joint 4)
    Rz = (z - L3*math.sin(gp) - UP_OFFS)

    C3 = ( pow(Ry, 2) + pow(Rz, 2) - pow(L2, 2) - pow(L1, 2) ) / (2*L2*L1)
    theta[2] = math.acos(C3)
    S3 = math.sin(theta[2])

    C2 = ( L2*(Ry*C3 + Rz*S3) + L1*Ry ) / ( pow(Ry, 2) + pow(Rz, 2) )
    theta[1] = math.acos(C2)
    S3 = math.sin(theta[1])
 
    theta[3] = - gp - theta[1] - theta[2] + 270*PI/180

    theta[4] = gr

    # Conversion for center in 0 for backhoe position
    # Theta returns in the same format convention that we see in rviz (use gui:=True)

    theta[0] =  theta[0] - PI
    theta[1] =  theta[1] - PI/2 - 0.35954
    theta[2] = -theta[2] + 1.3858
    theta[3] = -theta[3] + PI/2
 
    return (theta)



def armcontrol_send_receive():
    global jointstates, gr, gp, gripval, ready, mov, paused

    msg = '\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF\x00\x80'
    msg = msg + calc_checksum(msg)

    ser.write(msg)

    # read 0xFF followed by 12 bytes (h/l of servos) followed by 3 bytes (interpol, paused, csum)
    #16 bytes in total

    msg = ser.read(16)

    if ord(msg[0]) != 0xFF:
        ser.read(19)
        msg = ser.read(16)

#    rospy.loginfo("pos is %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x" % (ord(msg[0]),ord(msg[1]),ord(msg[2]),ord(msg[3])
#                                          ,ord(msg[4])
#                                          ,ord(msg[5])
#                                          ,ord(msg[6])
#                                          ,ord(msg[7])
#                                          ,ord(msg[8])
#                                          ,ord(msg[9])
#                                          ,ord(msg[10])
#                                          ,ord(msg[11])
#                                          ,ord(msg[12])
#                                          ,ord(msg[13])
#                                          ,ord(msg[14])
#                                          ,ord(msg[15])
#                                      ))

        
    # retransmit on /joint_states

    jsm = JointState()

    jsm.name.append("base_arm_to_base2")
    jsm.name.append("base_to_shoulder")
    jsm.name.append("shoulder_to_elbow")
    jsm.name.append("elbow_to_wrist")
    jsm.name.append("wrist_to_grip_base")
    jsm.name.append("grip_base_to_right_finger")
   
    ba = float(ord(msg[1]) * 256 + ord(msg[2])) - BASE_CORRECTION

    sh = float(ord(msg[3]) * 256 + ord(msg[4]))
    el = float(ord(msg[5]) * 256 + ord(msg[6]))

    gp = float(ord(msg[7]) * 256 + ord(msg[8]))

    gr = float(ord(msg[9]) * 256 + ord(msg[10]))
    gval = float(ord(msg[11]) * 256 + ord(msg[12]))
    
    mov = ord(msg[13])
    paused = ord(msg[14])

    # TODO - check csum

#    rospy.loginfo("original vals %f %f %f %f" % (ba,sh,el,gp))

    VAL1 = 16384.

    nba = (ba / VAL1 - 1.0) * PI     # Range: [0, 16384, 32767] -> [-PI, 0, PI] 
    nsh = (sh / VAL1 - 1.0) * PI # Range: [0, 16384, 32767] -> [-PI, 0, PI] 
    nel = (el / VAL1 - 1.0) * PI # Range: [0, 16384, 32767] -> [-PI, 0, PI] 
    ngp = (gp / VAL1 - 1.0) * PI * 150./180. # value in radians

    jsm.position.append(nba)
    jsm.position.append(nsh)
    jsm.position.append(nel)
    jsm.position.append(ngp)

#    rospy.loginfo("calc vals %f %f %f %f" % (nba,nsh,nel,ngp))

    gr = (gr / 4096. - 1.0) * PI * 150./180. # value in radians
    gripval = (gval / 4096.)    # 1.0 = open, 0.0 = closed

    jsm.position.append(gr)
    jsm.position.append(gripval)

#    rospy.loginfo("Current actual positions received: %f , %f , %f , %f , %f, %f" \
#    % (jsm.position[0], jsm.position[1], jsm.position[2], ngp, gr, gripval))

#    rospy.loginfo("extra vals %f %f %f" % (gr, gp, gripval))

    now = rospy.get_rostime()

    jsm.header.stamp.secs = now.secs
    jsm.header.stamp.nsecs = now.nsecs

    jointstates.publish(jsm)

    confirm_DK(nba, nsh, nel, ngp, False)



if __name__=="__main__":
    global ser, count, state, jointstates, mov, paused

    rospy.init_node("armcontrol")
    rospy.loginfo("ROS arm control")

    count = 0

    mov = 0
    paused = 0

    port_name = rospy.get_param('~arm_port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~arm_baud','38400'))

    rospy.loginfo("Connecting to %s at %d baud" % (port_name,baud) )

    if not TEST:
        ser = serial.Serial(port_name, baud)
        time.sleep(10)

    rospy.loginfo("Arm ready, moving to home position")

    if ARM_MODE == ARM_MODES.CARTESIAN0:
        msg = '\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF\x00\x20'
    elif ARM_MODE == ARM_MODES.BACKHOE:
        msg = '\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xFF\x00\x40'

    msg = msg + calc_checksum(msg)
 
    if not TEST:
        ser.write(msg)
        ser.read(5)

    rospy.Subscriber("/armcontrol/input", String, control_callback)

    # publisher for VOXAR (temporary)
    state = rospy.Publisher('/armcontrol/state', String, queue_size=10)

    # real publisher (retransmit from arm controller)
    jointstates = rospy.Publisher('/joint_states', JointState, queue_size=10)

    r = rospy.Rate(10) # 10hz

    rospy.Service('armcontrol_check', check, service_control_callback)

    while not rospy.is_shutdown():
        if not TEST:
            armcontrol_send_receive()
            armstate = arm_state_to_string()
            state.publish(armstate)
        r.sleep()

    if not TEST:
        ser.close()

