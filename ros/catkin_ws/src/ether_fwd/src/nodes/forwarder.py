#!/usr/bin/env python

# (C) CESAR 2014 
# BSD license

import socket, select, time
import rospy
import errno

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from std_srvs.srv import *

from socket import error as socket_error

HOST_R = '' # Use all interfaces
PORT_R = 49001

HOST_W = '192.168.1.64'
#HOST_W = '127.0.0.1'
#HOST_W = '172.27.69.193'
PORT_W = 49002



#list of messageTypes
# REMOTE -> LOCAL
SPEAK = 0x4100
BASE_MOVE = 0x4101
ARM_MOVE = 0x4102
#GRIPPER_MOVE = 0x4103

STATUS_REQUEST = 0x4601

FOLLOWME = 0x4E01
OBJREC = 0x4E03
NECKHMOVE = 0x4E05

# LOCAL -> REMOTE
RSERVICE = 0x8001

STATUS_RETURN = 0x8601




def init_sockets():
    global sockr, sockw
    # open a reader socket on UDP port PORT_R
    sockr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockr.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # ignore TIME_WAIT on socket
    sockr.setblocking(0) # set non-blocking
    sockr.bind((HOST_R,PORT_R))

    # open a writer socket on UDP port PORT_W
    sockw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockw.connect((HOST_W, PORT_W))


def read_socket():
    global sockr
    ready = select.select([sockr], [], [], 0)
    if (ready[0]):
        # data is ready, read it; append it to xval, which is whatever remained from last time
        return xval + sockr.recv(512)
    return xval


def write_socket(msg, msglen):
    global sockw

    try:
        sockw.sendall(msg)
    except socket_error as serr:
        if serr.errno == errno.ECONNREFUSED:
            rospy.loginfo ("Conn error")


def close_sockets():
    global sockr, sockw
    sockr.close
    sockw.close


def checksum(val):
    return (255 - val) % 256


def addstring(string):
    tot = 0
    strlen = len(string)
    for i in xrange(0, strlen):
        tot += ord(string[i])
    return tot


def socketbuff(val):
    global sbuff
    sbuff = sbuff + chr(val)


def socketbuff_string(string):
    global sbuff
    sbuff = sbuff + string + '\x00'


def socketsend(data):
    write_socket(data, len(data))
    

def getfloat(data, n):
    # deserialize a float
    return struct.unpack('<f', data[n:n+4])[0]

def getint(data, n):
    # deserialize a float
    return struct.unpack('<i', data[n:n+4])[0]


def packfloat(fldata):
    #serialize a float
    return struct.pack('<f',fldata)


def packint(indata):
    #serialize an int
    return struct.pack('<i',indata)



def parse_data(val):
    # parse message data
    # here we will use the same format as for rosserial messages
    #
    # see: http://wiki.ros.org/rosserial/Overview/Protocol

    global followmePub, objrecPub, xval, getstatus, armconPub, cmdvelPub, speechPub

    while len(val) >= 8:

        rospy.loginfo("got data2")

        s = list(val) # convert val (string) to s (array of char)

        if (ord(s[0]) != 0xFF): 
            xval = val[1:]
            rospy.loginfo("got FF mismatch %d" % ord(s[0]))
            return # sync start

        if (ord(s[1]) != 0xFD): 
            xval = val[1:]
            rospy.loginfo("got FD mismatch %d" % ord(s[1]))
            return # protocol
    
        dlen = ord(s[2]) + ord(s[3]) * 256 # data length

        cksum = checksum(ord(s[2]) + ord(s[3]))
        if (cksum != ord(s[4])):
            xval = val[2:]
            rospy.loginfo("got hdr cksum mismatch %d and %d " % (cksum,ord(s[4])))
            return # datalength checksum

        # the message type
        mtype = ord(s[5]) + ord(s[6]) * 256

        rospy.loginfo("got msg type %0x" % mtype)
    
        if (mtype == FOLLOWME):
            xlen = 1 # data length in bytes
            topic = followmePub # topic to republish on

	elif (mtype == NECKHMOVE):
            xlen = 1 # data length in bytes
            topic = neckhmovePub # topic to republish on

        elif (mtype == OBJREC):
            xlen = dlen
            topic = objrecPub # topic to republish on

        elif (mtype == ARM_MOVE):
            xlen = dlen
            topic = armconPub # topic to republish on

        elif (mtype == BASE_MOVE):
            xlen = dlen
            topic = cmdvelPub # topic to republish on

        elif (mtype == SPEAK):
            xlen = dlen
            topic = speechPub


        elif (mtype == STATUS_SEND):
            xlen = 0

        else:
            topic = ""

        msglen = xlen + 8
        xval = val[msglen:]


        # len val is 36, msglen is 44
        #rospy.loginfo("cf %s and %s" % (len(val), msglen))

        if (len(val) < msglen):
            xval = val
            return # invalid message length (try to read more)

        if (topic != ""):
            if (dlen != xlen): return # invalid data length

        tot = ord(s[5]) + ord(s[6])
        for i in xrange(7, 7 + dlen):
            tot += ord(s[i])

        cksum = checksum(tot)
        if (cksum != ord(s[7 + dlen])): 
            rospy.loginfo("got final cksum mismatch %d and %d " % (cksum,ord(s[7+dlen])))
            return # msgtype + data checksum

        # package data for sending depending on message type
        if (mtype == FOLLOWME):
            data = str(ord(s[7]))
            #print 'msgtype is FOLLOWME: sending ',data

        elif mtype == SPEAK:
            if ord(s[len(s) - 1]) == 0:
                data = s[7:-1]
            else:
                data = s[7:]

	elif (mtype == NECKHMOVE):
            data = str(ord(s[7]))
            #print 'msgtype is NECKHMOVE: sending ',data

        elif (mtype == OBJREC):
            data = ""
            for i in xrange(0, dlen/4):
                val = str(ord(s[7+i*4]) + (ord(s[8+i*4])<<8) + (ord(s[9+i*4])<<16) + (ord(s[10+i*4])<<24))
                if (data == ""):
                    data = str(val)
                else:
                    data = data + " " + str(val)
            #print 'msgtype is OBJREC: sending ',data

        elif (mtype == BASE_MOVE):
            # read 3 floats, xvel, yvel, angvel

            xvel = getfloat(val,7)
            yvel = getfloat(val,11)
            angvel = getfloat(val,15)

            # create a twist message (xvel, yvel, 0), (0, 0, angvel)

            data = Twist()

            data.linear.x = xvel
            data.linear.y = yvel
            data.linear.z = 0.
            data.angular.x = 0.
            data.angular.y = 0.
            data.angular.z = angvel
            

        elif (mtype == ARM_MOVE):
            # read 6 floats: x, y, z, gr, gp, grip and int delta

            # note x sense is swapped
            x = - getfloat(val,7)

            # note y and z are swapped !
            z = getfloat(val,11)
            y = getfloat(val,15)

            gr = getfloat(val,19)
            gp = getfloat(val,23)
            grip = getfloat(val,27)

            delta = getint(val,31)

            # note we swap posns of gp and gr
            data = "%f %f %f %f %f %f %d" % (x, y, z, gp, gr, grip, delta)

            rospy.loginfo("armcontrol: %s" % data)


        elif (mtype == STATUS_REQUEST):
            # send service request to robot_state
            try:
                getstatus
            except rospy.ServiceException as exc:
                rospy.loginfo ("Service call failed: %s" % str(exc))



        if (topic != ""):
            #publish data on correct topic
            topic.publish(data)
    
        # if any more data in packet, process it
        val = xval



def send_header(msglen, topicID):
    socketbuff(0xFF)
    socketbuff(0xFD)
    socketbuff(msglen % 256)
    socketbuff(msglen >> 8)
    
    cksum = checksum(msglen)
    socketbuff(cksum)

    socketbuff(topicID & 0xFF)
    socketbuff(topicID >> 8)




def rserv_cb(data):
    global sbuff

    sbuff = ""
    
    df = data.data.split('/')

    if (df[0] == "+"):
        if (len(df) < 3):
            return

        msglen = 3 + len(df[1]) + len(df[2])

        send_header(msglen, RSERVICE) # send sync, protocol #, msglen, msglen checksum,  and topic ID

        # send data packet
        socketbuff(0x01) # startup
        socketbuff_string(df[1]) # send NULL terminated string
        socketbuff_string(df[2]) # send NULL terminated string

        # send data / topic ID checksum
        x = (RSERVICE & 0xFF) + (RSERVICE >> 8) + 0x01
        x += addstring(df[1])
        x += addstring(df[2])

        cksum = checksum(x)
        socketbuff(cksum)


    elif (df[0] == "-"):
        if (len(df) < 2):
            return

        if (len(df) == 3):
            msglen = 3 + len(df[1]) + len(df[2])
        else:
            msglen = 2 + len(df[1])

        send_header(msglen, RSERVICE) # send sync, protocol #, msglen, msglen checksum,  and topic ID

        # send data packet
        socketbuff(0x00) # shutdown
        socketbuff_string(df[1]) # send NULL terminated string
        if (len(df) == 3):
            socketbuff_string(df[2]) # send NULL terminated string

        # send data / topic ID checksum
        x = (RSERVICE & 0xFF) + (RSERVICE >> 8)
        x += addstring(df[1])
        if (len(df) == 3):
            x += addstring(df[2])

        cksum = checksum(x)
        socketbuff(cksum)

    socketsend(sbuff)

    rospy.loginfo("Msg of length "+str(msglen)+" bytes sent.")



def fmt_robot_state(stri):
    status = stri.split(' ')

    posx = packfloat(status[0])
    posy = packfloat(status[1])
    posang = packfloat(status[2])

    velx = packfloat(status[3])
    vely = packfloat(status[4])
    velang = packfloat(status[5])

    armx = packfloat(status[6])
    army = packfloat(status[7])
    armz = packfloat(status[8])

    groll = packfloat(status[9])
    gpitch = packfloat(status[10])

    gopen = packfloat(status[12])

    now = rospy.get_rostime()

    tsec = packint(now.secs)
    tnsec = packint(now.nsecs)

    ret = "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s" % (tsec, tnsec, posx, posy, posang, velx, vely, velang, armx, army, armz, groll, gpitch, gopen)

    return ret



def robot_state_cb(data):
    msg = fmt_robot_state(data.data)

    # forward status to VOXAR
    send_status(STATUS_RETURN, msg)



def send_status(proto, df):

    msglen = 3 + len(df)

    send_header(msglen, proto) # send sync, protocol #, msglen, msglen checksum,  and topic ID

    # send data packet
    socketbuff_string(df) # send NULL terminated string

    # send data / topic ID checksum
    x = (proto & 0xFF) + (prot >> 8)
    x += addstring(df)

    cksum = checksum(x)
    socketbuff(cksum)

    socketsend(sbuff)




def forwarder():
    global followmePub, objrecPub, neckhmovePub, xval, getstatus, armconPub, cmdvelPub, speechPub

    rospy.init_node('forwarder', anonymous=True)

    rospy.loginfo("eth_fwd STARTING...")

    print "OK\n"

    # init all topics to republish on here
    followmePub = rospy.Publisher('/followme', String, queue_size=10)

    # init all topics to republish on here
    neckhmovePub = rospy.Publisher('/eth_neckhmove', String, queue_size=1)

    # init all topics to republish on here
    objrecPub = rospy.Publisher('/objrec', String, queue_size=1)

    # init all topics to republish on here
    armconPub = rospy.Publisher('/armcontrol/target', String, queue_size=1)

    # init all topics to republish on here
    cmdvelPub = rospy.Publisher('/cmd_vel', String, queue_size=1)

    # init all topics to republish on here
    speechPub = rospy.Publisher('/speech_request', String, queue_size=1)

    # init all topics to listen to here

    rospy.Subscriber('/rservice', String, rserv_cb)
    rospy.Subscriber('/robotstatus/output', String, robot_state_cb)

#    rospy.wait_for_service('/robotstatus/bang')

    try:
        getstatus = rospy.ServiceProxy('/robotstatus/bang', Empty)
    except rospy.ServiceException as exc:
        rospy.loginfo( "Service call failed: %s" % str(exc))

    init_sockets()

    xval = ""

    while not rospy.is_shutdown():
        # read bytes from ether
        str = read_socket()
        if (str != ""): parse_data(str)
        else: time.sleep(0.01)

    close_sockets()



if __name__ == '__main__':
    try:
        forwarder()
    except rospy.ROSInterruptException: pass



