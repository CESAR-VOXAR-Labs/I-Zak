#!/usr/bin/env python


# claustrum.py : robot controller node
# gf@cesar.org.br

# 2014 original version


# debug data with:
# rostopic echo /claustrum/output

import conversa

from enum import Enum
import time
import signal
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from keyboard.msg import Key

import math
PI = 3.1415926


import cesar_robot_arm.srv

PROMISCUOUS = 1
#PROMISCUOUS = 0

MAX_CONFIRM_ATTEMPTS = 3

OBJREC_MIN_SECS = 5

FACEREC = 0
FACEREC_MIN_SECS = 10

if FACEREC:
    IDLE_FACEREC = 1
    from face_recognition.msg import *
else:
    IDLE_FACEREC = 0


MOVEABLE_CAMERA = 0

# set FAKE = 1 if we want fake toothpaste
FAKE = 1

#if MOVEABLE_CAMERA:
#import tf



class SPEECH_REC_STATE(Enum):
    OFF = 0
    WAIT = 1
    READY = 2
    CONFIRM = 3

class ACTION(Enum):
    NONE = 0
    TRACK_BALL = 1
    FOLLOW_ME = 2
    OBJREC = 3
    IDLE = 4
    TOOTHPASTE = 5
    REMEMBER_FACE = 6

objdb = {
    65525: 'biz', \
    65526: 'waffles', \
    65527: 'toast', \
    65528: 'chocolate', \
    65529: 'coconut water', \
    65530: 'cracker', \
    65531: 'shokeetoo', \
    65532: 'potato crisps', \
    65533: 'fanta', \
    65534: 'juice', \
    65535: 'milk', \
    65536: 'coke', \
    65537: 'gwahrahnah', \
    65538: 'beer', \
    65539: 'water', \
    65540: 'chips'
    #etc.
}


TASK_STOP = "stop"
TASK_FOLLOW_ME = "follow you"
TASK_IDLE = "look for you"
TASK_TRACK_BLUE = "track the blue ball"
TASK_TRACK_RED = "track the red ball"
TASK_TRACK_GREEN = "track the green ball"
TASK_TRACK_YELLOW = "track the yellow ball"
TASK_OBJREC = "tell you what I see"
TASK_FETCH_TOOTHPASTE = "fetch the toothpaste"
TASK_REMEMBER_FACE = "remember your face"



def say(data):
    global talk

    talk.publish(data)


def mprint(data):
    global xprint
    xprint.publish(data)
    rospy.loginfo(rospy.get_caller_id()+" "+data)
    


def send_target_ball_color(red, green, blue, thresh):
	# do nothing
	mprint("")


def rservice_startup(srv, subsrv):
    global rservice
    rservice.publish("+/" + srv + "/" + subsrv)



def rservice_shutdown(srv, subsrv):
    global rservice
    if (subsrv != ""):
        rservice.publish("-/" + srv + "/" + subsrv)
    else:
        rservice.publish("-/" + srv)


def do_reset():
    global Action, wheels, neck, last_objrec_time, track_msgcnt

    # stop image processing
    send_target_ball_color(0, 0, 0, 6000)

    # stop wheel movement
    twist = Twist()
    wheels.publish(twist)

    # center neck
    neck.publish(64)

    if (Action == ACTION.FOLLOW_ME):
        rservice_shutdown("followme","")
    
    if (Action == ACTION.IDLE):
        rservice_shutdown("neckhmove","")

    elif (Action == ACTION.OBJREC):
        rservice_shutdown("objrec","")

    elif (Action == ACTION.TOOTHPASTE):
        rservice_shutdown("objrec","centroid")

    last_objrec_time = 0
    track_msgcnt = 0


def do_easter_egg():
    say("I am the robot called poppy. I became operational at CESAR laboratories, Receefay, Brazil on 14th of November 2014. \
    I am very pleased to meet you.")


def do_shutdown():
    do_reset()
    rservice_shutdown("*","")



def signal_handler(signal, frame):
    mprint("shutting down\n")
    rospy.signal_shutdown("Ctrl-c")
    

def set_action(act):
    global Action, rservice, kign, spcb, kdcb, facerec

    if (Action == ACTION.FOLLOW_ME or Action == ACTION.TRACK_BALL or Action == ACTION.OBJREC or Action == ACTION.IDLE \
        or Action == ACTION.TOOTHPASTE):
        do_reset()

    Action = act

    if (Action == ACTION.FOLLOW_ME):
        rservice_startup("followme", "followme")

    if (Action == ACTION.IDLE):
        rservice_startup("neckhmove", "neckhmove")

        if IDLE_FACEREC == 1:
            goal = FRClientGoal()
            goal.order_argument = "none"
            goal.order_id = 1

            facerec.publish(goal)

    elif (Action == ACTION.OBJREC):
        rservice_startup("objrec", "objrec")

    elif (Action == ACTION.TOOTHPASTE):
        rservice_startup("objrec", "centroid")

        if FAKE:
            time.sleep(10)
            say ("Toothpaste located.")
            perform_arm(0., 0.32, 0.05, 0.)

            
    elif Action == ACTION.REMEMBER_FACE:
        kdcb.unregister()
        say ("First I need to know your name. Please enter it using my keyboard.")
        complete = 0
        allow_timeout = 1

        while complete == 0:
            person_name = ""
            done = 0

            while done == 0:
                # read keys until enter is pressed
                try:
                    data = rospy.wait_for_message('/keyboard/keydown', Key, 5)
                except rospy.ROSException:
                    if allow_timeout: allow_timeout = 0
                    elif person_name != "": done = 1
                    continue
                if done == 0:
                    key = data.code
                    if key == 13:
                        done = 1
                    else:
                        person_name += chr(key)


            say("Your name is %s. Is that correct ?" % person_name)

            # get yes or no response
            done = 0
            spcb.unregister()

            while done == 0:
                try:
                    data = rospy.wait_for_message("/recognizer/output", String, 10)
                except rospy.ROSException:
                    done = 0
                    continue
                if done == 0:
                    said = data.data
                    #say ("I heard %s" % said)

                    if said == 'yes':
                        done = 1
                        complete = 1

                    if said == 'no':
                        say ('Please enter your name again.')
                        break

                
        spcb = rospy.Subscriber("/recognizer/output", String, speech_callback)

        say ("Thank you %s. I will now remember your face. Please move in front of my camera." % person_name)

        done = 0

        goal = FRClientGoal()
        goal.order_argument = person_name
        goal.order_id = 2

        facerec.publish(goal)

        while done == 0:
            try:
                data = rospy.wait_for_message("/face_recognition/result", FaceRecognitionActionResult, 10)
            except rospy.ROSException:
                    done = 0
                    continue
            if data.result.order_id == 2 and data.result.names[0] == person_name:
                    done = 1

        #need a short delay after grabbing before training
        time.sleep(1)
        
        #train with person data
        goal.order_argument = person_name
        goal.order_id = 3

        facerec.publish(goal)

        while done == 0:
            try:
                data = rospy.wait_for_message("/face_recognition/result", FaceRecognitionActionResult, 10)
            except rospy.ROSException:
                    done = 0
                    continue
            if data.result.order_id == 3:
                    done = 1

        say ("Thank you %s. I have now memorised your face." % person_name)
        # go back to IDLE


        # in IDLE mode, keep looking for faces
        kdcb = rospy.Subscriber("/keyboard/keydown", Key, keydown_callback)

        set_action(ACTION.IDLE)





def handle_task(data):
    if (data == TASK_TRACK_BLUE):
        set_action(ACTION.TRACK_BALL)
        send_target_ball_color(60, 80, 180, 6000)
    elif (data == TASK_TRACK_RED):
        set_action(ACTION.TRACK_BALL)
        send_target_ball_color(180, 50, 50, 4000)
    elif (data == TASK_TRACK_GREEN):
        set_action(ACTION.TRACK_BALL)
        send_target_ball_color(70, 140, 80, 4000)
    elif (data == TASK_TRACK_YELLOW):
        set_action(ACTION.TRACK_BALL)
        send_target_ball_color(180, 150, 40, 4000)
    elif (data == TASK_FOLLOW_ME):
        set_action(ACTION.FOLLOW_ME)
    elif (data == TASK_IDLE or data == TASK_STOP):
        set_action(ACTION.IDLE)
        # start following
    elif (data == TASK_OBJREC):
        set_action(ACTION.OBJREC)
    elif (data == TASK_FETCH_TOOTHPASTE):
        set_action(ACTION.TOOTHPASTE)
    elif (data == TASK_REMEMBER_FACE):
        set_action(ACTION.REMEMBER_FACE)
        # recognise objects
    #elif (data == TASK_ANSWERS):
	#set_action(ACTION.ANSWERS)
	# start answer
    else:
        set_action(ACTION.IDLE)


def is_like(needles,haystack):
    # take each needle and see if we find it in haystack
    # if not we return false

    nn = needles.split()
    hh = haystack.split()
    nhh = len(hh)
    cinh = 0

    for current_word in nn:
        while (1):
            cinh = cinh + 1
            if (cinh > nhh): return False
            #rospy.loginfo(rospy.get_caller_id()+" Check %s against %s",current_word,hh[cinh-1])
            if (current_word == hh[cinh-1]): break

    return True



def proc_speech_data(phrase):
    #rospy.loginfo(rospy.get_caller_id()+" Proc speech %s",phrase)

    if (phrase == "stop" or phrase == "the" or phrase == "no" or phrase == "ball" or phrase == "hello"):
        return TASK_STOP

    if (is_like("toothpaste",phrase)):
        return TASK_FETCH_TOOTHPASTE

    if (is_like("fetch",phrase)):
        return TASK_FETCH_TOOTHPASTE

    if (is_like("face",phrase)):
        return TASK_REMEMBER_FACE

    if (is_like("remember",phrase)):
        return TASK_REMEMBER_FACE

    if (is_like("track blue",phrase)):
        return TASK_TRACK_BLUE

    if (is_like("track red",phrase)):
        return TASK_TRACK_RED

    if (is_like("track green",phrase)):
        return TASK_TRACK_GREEN

    if (is_like("track yellow",phrase)):
        return TASK_TRACK_YELLOW

    if (is_like("follow", phrase)):
 	return TASK_FOLLOW_ME

    if (is_like("you see",phrase)):
        return TASK_OBJREC
    
    return ""



def confirm_task(task):
    global Speech_state
    say("I will now " + task + ".")
    handle_task(task)
    Speech_state = SPEECH_REC_STATE.WAIT


def speech_callback(data):
    global Speech_state, CONVERSA
    global Next_task
    global last_person_seen

    mprint(" I heard" + data.data)

    if (Speech_state == SPEECH_REC_STATE.CONFIRM):
        #rospy.loginfo(rospy.get_caller_id()+"Setting speech_rec_state to SPEECH_WAIT")
        Speech_state = SPEECH_REC_STATE.WAIT

        if (is_like("no", data.data)):
            say("Please try again.")
	    Speech_state = SPEECH_REC_STATE.READY

        elif (is_like("yes", data.data)):
            confirm_task(Next_task)

        elif (data.data == "robot"):
            #rospy.loginfo(rospy.get_caller_id()+"Setting speech_rec_state to SPEECH_READY")
            Speech_state = SPEECH_REC_STATE.READY

            say("How may I help you %s ?" % last_person_seen)



        else:
            if (speech_callback.confAttempts < MAX_CONFIRM_ATTEMPTS):
                say("Please repeat.")
                speech_callback.confAttempts += 1
                Speech_state = SPEECH_REC_STATE.CONFIRM
            else:
                say("I did not understand your response.")


    elif (Speech_state == SPEECH_REC_STATE.READY):
        if (data.data == "robot"):
            #rospy.loginfo(rospy.get_caller_id()+"Setting speech_rec_state to SPEECH_READY")
            Speech_state = SPEECH_REC_STATE.READY

            say("How may I help you %s ?" % last_person_seen)

        else:
            #rospy.loginfo(rospy.get_caller_id()+" READY %s",data.data)	
            Next_task = proc_speech_data(data.data)

            if (Next_task == ""):
                say("I did not understand your request.")	    
                Speech_state = SPEECH_REC_STATE.READY

            else:
                say("Would you like me to " + Next_task + " ?")
                #rospy.loginfo(rospy.get_caller_id()+"Setting speech_rec_state to SPEECH_CONFIRM")
                Speech_state = SPEECH_REC_STATE.CONFIRM

    elif (Speech_state == SPEECH_REC_STATE.WAIT):
        mprint("speech state wait\n")

        speech_callback.confAttempts = 0
        if (CONVERSA == 0):
            if (data.data == "robot"):
                #rospy.loginfo(rospy.get_caller_id()+"Setting speech_rec_state to SPEECH_READY")
                Speech_state = SPEECH_REC_STATE.READY

                say("How may I help you %s ?" % last_person_seen)

            elif (data.data == "hello robot"):
                #do_easter_egg()
		Speech_state = SPEECH_REC_STATE.READY

                say("How may I help you %s ?" % last_person_seen)

        else:
	#start answers
            mprint("conversa is 1\n")
            conversa.answer_questions(data)
            
	#end answers

    # done


def tquad_callback(data):
    global track_msgcnt, Action, neck

    track_msgcnt -= 1

    if (Action != ACTION.TRACK_BALL): return
    if (Speech_state == SPEECH_REC_STATE.READY or Speech_state == SPEECH_REC_STATE.CONFIRM):
        set_action(ACTION.IDLE)
        return

    if (data.data == "tl"): neck.publish(17)
    elif (data.data == "tr"): neck.publish(18)
    elif (data.data == "ll"): neck.publish(33)
    elif (data.data == "lr"): neck.publish(34)
    elif (data.data == "cc"): neck.publish(0)

    if (track_msgcnt > 0): return

    if (data.data == "tl"): say("The ball is in the top left quadrant.")
    elif (data.data == "tr"): say("The ball is in the top right quadrant.")
    elif (data.data == "ll"): say("The ball is in the lower left quadrant.")
    elif (data.data == "lr"): say("The ball is in the lower right quadrant.")
    elif (data.data == "cc"): say("The ball is in the center.")

    track_msgcnt = 50



def objrec_callback(data):
    global Action, last_objrec_time
    
#    currtime = time.time()

#    if (currtime < last_objrec_time + OBJREC_MIN_SECS): return
#    last_objrec_time = currtime
    if (Action != ACTION.OBJREC): return

    objs = data.data.split(" ")
    
    say ("The order of objects is ")
    time.sleep(2)
    for i in xrange(0, len(objs)):

        val = int(objs[i])
        try:
            objname = objdb[val]
        except KeyError as keyerr:
            objname = "an un-known OBject."

        say (objname)
        time.sleep(2)



def neckhmove_action(val):
    global neckhmove

    neckhmove.publish(val);



def convert_to_twist(val):
    twist = Twist()
    twist.linear.x = 0.
    twist.angular.z = 0.
    
    if (val & 4): twist.linear.x = 1.4
    if (val & 8): twist.linear.x = -1.4

    if (val & 1): twist.angular.z = -1.
    if (val & 2): twist.angular.z = 1.

    return twist


def followme_action(val):
    global wheels, kbd

    if (val == 0):
        speech = "Stop moving."

    if (val == 1):
        speech = "Turn left."

    elif (val == 2):
        speech = "Turn right."

    if (val & 4):
        speech = "Move forward."

    if (val & 8):
        speech = "Move backward."

    if (val == 5 or val == 9):
        speech += " and left."

    if (val == 6 or val == 10):
        speech += " and right."

    if (val >= 128):
        speech = "I see somebody"
        say(speech)


    if kbd == 1 and val < 128:
        say(speech)

    if (val < 128):
        twist = convert_to_twist(val)
        wheels.publish(twist)


def neckhmove_callback(data):
            
    global Action
    
    val = int(data.data)
    neckhmove_action(val)


def speech_request_callback(data):
    say (data.data)


def face_recognition_callback(data):
    global Action, last_person_seen, last_facerec_time

    if Action != ACTION.IDLE or Speech_state != SPEECH_REC_STATE.WAIT: return

    currtime = rospy.get_rostime()

    if (currtime.secs < last_facerec_time + FACEREC_MIN_SECS): return

    if last_person_seen != data.feedback.names[0]:
        last_person_seen = data.feedback.names[0]
        last_facerec_time = currtime.secs
        say ("Hello %s" % last_person_seen)



def followme_callback(data):
    global Action
    if (PROMISCUOUS == 0 and (Action != ACTION.FOLLOW_ME and Action != ACTION.OBJREC)): return

    val = int(data.data)
    followme_action(val)


def arm_move(x,y,z,gp,gr,grip,delta):
    global armcontrol
    data = "0 %f %f %f %f %f %f %d" % (x, y, z, gp, gr, grip, delta)
    armcontrol.publish(data)

    # need to wait some time after publishing
    time.sleep(2)

    while arm_test(2, 0., 0., 0., 0.):
        # wait for movement to terminate
        time.sleep(.1)



def arm_test(type,x,y,z,gp):
    global mov, paused, armcheck

    mov = paused = 0
    impos = True
    
    data = "%d %f %f %f %f" % (type, x, y, z, gp)

    try:
        resp = armcheck(data)

    # wait for response from armcontrol

    except rospy.ServiceException as exc:
        rospy.loginfo("Service did not process request: " + str(exc))
        return False

    armresp = resp.resp

    vals = armresp.split(' ')
    impos = int(vals[0])
    mov = int(vals[1])
    paused = int(vals[2])

    if type == 2:
        rospy.loginfo("XMOVED is %d" % mov)
        # check if in motion
        if mov: return True
        return False

    # else check if possible
    if impos: return False # impossible point
    return True





def perform_arm(tx, ty, tz, gp):
    lift = .1
    dist = .01
    dist_apprch = 0.07
    gripclose = .7

    armrot = PI / 2.

    # move arm to posn, but about 2cm away
    ty -= dist

    rospy.loginfo("Desired position is %f %f %f %f" % (tx, ty, tz, gp))

    res = arm_test(1, tx, ty, tz, gp)

    if not res:
        say ("I cannot reach the toothpaste, sorry.")
        return

    say ("I can reach it.")

    # try to move in front
    ty -= dist_apprch
    arm_move(tx, ty, tz, 0., 0., 1., 255)

    ty += dist_apprch

    arm_move(tx, ty, tz, 0., 0., 1., 255)


    say("I moved")

    # close arm gripper
    arm_move(tx, ty, tz, 0., 0., gripclose, 255)

    say("I grabbed it")

    # move arm to posn lift by 10 cm
    res = arm_test(1, tx, ty, tz + lift, 0.)

    if res:
        # rotate gripper, lift arm
        arm_move(tx, ty, tz + lift, 0., armrot, gripclose, 255)

    else:
        # if impossible try moving back 10 cm
        res = arm_test(1, tx, ty, tz + lift, 0.)

        if res:
            ty = ty - .1
            arm_move(tx, ty - .1, tz + lift, 0., armrot, gripclose, 255)

        else:
            # if still impossible move back 20 cm
            ty = ty - .2
            arm_move(tx, ty - .2, tz + lift, 0., armrot, gripclose, 255)

#    time.sleep(3)
    say("I lifted it")

    time.sleep(2)

    arm_move(0., ty - .2, tz + lift, 0., armrot, 1., 255)
    say("Please take it")

    time.sleep(3)

    arm_move(0., 0.18, 0.1, 0., 0., 1., 128)

    set_action(ACTION.IDLE)







def arm_pickup(tx, ty, tz, gp, delta):
    #input values are in camera frame, arm coord system (x = left, y = fwd, z = up)
    # gp and delta currently ignored

    global cam_to_arm_x, cam_to_arm_y, cam_to_arm_z, cam_to_arm_roll, cam_to_arm_pitch, cam_to_arm_yaw

    # use ROS coord system
    cam_to_arm_x = 0.215 # fwd 
    cam_to_arm_y = 0. # left
    cam_to_arm_z = -0.613 # up

    cam_to_arm_pitch = -0.925
    cam_to_arm_roll = 0.
    cam_to_arm_yaw = 0.

    otx = tx
    oty = ty
    otz = tz

    say ("Toothpaste located.")

    rospy.loginfo("Camera coords %f %f %f (%f)" % (tx, ty, tz, gp))

    # trans arm -> cam to move origin
    # ROS has x fwd, y left/right, whereas camera has y fwd, x left/right !!
    tx = cam_to_arm_y
    ty = cam_to_arm_x
    tz = cam_to_arm_z

    # y is now in front, x is left/right


    # transform arm position to arm position as seen by camera, we then subtract this
    myy = ty * math.cos(cam_to_arm_pitch) + tz * math.sin(cam_to_arm_pitch)
    myz = tz * math.cos(cam_to_arm_pitch) - ty * math.sin(cam_to_arm_pitch)


    ty = myy
    tz = myz

    myx = tx * math.cos(-cam_to_arm_yaw) + ty * math.sin(-cam_to_arm_yaw)
    myy = ty * math.cos(-cam_to_arm_yaw) + tx * math.sin(-cam_to_arm_yaw)

    tx = myx
    ty = myy

    myx = tx * math.cos(-cam_to_arm_roll) + tz * math.sin(-cam_to_arm_roll)
    myz = tz * math.cos(-cam_to_arm_roll) + tz * math.sin(-cam_to_arm_roll)

    tx = myx;
    tz = myz;


    rospy.loginfo("Camera sees arm base at %f %f %f (%f)" % (tx, ty, tz, gp))

    otx -= tx
    oty -= ty
    otz -= tz

    tx = otx
    ty = oty
    tz = otz

    #origins should now coincide, we can do the rotation
    rospy.loginfo("Coords before transform are: %f %f %f (%f)" % (tx, ty, tz, gp))

    # transform coords from camera view to arm view
    myy = ty * math.cos(-cam_to_arm_pitch) + tz * math.sin(-cam_to_arm_pitch)
    myz = tz * math.cos(-cam_to_arm_pitch) - ty * math.sin(-cam_to_arm_pitch)

    ty = myy
    tz = myz

    myx = tx * math.cos(cam_to_arm_yaw) + ty * math.sin(cam_to_arm_yaw)
    myy = ty * math.cos(cam_to_arm_yaw) + tx * math.sin(cam_to_arm_yaw)

    tx = myx
    ty = myy

    myx = tx * math.cos(cam_to_arm_roll) + tz * math.sin(cam_to_arm_roll)
    myz = tz * math.cos(cam_to_arm_roll) + tz * math.sin(cam_to_arm_roll)


    # ROS has x fwd, y left/right, whereas camera has y fwd, x left/right !!
    #    tx = myx + cam_to_arm_y
    #    ty = myy + cam_to_arm_x
    #    tz = myz + cam_to_arm_z

    gp = 0.

    rospy.loginfo("Arm coords %f %f %f (%f)" % (tx, ty, tz, 0.))

    #ty += .2

    if FAKE == 0:
        perform_arm(tx, ty, tz, 0.)

    else:
        perform_arm(0., 0.32, 0.05, 0.)


        time.sleep(10)
        arm_move(0., 0.18, 0.1, 0., 0., 1., 128)

    # done




def armtarget_callback(data):
    global Action
    
    #    if (PROMISCUOUS == 0 and (Action != ACTION.TOOTHPASTE)): return
    if Action != ACTION.TOOTHPASTE: return

    # check if we are moving
    if arm_test(2,0.,0.,0.,0.): return

    vals = data.data.split(' ')

    # vals are in camera frame of ref, but in arm coordinate system (x = right, y = forward, z = up)

    tx = float(vals[0])
    ty = float(vals[1])
    tz = float(vals[2])
    
    gp = float(vals[3])

    #    gr = float(vals[4])
    #    gripclose = float(vals[5])

    delta = int(vals[6])
    
    #    tx += .14

    arm_pickup(tx, ty, tz, gp, delta)



def keydown_callback(data):
    #keyboard over-ride
    global Action, kbd
    key = data.code

    #say("You pressed "+str(data.code))

    if (key == 114):
        # key 'r'
        confirm_task(TASK_REMEMBER_FACE)
        return

    if (key == 115):
        # key 's'
        confirm_task(TASK_STOP)
        return

    if (key == 116):
        # key 't'
        confirm_task(TASK_FETCH_TOOTHPASTE)
        return

    if (key == 105 or key == 110):
        # key 'i' or 'n'
	confirm_task(TASK_IDLE)
	return

    if (Action == ACTION.FOLLOW_ME or Action == ACTION.OBJREC):
        if (key == 273):
            #key up
            kbd = 1
            followme_action(4)
            kbd = 0
            return

        if (key == 274):
            #key down
            kbd = 1
            followme_action(8)
            kbd = 0
            return

        if (key == 275):
            #key right
            kbd = 1
            followme_action(2)
            kbd = 0
            return

        if (key == 276):
            #key left
            kbd = 1
            followme_action(1)
            kbd = 0
            return

        if (key == 113):
            #key 'q'
            kbd = 1
            followme_action(5)
            kbd = 0
            return

        if (key == 101):
            #key 'e'
            kbd = 1
            followme_action(6)
            kbd = 0
            return

        if (key == 122):
            #key 'z'
            kbd = 1
            followme_action(9)
            kbd = 0
            return

        if (key == 99):
            #key 'c'
            kbd = 1
            followme_action(10)
            kbd = 0
            return

        if (key == 32):
            #key SPACE
            kbd = 1
            followme_action(0)
            kbd = 0
            return


    if (key == 102):
        #key 'f'
        confirm_task(TASK_FOLLOW_ME)
        return

    if (key == 98):
        #key 'b'
        confirm_task(TASK_TRACK_BLUE)
        return

    if (key == 114):
        #key 'r'
        confirm_task(TASK_TRACK_RED)
        return

    if (key == 103):
        #key 'g'
        confirm_task(TASK_TRACK_GREEN)
        return

    if (key == 121):
        #key 'y'
        confirm_task(TASK_TRACK_YELLOW)
        return


    if (key == 111):
        #key 'o'
        confirm_task(TASK_OBJREC)
        return


    if (key == 104):
        #key 'h'
	#do_answer_capital()
        do_easter_egg()
        return



def housekeep():
    global br, tflistener, cam_to_arm_x, cam_to_arm_y, cam_to_arm_z, cam_to_arm_roll, cam_to_arm_pitch, cam_to_arm_yaw

    # publish camera tf

    camx = 0. # fwd/back
    camy = 0. # left/right
    camz = 1.0

    camroll = 0.
    campitch = PI
    camyaw = 0.

    now = rospy.Time.now()
    
    br.sendTransform((camx, camy, camz),
                     (camroll, campitch, camyaw, 1.0),
                     now,
                     "kinect",
                     "base_link")
    
    tflistener.waitForTransform("/kinect", "/base_arm", now, rospy.Duration(4.0))
    (trans,rot) = tflistener.lookupTransform("/kinect", "/base_arm", now)

    cam_to_arm_x = trans[0]
    cam_to_arm_y = trans[1]
    cam_to_arm_z = trans[2]

    cam_to_arm_roll = rot[0]
    cam_to_arm_pitch = rot[1]
    cam_to_arm_yaw = rot[2]

    rospy.loginfo("camera -> arm is %f %f %f, %f %f %f" % (trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]))



def claustrum():

    
    global Speech_state, talk, tcolor, xprint, Action, wheels, neck, neckhmove, rservice, CONVERSA, armcontrol, armcheck, br, tflistener, spcb, kdcb, facerec, last_person_seen, last_facerec_time

    Speech_state = SPEECH_REC_STATE.WAIT

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off.

    rospy.init_node('claustrum', anonymous=False, disable_signals=True)
    xprint = rospy.Publisher('/claustrum/output', String)

    mprint("\n\n\n\n\n\n\n\nClaustrum init\n\n\n\n")

    try:
        CONVERSA = rospy.get_param('~conversa')
    except:
        CONVERSA = 0


    talk = rospy.Publisher('/chatter', String, queue_size=10)
    wheels = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    neck = rospy.Publisher('/robot/neck', UInt16, queue_size=10)
    neckhmove = rospy.Publisher('/robot/neckHMove', UInt16, queue_size=10)
 #   tcolor = rospy.Publisher('/image_tracker/target_color', Colthresh, queue_size=1)
    rservice = rospy.Publisher('/rservice', String, queue_size=10)
    armcontrol = rospy.Publisher('/armcontrol/input', String, queue_size=1)

    if FACEREC:
        facerec = rospy.Publisher('/fr_order', FRClientGoal, queue_size=1)

    Action = ACTION.NONE

    do_reset()


    spcb = rospy.Subscriber("/recognizer/output", String, speech_callback)
    last_person_seen = ""
    last_facerec_time = 0

    speech_callback.confAttempts = 0

    mprint ("\n\nWAITING 10 seconds for services to start\n")

#    br = tf.TransformBroadcaster()
#    tflistener = tf.TransformListener()

    time.sleep(5)
    rservice_shutdown("*","")
    time.sleep(5)

    # comment next line out if no arm connected
#    rospy.wait_for_service('armcontrol_check')
    armcheck = rospy.ServiceProxy('armcontrol_check', cesar_robot_arm.srv.check)

    rospy.Subscriber("/image_tracker/quadrant", String, tquad_callback)
    rospy.Subscriber("/objrec", String, objrec_callback)
    rospy.Subscriber("/followme", String, followme_callback)
    rospy.Subscriber("/eth_neckhmove", String, neckhmove_callback)
    kdcb = rospy.Subscriber("/keyboard/keydown", Key, keydown_callback)
    rospy.Subscriber("/armcontrol/target", String, armtarget_callback)
    rospy.Subscriber("/speech_request", String, speech_request_callback)

    if FACEREC:
        rospy.Subscriber("/face_recognition/feedback", FaceRecognitionActionFeedback, face_recognition_callback)

    mprint ("\n\nREADY !!!!!\n\n")

    if (CONVERSA):
        say ("Talk to me !")

    set_action(ACTION.IDLE)

    r = rospy.Rate(10) # 10hz

    arm_move(0., 0.18, 0.1, 0., 0., 1., 128)

    say("Ready.")

#    if not MOVEABLE_CAMERA:
#        housekeep()

    while not rospy.is_shutdown():
#        if MOVEABLE_CAMERA:
#            housekeep()
        r.sleep

    do_shutdown()


if __name__ == '__main__':
    global Speech_state
    Speech_state = SPEECH_REC_STATE.OFF
    signal.signal(signal.SIGINT, signal_handler)
    claustrum()

