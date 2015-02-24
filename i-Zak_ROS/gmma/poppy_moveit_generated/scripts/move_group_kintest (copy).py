#!/usr/bin/python

import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

# IT SEEMS TO BE AN INTERESTING LIBRARY
import moveit_python

from std_msgs.msg import String

PI = 3.1415926
ANG_CORR = 0.4
UP_OFFS = 0
l0 = 0.151 # base2 -> shoulder
l1 = 0.144 # shoulder -> elbow
l2 = 0.160 # elbow -> gripper point

#########################################################################################
#########################################################################################
def move_group_kintest():

  #First initialize moveit_commander and rospy.
  print "============ Starting setup"
  moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_kintest', anonymous=True)
  
  # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()
  
  # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the arm. This interface can be used to plan and execute motions on the arm.
  group = moveit_commander.MoveGroupCommander("arm")

  # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  reference_frame = group.get_planning_frame()
  scenario = moveit_python.PlanningSceneInterface(reference_frame)
  
  # We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)
													#[ME] BECAREFUL ABOUT QUEUE_SIZE
  
#  # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
#  print "============ Waiting for RVIZ..."
#  rospy.sleep(10)
#  print "============ Start getting basic information"
  
  
  

  #############################
  # GETTING BASIC INFORMATION #
  #############################
  
  # We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  
  # We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()
  
  # We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  
  # Sometimes for debugging it is useful to print the entire state of the robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  
  

  
  #####################
  # ADDING A SCENARIO #
  #####################
  
  scenario.addBox("table", 0.5, 2.0, 0.7, 0.75, 0.0, 0.35, True)
  scenario.addBox("object1", 0.05, 0.02, 0.1, 0.6, 0.0, 0.75, True)
  scenario.addCylinder("object2", 0.1, 0.02, 0.6, 0.1, 0.75, True)
  
  scenario_message = moveit_msgs.msg.PlanningScene()
  scenario.sceneCb(scenario_message)
  
  print "============ COLLISION OBJECT LIST:"
  print scenario.getKnownCollisionObjects()




  #########################
  # SOME INITIAL SETTINGS #
  #########################

  #[ME] Defining a home position
  home = geometry_msgs.msg.Pose()
  home = group.get_current_joint_values()
  home[0] = 0.0
  home[1] = -1.5
  home[2] = -1.3
  home[3] = 0.0
  home[4] = 0.0
  print "============ GOING HOME: ", home
  group.set_joint_value_target(home)
  h_plan = group.plan()
  group.execute(h_plan)
  #group.go(home)
  print "============ SLEEPING FOR 3 SECONDS"
  rospy.sleep(5)

  #[ME] Defining a workspace which involve the whole robobt
  group.set_workspace([-1.0, -1.0, -0.5, 1.0, 1.0, 1.5])

  #[ME] Setting the motion planner
  group.set_planner_id("RRTConnectkConfigDefault")




  ##################################
  # PLANNING TO A JOINT-SPACE GOAL #
  ##################################
  
  # Lets set a joint space goal and move towards it. First, we will clear any possible pose target we had set.
  group.clear_pose_targets()
  
  # Then, we will get the current set of joint values for the group
  #group_variable_values = group.get_current_joint_values()
  target1 = [theta0, theta1, theta2, theta3, 0.0]
  print "============ Joint values: ", target1
  
  # Now, lets modify one of the joints, plan to the new joint space goal and visualize the plan.
  group.set_joint_value_target(target1)

  plan1 = group.plan()
  print "============ Showing planned path before executing it"
  rospy.sleep(5)
  group.execute(plan1)
  print "============ plan1 executed !"
  rospy.sleep(5)

  # spin() simply keeps python from exiting until this node is stopped
  # rospy.spin()

  moveit_commander.roscpp_initializer.roscpp_shutdown()



#########################################################################################
#########################################################################################
def confirm_DK(theta0, theta1, theta2, theta3):
    global x, y, z, tx, ty, tz, tgp

    distthresh = 0.0001
    gpthresh = 0.1

    up = 0
    fwd = 0

    ang = theta1 + ANG_CORR

    up += l0 * math.cos(ang)
    fwd += l0 * math.sin(ang)

    print ("============ p1 is %f %f up %f fwd" % (ang,up,fwd))

    ang += PI / 2. - theta2 - ANG_CORR

    up += l1 * math.cos(ang)
    fwd += l1 * math.sin(ang)

    print("============ p2 is %f %f up %f fwd" % (ang,up,fwd))

    u0 = up

    ang -= theta3

    up += l2 * math.cos(ang)
    fwd += l2 * math.sin(ang)

    print("============ p3 is %f %f up %f fwd" % (ang,up,fwd))

    z = up + UP_OFFS

    y = fwd * math.cos(theta0)
    x = fwd * math.sin(theta0)

    print("============ DK position confirming --->  x: %f , y: %f , z: %f" % (x, y, z) )

    hor_angle = math.asin((up - u0)/l2)

    print("============ Gripper pitch confirming ->  gp: %f " % hor_angle )
    
    dist = (tx - x) * (tx - x) + (ty - y) * (ty - y) + (tz - z) * (tz - z)
    if dist > distthresh:
	print "============ ERROR in pos\n"
	return False

    if math.fabs(tgp - hor_angle) > gpthresh:
	print "============ ERROR in grip angle\n"
	return False

    return True


#########################################################################################
#########################################################################################
def IK_alt(p3fwd, p3up, theta4):
    global theta1, theta2, theta3

    theta2 = 0.

    steps = 10000
    p0min = -PI/2
    p0max = PI/2

    # position of elbow
    p2up = p3up - l2 * math.sin(theta4)
    p2fwd = p3fwd - l2 * math.cos(theta4)

    distar = l1 * l1
    best = -1000.
    minv = 100.
    rangv = (p0max - p0min) / steps
    ang = p0min - rangv

    for i in xrange(0, steps):
        ang += rangv

        p1up = l0 * math.cos(ang)
        p1fwd = l0 * math.sin(ang)

        dist = (p1up - p2up) * (p1up - p2up) + (p1fwd - p2fwd) * (p1fwd - p2fwd)

        diff = math.fabs(distar - dist)

        if diff <= minv:
            theta1 = ang - ANG_CORR

            p1up = l0 * math.cos(ang)
            p1fwd = l0 * math.sin(ang)

            try:
                horz = math.asin((p2up - p1up)/l1)
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
        print ("============ IMPOSSIBLE REQUEST\n")
        return False

    theta1 = best - ANG_CORR

    p1up = l0 * math.cos(best)
    p1fwd = l0 * math.sin(best)

    try:
        horz = math.asin((p2up - p1up)/l1)
    except ValueError:
        print ("============ IMPOSSIBLE\n")
        return False

    theta2 = horz + theta1

    theta3 = theta4 - theta2 + theta1

    res = confirm_DK(theta0, theta1, theta2, theta3)

    return res


#########################################################################################
#########################################################################################
if __name__=='__main__':
    global theta0, theta1, theta2, theta3, tx, ty, tz, tgp

    tx = 0.2
    # CAUTION: y MUST BE DIFFERENT OF 0
    ty = 0.35
    tz = 0.05
    tgp = 0.0

    fwd = math.sqrt(tx * tx + ty * ty)
    theta0 = math.atan(tx/ty)

    # gp is theta4
    res = IK_alt(fwd, tz, tgp)

    print ("============ Output is %f %f %f %f" % (theta0, theta1, theta2, theta3))
    if res:
        try:
	    move_group_kintest()
        except rospy.ROSInterruptException:
	    pass
