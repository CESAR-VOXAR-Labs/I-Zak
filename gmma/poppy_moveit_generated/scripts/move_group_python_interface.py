#!/usr/bin/python

import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

from std_msgs.msg import String

PI = 3.1415926
ANG_CORR = 0.4
UP_OFFS = 0
l0 = 0.151 # base2 -> shoulder
l1 = 0.144 # shoulder -> elbow
l2 = 0.160 # elbow -> gripper point


#########################################################################################
#########################################################################################
def move_group_python_interface():

  #First initialize moveit_commander and rospy.
  print "============ Starting setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface', anonymous=True)
  
  # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()
  
  # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()
  
  # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the arm. This interface can be used to plan and execute motions on the arm.
  group = moveit_commander.MoveGroupCommander("arm")
  
  # We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 10)
													#[ME] BECAREFUL ABOUT QUEUE_SIZE
  
  # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(10)
  print "============ Start getting basic information"
  
  
  
  ###########################
  #GETTING BASIC INFORMATION
  ###########################
  
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
  
  

  ######################
  #SOME INITIAL SETTINGS
  ######################

  #[ME] Defining a home position
  home = geometry_msgs.msg.Pose()
  home = group.get_current_joint_values()
  home[1] = -1.5
  home[2] = -1.3
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



  ####################
  #PLANNING A SCENARIO
  ####################

#  p = geometry_msgs.msg.PoseStamped()
#  p.header.frame_id = robot.get_planning_frame()
#  
#  # [ME] Creating a table
#  tbl = p
#  tbl.pose.position.x = 0.7
#  tbl.pose.position.y = 0.0
#  tbl.pose.position.z = 0.3
#  scene.add_box("table", tbl, (0.5, 1.5, 0.6))
#  print "============ WAITING FOR RVIZ TO DISPLAY THE TABLE"
#  rospy.sleep(1)
#  print "============ TABLE ADDED"

#  # [ME] Creating an object
#  obj = p
#  obj.pose.position.x = 0.65
#  obj.pose.position.y = 0.0
#  obj.pose.position.z = 0.65
#  scene.add_box("object1", obj, (0.07, 0.01, 0.1))
#  print "============ WAITING FOR RVIZ TO DISPLAY THE OBJECT1"
#  rospy.sleep(1)
#  print "============ OBJECT1 ADDED"



  ########################
  #PLANNING TO A POSE GOAL
  ########################
  
#  # We can plan a motion for this group to a desired pose for the end-effector
#  print "============ Generating plan1"
#  pose_target = geometry_msgs.msg.Pose()
#  #[ME] IS THIS POSE GOING TO BE PUBLISH BY KINECT ??
#  #[ME] PARA y <> 0 DA PROBLEMA POR CAUSA DA ORIENTACAO DO GRIPPER QUE NAO FICA (0, 0, 0)
#  pose_target.orientation.w = 1.0
#  pose_target.position.x = obj.pose.position.x - 0.1
#  pose_target.position.y = obj.pose.position.y
#  pose_target.position.z = obj.pose.position.z
#  group.set_pose_target(pose_target)
#  
#  # Now, we call the planner to compute the plan and visualize it if successful. Note that we are just planning, not asking move_group to actually move the robot.
#  plan1 = group.plan()
#  print "============ Waiting while RVIZ displays plan1..."
#  rospy.sleep(5)

  # You can ask RVIZ to visualize a plan (aka trajectory) for you. But the group.plan() method does this automatically so this is not that useful here (it just displays the same trajectory again).
#  print "============ Visualizing plan1"
#  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#  
#  display_trajectory.trajectory_start = robot.get_current_state()
#  display_trajectory.trajectory.append(plan1)
#  display_trajectory_publisher.publish(display_trajectory);
#  
#  print "============ Waiting while plan1 is visualized (again)..."
#  rospy.sleep(5)
  
  
  
  
  ######################
  #MOVING TO A POSE GOAL
  ######################
  
  # Moving to a pose goal is similar to the step above except we now use the go() function. Note that the pose goal we had set earlier is still active and so the robot will try to move to that goal. We will not use that function in this tutorial since it is a blocking function and requires a controller to be active and report success on execution of a trajectory.
  
  # Uncomment below line when working with a real robot
  # group.go(wait=True)
  
  
  
  ###############################
  #PLANNING TO A JOINT-SPACE GOAL
  ###############################
  
  # Lets set a joint space goal and move towards it. First, we will clear the pose target we had just set.
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
  

#  print "============"
#  print group.get_current_pose()
#  print "============"  


#  #####
#  print "*******"
#  actual_place = geometry_msgs.msg.Pose()
#  print "X:", actual_place.position.x
#  print "Y:", actual_place.position.y
#  print "Z:", actual_place.position.z
#  print "*******"
#  #####

#  print "============ Waiting while RVIZ displays plan2..."
#  rospy.sleep(5)
  

  
  ################
  #CARTESIAN PATHS
  ################
  
#  # You can plan a cartesian path directly by specifying a list of waypoints for the end-effector to go through.
#  
#   waypoints = []
#  
#  # start with the current pose
#  waypoints.append(group.get_current_pose().pose)
#  
#  # first orient gripper and move forward (+x)
#   wpose = geometry_msgs.msg.Pose()
#  wpose.orientation.w = 1.0
#  wpose.position.x = waypoints[0].position.x + 0.1
#  wpose.position.y = waypoints[0].position.y
#  wpose.position.z = waypoints[0].position.z
#  waypoints.append(copy.deepcopy(wpose))
#   
#  # second move down
#  wpose.position.z -= 0.10
#  waypoints.append(copy.deepcopy(wpose))
#  
#  # third move to the side
#  wpose.position.y += 0.05
#  waypoints.append(copy.deepcopy(wpose))
#  
#  # We want the cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the eef_step in cartesian translation. We will specify  the jump threshold as 0.0, effectively disabling it.
#  (plan3, fraction) = group.compute_cartesian_path (
#                               waypoints,	# waypoints to follow
#                               0.01,		# eef_step
#                               0.0)		# jump_threshold
#  
#  print "============ Waiting while RVIZ displays plan3..."
#  rospy.sleep(5)
  
  
  
  ################################################
  #ADDING/REMOVING AND ATTACHING/DETACHING OBJECTS
  #################################################
  
#  # First, we will define the collision object message
#  collision_object = moveit_msgs.msg.CollisionObject()
#  collision_object.plane_poses.x = 5
#  collision_object.plane_poses.y = 5
#  collision_object.plane_poses.z = 0
  
  # When finished shut down
  moveit_commander.roscpp_shutdown()
  
  
  print "============ STOPPING"

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

    print ("p1 is %f %f up %f fwd" % (ang,up,fwd))

    ang += PI / 2. - theta2 - ANG_CORR

    up += l1 * math.cos(ang)
    fwd += l1 * math.sin(ang)

    print("p2 is %f %f up %f fwd" % (ang,up,fwd))

    u0 = up

    ang -= theta3

    up += l2 * math.cos(ang)
    fwd += l2 * math.sin(ang)

    print("p3 is %f %f up %f fwd" % (ang,up,fwd))

    z = up + UP_OFFS

    y = fwd * math.cos(theta0)
    x = fwd * math.sin(theta0)

    print("DK position confirming --->  x: %f , y: %f , z: %f" % (x, y, z) )

    hor_angle = math.asin((up - u0)/l2)

    print("Gripper pitch confirming ->  gp: %f " % hor_angle )
    
    dist = (tx - x) * (tx - x) + (ty - y) * (ty - y) + (tz - z) * (tz - z)
    if dist > distthresh:
	print "ERROR in pos\n"
	return False

    if math.fabs(tgp - hor_angle) > gpthresh:
	print "ERROR in grip angle\n"
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
        print ("IMPOSSIBLE REQUEST\n")
        return False

    theta1 = best - ANG_CORR

    p1up = l0 * math.cos(best)
    p1fwd = l0 * math.sin(best)

    try:
        horz = math.asin((p2up - p1up)/l1)
    except ValueError:
        print ("IMPOSSIBLE\n")
        return False

    theta2 = horz + theta1

    theta3 = theta4 - theta2 + theta1

    res = confirm_DK(theta0, theta1, theta2, theta3)

    return res

#########################################################################################
#########################################################################################
if __name__=='__main__':
    global theta0, theta1, theta2, theta3, tx, ty, tz, tgp

    tx = 0.3
    ty = 0.1
    tz = 0.0
    tgp = 0.

    fwd = math.sqrt(tx * tx + ty * ty)
    theta0 = math.atan(tx/ty)

    # gp is theta4
    res = IK_alt(fwd, tz, tgp)

    rospy.loginfo("output is %f %f %f %f" % (theta0, theta1, theta2, theta3))
    if res:
        try:
	    move_group_python_interface()
        except rospy.ROSInterruptException:
	    pass

