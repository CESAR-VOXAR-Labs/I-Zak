#!/usr/bin/python

# author: Gabriel Max

import math
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg

# IT SEEMS TO BE AN INTERESTING LIBRARY
import moveit_python

from copy import deepcopy
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

GRIPPER_OPEN = [0.011]
GRIPPER_CLOSED = [-0.007]
PI = 3.1415926
ANG_CORR = 0.4
UP_OFFS = 0
l0 = 0.151 # base2 -> shoulder
l1 = 0.144 # shoulder -> elbow
l2 = 0.160 # elbow -> gripper point

#########################################################################################
#########################################################################################
def move_group_kintest():

  #Initialize moveit_commander and rospy.
  print "============ Starting setup"
  moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_kintest', anonymous=True)
  
  # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()
  
  # Instantiate a Pick/Place interface from moveit_python
  group_pp = moveit_python.PickPlaceInterface("arm", "gripper", plan_only = False)

  # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the arm. This interface can be used to plan and execute motions on the arm.
  group = moveit_commander.MoveGroupCommander("arm")
  group2 = moveit_commander.MoveGroupCommander("gripper")

  # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  reference_frame = group.get_planning_frame()
  scenario = moveit_python.PlanningSceneInterface(reference_frame)
  
  # We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
													#[ME] BECAREFUL ABOUT QUEUE_SIZE
  
#  # Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
#  print "============ Waiting for RVIZ..."
#  rospy.sleep(10)
#  print "============ Start getting basic information"
  
  
  

  #############################
  # GETTING BASIC INFORMATION #
  #############################
  
  # We can get the name of the reference frame for this robot
  print "============ Arm reference frame: %s" % group.get_planning_frame()
  
  # We can also print the name of the end-effector link for this group
  print "============ End-effector link: %s" % group.get_end_effector_link()
  
  # We can get a list of all the groups in the robot
  print "============ Robot groups:", robot.get_group_names()
  
  # Sometimes for debugging it is useful to print the entire state of the robot.
  print "============================= Printing robot state ============================="
  print robot.get_current_state()
  print "================================================================================"
  
  # Get some information about the gripper
  print "============ Gripper active joints: ", group2.get_active_joints()	
  print "============ Joint values: ", group2.get_current_joint_values()




  #######################################
  # INITIAL SETTINGS AND SPECIFICATIONS #
  #######################################

  #[ME] Defining a home position
  home = [0.0, -1.5, -1.3255, 0.0, 0.0]
  home_g = [0.004, 0.004]

  #[ME] Defining a workspace which involve the whole robobt
  group.set_workspace([-1.0, -1.0, -0.5, 1.0, 1.0, 1.5])

  #[ME] Allow 5 seconds per planning attempt
  group.set_planning_time(5)

  #[ME] Setting the motion planner (PICK JUST ONE)
  planners = [	"SBLkConfigDefault",
  		"ESTkConfigDefault",
		"LBKPIECEkConfigDefault",
		"BKPIECEkConfigDefault",
		"KPIECEkConfigDefault",
		"RRTkConfigDefault",
		"RRTConnectkConfigDefault",
		"RRTstarkConfigDefault",
		#"TRRTkConfigDefault",		# ESSE TA DANDO PAU !!!
		"PRMkConfigDefault",
		"PRMstarkConfigDefault"]
  group.set_planner_id(planners[4])

  #[ME] Starting some helpfull lists
  things_names = []			# list of all things in the scenario (table will be thing 0)
  things_coords = []			# list of coordinates of all things in the scenario
  things_geo = []			# list with geometry of the things
  targets = [home] 			# list of targets joints values (Home position is target 0)
  targets_coords = [[0.65, 0.65, 0.65]] # list of targets coordinates (relatives to virtual ground)
  targets_orient = [[0, 0, 0, 1.0]]	# list od orientations
  plans = [group.plan(home)] 		# list of plans (Home position is plan 0)




  #####################
  # ADDING A SCENARIO #
  #####################
  
  # Cleaning the old scenario
  obj_to_detach = scenario.getKnownAttachedObjects()
  print "============ ATTACHED OBJECTS", obj_to_detach
  i = 0
  while i < len(obj_to_detach):
  	#scene.remove_attached_object(obj_to_detach[i])
  	scenario.removeAttachedObject(obj_to_detach[i])
	i = i + 1

  obj_to_clean = scenario.getKnownCollisionObjects()
  i = 0
  while i < len(obj_to_clean):
  	scenario.removeCollisionObject(obj_to_clean[i])
  	i = i + 1

  #[ME] Lets go home before building the scenario
  group.go(home)
  group2.go(home_g) # Grips belong to another group and should be moved separately

  # Building a new scenario and filling the things lists
  scenario.addBox("table", 0.5, 2.0, 0.68, 0.75, 0.0, 0.34)
  things_names.append("table")
  things_geo.append([0.5, 2.0, 0.68])
  things_coords.append([0.75, 0.0, 0.34])

#  scenario.addBox("barrier", 0.01, 0.5, 0.1, 0.61, 0.0, 0.75)
#  things_names.append("object3")
#  things_geo.append([0.01, 0.5, 0.1])
#  things_coords.append([0.6, 0.0, 0.75])
  
  	# [ME] VOXAR LAB WILL GIVE US THIS OBJECTS COORDINATES
#  scenario.addCylinder("object1", 0.1, 0.0075, 0.6, 0.0, 0.8)
#  things_names.append("object1")
#  things_geo.append([0.1, 0.0075])
#  things_coords.append([0.6, 0.0, 0.8])
  
  scenario.addCylinder("object2", 0.1, 0.0075, 0.58, 0, 0.8)
  things_names.append("object2")
  things_geo.append([0.1, 0.0075])
  things_coords.append([0.58, 0, 0.8])

#  scenario.addCylinder("object3", 0.1, 0.0075, 0.6, 0.2, 0.8)
#  things_names.append("object3")
#  things_geo.append([0.1, 0.0075])
#  things_coords.append([0.6, 0.2, 0.8])

  # Update the object lists from a PlanningScene message. (Recieve updates from move_group)
  rospy.sleep(2)
  scenario_message = moveit_msgs.msg.PlanningScene()
  scenario.sceneCb(scenario_message, True)
  rospy.sleep(2)


  print "============ COLLISION OBJECT LIST: ", things_names
  # At this point we have a list with all objects in the scenario and its characteristics (coords and geom)
  
  print "============ RESTING AT HOME BEFORE START PLANNING AND EXECUTING"
  rospy.sleep(4)




  ####################################
  # PLANNING TO A JOINT-SPACE TARGET #
  ####################################
  
  # Lets set a(some) joint space goal(s) and move towards it(them). First, we will clear any possible pose target we had set.
  group.clear_pose_targets()

  # Then, we will get the set of joint(s) values for the target(s)

  i = 1
  while i < len(things_names):
	angle_corr = math.atan(things_coords[i][1] / things_coords[i][0])
	coord_x = things_coords[i][0] + 0.008 * math.cos(angle_corr)
	coord_y = things_coords[i][1] + 0.008 * math.sin(angle_corr)
	coord_z = things_coords[i][2]

	if calculate_goal_joint_values(coord_x, coord_y, coord_z, 0.0):
		targets.append([theta0, theta1, theta2, theta3, 0.0])
		group.set_joint_value_target(targets[-1])
		plans.append(group.plan())
		print "============   Showing plan %d" % i
	  	rospy.sleep(4)
		group.execute(plans[-1])
		pose = group.get_current_pose()
		print "ARM position: ", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
	  	targets_coords.append([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
		targets_orient.append([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
		rospy.sleep(4)
	else: print "============ SORRY I CANNOT REACH TARGET %d" % i 
		# BECAREFUL WHEN SOME TARGET COULD NOT BE REACHED !!!

	group.go(home)
	rospy.sleep(2)

	i = i + 1




  ##################
  # PICK AN OBJECT #
  ##################
  
  # SOME INITIAL SETTINGS BEFORE START PICKING OBJECTS
  # Allow some leeway in position (meters) and orientation (radians)
  group.set_goal_position_tolerance(0.05)
  group.set_goal_orientation_tolerance(0.1)
  # Allow replanning to increase the odds of a solution
  group.allow_replanning(True)
  # Set the arm reference frame
  group.set_pose_reference_frame(reference_frame)
  # Allow 30 seconds per planning attempt
  group.set_planning_time(30)
  # Set a limit on the number of pick attempts before bailing
  max_pick_attempts = 2
  
  
  # Choose the thing (0 is table) you want to pick. It depends on the way you built the scenario
  thg = 1
  print "============ COORDENADAS", targets_coords[thg]

  
  rospy.sleep(2)

  # Preparing a PoseStamped message based on the target choosed
  p_msg = geometry_msgs.msg.PoseStamped()
  p_msg.header.frame_id = group.get_planning_frame()
  p_msg.header.stamp = rospy.Time.now()
  p_msg.pose.position.x = targets_coords[thg][0]
  p_msg.pose.position.y = targets_coords[thg][1]
  p_msg.pose.position.z = targets_coords[thg][2]


  # Preparing GripperTranslation messages: defines a translation for the gripper, used in pickup tasks for example for lifting an object off a table
  	# This one should be used in pre_grasp_approach
  grip_trans_pre = moveit_msgs.msg.GripperTranslation()
  grip_trans_pre.direction.header.frame_id = "grip_base" #group.get_planning_frame() # This should be our gripper tool frame, since we usually want the approach to happen along a specific axis of the gripper, not the base. 
  grip_trans_pre.direction.vector.x = 1.0
  grip_trans_pre.direction.vector.y = 0.0
  grip_trans_pre.direction.vector.z = 0.0
	# These two fields will be filled ahead
  #grip_trans_pre.desired_distance 	# Desired translation distance to take before the grasp
  #grip_trans_pre.min_distance 		# The min distance that must be considered feasible before the grasp is even attempted

  	# This one should be used in post_grasp_retreat
  grip_trans_post = moveit_msgs.msg.GripperTranslation()
  grip_trans_post.direction.header.frame_id = group.get_planning_frame() # We go up no matter how the grip is oriented
  grip_trans_post.direction.vector.x = 0.0#-0.707
  grip_trans_post.direction.vector.y = 0.0
  grip_trans_post.direction.vector.z = 1.0#0.707
	# These two fields will be filled ahead
  #grip_trans_post.desired_distance	# Desired translation distance to take after the grasp
  #grip_trans_post.min_distance		# The min distance that must be considered feasible after the grasp
  

  # PREPARING A GRASP MESSAGE
  g = moveit_msgs.msg.Grasp()

  	# The position of the end-effector for the grasp. This is the pose of the "PARENT_LINK" of the end-effector, not actually the pose of any link *in* the end-effector. Typically this would be the pose of the most distal wrist link before the hand (end-effector) links began.
  g.grasp_pose = p_msg

  	# The APPROACH direction to take before picking an object
  g.pre_grasp_approach = grip_trans_pre
  
  	# The RETREAT direction to take after a grasp has been completed (object is attached)
  g.post_grasp_retreat = grip_trans_post

  	# The internal POSTURE of the hand for the pre-grasp. Only positions are used
  g.pre_grasp_posture.header.frame_id = "grip_base"
  g.pre_grasp_posture.joint_names = ["grip_base_to_right_finger"]
  pos = trajectory_msgs.msg.JointTrajectoryPoint()
  pos.positions = [0.012] 	# THIS OPENS THE GRIPPER. DO NOT CHANGE THIS NUMBER !!!!!
  pos.effort = [1.0]	# [CARTEADO] Set the gripper effort
  g.pre_grasp_posture.points.append(pos)

  	# The internal POSTURE of the hand for the grasp positions and efforts are used
  g.grasp_posture.header.frame_id = "grip_base"
  g.grasp_posture.joint_names = ["grip_base_to_right_finger"]
  pos = trajectory_msgs.msg.JointTrajectoryPoint()
  pos.positions.append(-0.007)	# THIS CLOSES THE GRIPPER. THIS VALUE MUST BE AT LEAST -0.012 !!!!!
  pos.effort.append(0.0)	# [CARTEADO] Set the gripper effort
  g.grasp_posture.points.append(pos)

  	# An optional list of obstacles that we have semantic info about and that can be touched/pushed/moved in the course of grasping
  g.allowed_touch_objects = [things_names[thg], things_names[0]] # Set a list of the allowed touch objects. Lets include the table !?

  # What we did above was just setting some general information at the message.
  # Now we call a function that will prepare a list of grasps just changing some parameters
  grasps_list = prepare_grasps_list(g, targets[thg][0])
  print "GRASPS LIST SIZE: %d" % len(grasps_list)
  
  # Track success/failure and number of attempts for pick operation
  success = False
  n_attempts = 0

  # Repeat until we succeed or run out of attempts
  while success != True and n_attempts < max_pick_attempts:
  	#success = group_pp.pickup(things_names[thg], grasps_list)
  	success = group.pick(things_names[thg], grasps_list)
  	n_attempts = n_attempts + 1
  	rospy.loginfo("Pick attempt: " +  str(n_attempts))
  	rospy.sleep(0.5)

  if success == True:
  	print "============ I GRASPED IT !!! \o/ ============"
	successful_pick = True
  else:
	print "============ NOT THIS TIME :( ============"
	successful_pick = False



  print "============ POSE"
  print group.get_current_pose()




  ###################
  # PLACE AN OBJECT #
  ###################

  rospy.sleep(3)
  # SOME INITIAL SETTINGS BEFORE START PLACING OBJECTS
  # Allow replanning to increase the odds of a solution
  group.allow_replanning(True)
  # Set the arm reference frame
  group.set_pose_reference_frame(reference_frame)
  # Allow 30 seconds per planning attempt
  group.set_planning_time(30)
  # Set a limit on the number of place attempts before bailing
  max_place_attempts = 1

  # Choose the location to drop the object
  desired_coord_x = 0.57
  desired_coord_y = 0.0
  desired_coord_z = 0.8

  # Correction
  angle_corr = math.atan(desired_coord_y / desired_coord_x)
  place_coord_x = desired_coord_x + 0.008 * math.cos(angle_corr)
  place_coord_y = desired_coord_y + 0.008 * math.sin(angle_corr)
  place_coord_z = desired_coord_z

  # Lets see if the desired location is reachable
  if calculate_goal_joint_values(place_coord_x, place_coord_y, place_coord_z, 0.0):
	group.set_joint_value_target([theta0, theta1, theta2, theta3, 0.0])
	place_plan = group.plan()
	rospy.sleep(3)
	group.execute(place_plan)
	place_yaw = theta0
	place_p = group.get_current_pose()
	print "============ POSE"
	print place_p
	rospy.sleep(3)
  else: print "============ SORRY I CANNOT REACH THAT PLACE LOCATION !"
 

  #testeeeee = group.get_current_joint_values()
  #testeeeee[-1] = 1.5708
  #group.go(testeeeee)
  group.set_joint_value_target([0.0, -1.5, -1.2, 0.0, 1.5708])
  place_plan = group.plan()
  rospy.sleep(2)
  group.execute(place_plan)
  rospy.sleep(3)


  # Preparing a PoseStamped message based on the desired location
  p_msg = geometry_msgs.msg.PoseStamped()
  p_msg.header.frame_id = group.get_planning_frame()
  p_msg.header.stamp = rospy.Time.now()
  p_msg.pose.position.x = desired_coord_x
  p_msg.pose.position.y = desired_coord_y
  p_msg.pose.position.z = desired_coord_z
#  p_msg.pose.position.x = place_p.pose.position.x
#  p_msg.pose.position.y = place_p.pose.position.y
#  p_msg.pose.position.z = place_p.pose.position.z
  p_msg.pose.orientation.w = 1.0


  # Preparing GripperTranslation messages: defines a translation for the gripper, used in place tasks for example for approaching the table for placing
  	# This one should be used in pre_place_approach
  grip_trans_pre = moveit_msgs.msg.GripperTranslation()
  grip_trans_pre.direction.header.frame_id = "grip_base"
  grip_trans_pre.direction.vector.x = 0.0
  grip_trans_pre.direction.vector.y = 0.0
  grip_trans_pre.direction.vector.z = -1.0
	# These two fields will be filled ahead
  #grip_trans_pre.desired_distance 	# Desired translation distance to take before going to place location
  #grip_trans_pre.min_distance 		# The min distance that must be considered feasible before the place is even attempted

  	# This one should be used in post_place_retreat
  grip_trans_post = moveit_msgs.msg.GripperTranslation()
  grip_trans_post.direction.header.frame_id = "grip_base"
  grip_trans_post.direction.vector.x = -1.0
  grip_trans_post.direction.vector.y = 0.0
  grip_trans_post.direction.vector.z = 0.0
	# These two fields will be filled ahead
  #grip_trans_post.desired_distance	# Desired translation distance to take after the place operation
  #grip_trans_post.min_distance		# The min distance that must be considered feasible after the place operation


  # PREPARING A PLACELOCATION MESSAGE
  location = moveit_msgs.msg.PlaceLocation()
  
  	# The internal posture of the hand for the grasp. Positions and efforts ARE USED !
  location.post_place_posture.header.frame_id = "grip_base"
  location.post_place_posture.header.stamp = rospy.Time.now()
  location.post_place_posture.joint_names = ["grip_base_to_right_finger"]
  pos = trajectory_msgs.msg.JointTrajectoryPoint()
  pos.positions.append(0.011)	# This opens the grip after the desired location is reached
  pos.effort.append(1.0)	# [CARTEADO]
  location.post_place_posture.points.append(pos)
  
	# The approach motion
  location.pre_place_approach = grip_trans_pre

	# The retreat motion
  location.post_place_retreat = grip_trans_post

  	# The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
  location.place_pose = p_msg

	# An optional list of obstacles that we have semantic info about and that can be touched/pushed/moved in the course of grasping
  location.allowed_touch_objects = [things_names[thg]] # Set a list of the allowed touch objects. Lets include the table !?

  # Generate a list of possible locations near the desired one
  # These will be attempted in case the desired location fails
  locations_list = prepare_places_list(location)
  print "============ LOCATIONS LIST SIZE: %d" % len(locations_list)

  # Track success/failure and number of attempts for place operation
  success = False
  n_attempts = 0

#  success = group_pp.place(things_names[thg], locations_list, things_names[0], True, [things_names[0], things_names[thg]], False)

  if successful_pick:
	print "============ I AM TRYING TO PLACE IT ..."
	# Repeat until we succeed or run out of attempts
	while success != True and n_attempts < max_place_attempts:
		for location in locations_list:
			success = group.place(things_names[thg], location)
			if success == True:
				break
			rospy.sleep(0.2)
		n_attempts = n_attempts + 1
		rospy.loginfo("Place attempt: " +  str(n_attempts))
		rospy.sleep(0.3)
  else: print "============ I COULD NOT PICK IT, SO I CANNOT PLACE IT !"

  if success == True:
  	print "============ I PLACED IT !!! \o/ ============"
	successful_place = True
  else:
	print "============ NOT THIS TIME :( ============"
	successful_place = False


  # spin() simply keeps python from exiting until this node is stopped
  #rospy.spin()

  # Shut down MoveIt cleanly
  moveit_commander.roscpp_shutdown()

  # Exit the script
  moveit_commander.os._exit(0)

#########################################################################################
#########################################################################################
def prepare_grasps_list(g_msg, yaw):
  
  # THIS PREPARES A LIST OF GRASPS MESSAGES BY VARYING DESIRED AND MIN DISTANCES IN PRE_GRASP_APPROACH FIELD

  # A list to hold the grasps
  grasps = []

  # Don't restrict contact force (<= 0 to disable)
  g_msg.max_contact_force = 0.5

  # Shift roll values to try
  r_vals = [0]
  # Shift pitch values to try
  p_vals = [0]
  # Shift yaw values to try
  y_vals = [0, -0.0174533, 0.0174533, -0.034906, 0.034906]

  # Min distances to try at pre_grasp_approach
  pre_min_vals = [0.01, 0.02]
  # Desired distances to try at pre_grasp_approach
  pre_des_vals = [0.05, 0.1, 0.15]

  # Min distances to try at post_grasp_retreat
  post_min_vals = [0.04, 0.06]
  # Desired distances to try at post_grasp_retreat
  post_des_vals = [0.05, 0.07, 0.09]

  for j_post in post_min_vals:
	g_msg.post_grasp_retreat.min_distance = j_post
	for k_post in post_des_vals:
		g_msg.post_grasp_retreat.desired_distance = k_post
		for y in y_vals:
			# Create a quaternion from the Euler angles
			q = quaternion_from_euler(0, 0, yaw + y)
			# Set the grasp pose orientation accordingly
			g_msg.grasp_pose.pose.orientation.x = q[0]
			g_msg.grasp_pose.pose.orientation.y = q[1]
			g_msg.grasp_pose.pose.orientation.z = q[2]
			g_msg.grasp_pose.pose.orientation.w = q[3]
			for j_pre in pre_min_vals:
				g_msg.pre_grasp_approach.min_distance = j_pre
				for k_pre in pre_des_vals:
					g_msg.pre_grasp_approach.desired_distance = k_pre
					
					# Set and id for this grasp (simply needs to be unique)
					g_msg.id = "pegada" + str(len(grasps))
					
					# Degrade grasp quality for increasing pitch angles
					#g_msg.grasp_quality = 1.0
					
					# Append the grasp to the list
					grasps.append(deepcopy(g_msg))

  return grasps

#########################################################################################
#########################################################################################
def prepare_places_list(pl_msg):

  # THIS PREPARES A LIST OF POSSIBLE PLACE LOCATIONS NEAR THE DESIRED ONE

  # The list which will hold the locations
  places = []

  # Remember the end-effector position relaioned to desired location
  eef_x = pl_msg.place_pose.pose.position.x
  eef_y = pl_msg.place_pose.pose.position.y
  eef_z = pl_msg.place_pose.pose.position.z

  # Shift yaw values to try
  y_vals = [0, -0.0174533, 0.0174533]#, -0.034906, 0.034906]

  # Shift values to try at X and Y directions
  shifts_xy = [0, -0.005, 0.005]

  # Min distances to try at pre_place_approach
  pre_min_vals = [0.01]
  # Desired distances to try at pre_place_approach
  pre_des_vals = [0.05, 0.08]
  # Min distances to try at post_place_retreat
  post_min_vals = [0.01]
  # Desired distances to try at post_place_retreat
  post_des_vals = [0.05, 0.07]

  for x_shift in shifts_xy:
	pl_msg.place_pose.pose.position.x = eef_x + x_shift
	for y_shift in shifts_xy:
		pl_msg.place_pose.pose.position.y = eef_y + y_shift
		yaw = math.atan( (eef_y + y_shift) / (eef_x + x_shift) )
		for y in  y_vals:
			# Create a quaternion from the Euler angles
			q = quaternion_from_euler(0, 0, yaw + y)
			# Set the grasp pose orientation accordingly
			pl_msg.place_pose.pose.orientation.x = q[0]
			pl_msg.place_pose.pose.orientation.y = q[1]
			pl_msg.place_pose.pose.orientation.z = q[2]
			pl_msg.place_pose.pose.orientation.w = q[3]
			for j_post in post_min_vals:
				pl_msg.post_place_retreat.min_distance = j_post
				for k_post in post_des_vals:
					pl_msg.post_place_retreat.desired_distance = k_post
					for j_pre in pre_min_vals:
						pl_msg.pre_place_approach.min_distance = j_pre
						for k_pre in pre_des_vals:
							pl_msg.pre_place_approach.desired_distance = k_pre
							
							# Set and id for this grasp (simply needs to be unique)
							pl_msg.id = "localizacao" + str(len(places))
							
							# Append the palce to the list
							places.append(deepcopy(pl_msg))
							
  return places




#########################################################################################
#########################################################################################
def calculate_goal_joint_values(coord_x, coord_y, coord_z, ang_grip):
    global theta0, theta1, theta2, theta3, tx, ty, tz, tgp
    
    # CHANGING THE REFERENCE FROM VIRTUAL_WORLD TO ARM_JOINT (SALSA)
    tx = coord_y
    ty = coord_x - 0.215
    tz = coord_z - 0.693#0.743
    tgp = ang_grip + 0.17453

    
    # CAUTION: y MUST BE DIFFERENT OF 0
    fwd = math.sqrt(tx * tx + ty * ty)
    theta0 = math.atan(tx/ty)

    # gp is theta4
    response = IK_alt(fwd, tz, tgp)

    #print ("============ Output is %f %f %f %f" % (theta0, theta1, theta2, theta3))

    return response

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
    global theta0, theta1, theta2, theta3

    try:
	move_group_kintest()
    except rospy.ROSInterruptException:
	pass
