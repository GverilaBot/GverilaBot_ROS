#!/usr/bin/env python
"""
	Node for communication between odrive and robot driver
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gui_ros_lib.msg import gui_control_message

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  /param: goal       A list of floats, a Pose or a PoseStamped
  /param: actual     A list of floats, a Pose or a PoseStamped
  /param: tolerance  A float
  /returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True
  
## Class for gui custom blocks gui
class gui_moveit_interface(object):
	"""
		Graphical user interface moveit interface 
	"""
	
	## Initialisation
	def __init__(self):	
		'''
			\brief Initialization function
		'''
		# Subscriber for gui commands
		self.sub = rospy.Subscriber('/gui/command', gui_control_message, self.sub_callback)	
		
		super(gui_moveit_interface, self).__init__()
		## initialize `moveit_commander`_ and a:
		moveit_commander.roscpp_initialize(sys.argv)

		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		## kinematic model and the robot's current joint states
		robot = moveit_commander.RobotCommander()

		## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
		## for getting, setting, and updating the robot's internal understanding of the
		## surrounding world:
		scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to a planning group (group of joints).  In this tutorial the group is the primary
		## arm joints in the Panda robot, so we set the group's name to "panda_arm".
		## If you are using a different robot, change this value to the name of your robot
		## arm planning group.
		## This interface can be used to plan and execute motions:
		group_name = "robot_arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
													   moveit_msgs.msg.DisplayTrajectory,
													   queue_size=20)

		# We can get the name of the reference frame for this robot:
		planning_frame = move_group.get_planning_frame()

		# We can also print the name of the end-effector link for this group:
		eef_link = move_group.get_end_effector_link()

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()

		# Misc variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names
		
		# Publish
		self.EEF_pose_publisher = rospy.Publisher('/rls_robot_v3/EEF_pose',
													   geometry_msgs.msg.PoseStamped,
													   queue_size=20)
		
		# Timer for end effector pose
		self.timer_frequency = rospy.Duration(0.05)
		self.timer = rospy.Timer(self.timer_frequency, self.publish_current_pose)
		
	## aditional shutdown procedure
	def myhook(self):
		# On exit
		self.timer.shutdown()
		
	## Function callback for income commands from gui
	def sub_callback(self, msg):
		if msg.command_type == 0:
			# Message Init to avoid raising warning when webpage is refreshed.
			pass
		elif msg.command_type == 1:	
			self.go_to_joint_state(msg.joints)
		elif msg.command_type == 2:
			self.go_to_position_goal(msg.pose.position)
		elif msg.command_type == 3:
			self.go_to_pose_goal(msg.pose)
		elif msg.command_type == 5:
			self.cartesian_path(msg.waypoints)	
		elif msg.command_type == 10:
			self.print_pose()
		
			
	def go_to_joint_state(self, joints):
		'''
			\brief	Set joints position in rad
			\param: joints      commanded joints positions		
		'''

		# We can get the joint values from the group and adjust some of the values:
		joint_goal = joints

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.move_group.stop()

		# For testing:
		current_joints = self.move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)
		
	def go_to_position_goal(self, position):
		'''
			Set position for end-effector. Any orientation is acceptable
			\param: position      position for point in global coordinate system
		'''

		# Set position target
		self.move_group.set_position_target(position)

		## Now, we call the planner to compute the plan and execute it.
		plan = self.move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.move_group.stop()

		# For testing:
		current_pose = self.move_group.get_current_pose().pose
		#return all_close(pose_goal, current_pose, 0.05)
	
	def go_to_pose_goal(self, pose):
		'''
			Set pose for end-effector. Any orientation is acceptable
			\param: pose       	position and orientation
			
		'''
		# set new pose
		goal_pose = geometry_msgs.msg.Pose()
		goal_pose.position.x = pose.position[0]
		goal_pose.position.y = pose.position[1]
		goal_pose.position.z = pose.position[2]
		
		goal_pose.orientation.x = pose.orientation[0]
		goal_pose.orientation.y = pose.orientation[1]
		goal_pose.orientation.z = pose.orientation[2]
		goal_pose.orientation.w = pose.orientation[3]
		
		# Set position target
		self.move_group.set_pose_target(goal_pose)

		## Now, we call the planner to compute the plan and execute it.
		plan = self.move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.move_group.stop()
		# clear targets after planning with poses.
		self.move_group.clear_pose_targets()

		# For testing:
		current_pose = self.move_group.get_current_pose().pose
		#return all_close(pose_goal, current_pose, 0.05)	
		
	def print_pose(self):
		current_pose = self.move_group.get_current_pose()
		print(current_pose)
		
	def publish_current_pose(self, timer):
		if not rospy.is_shutdown():
			# Read current robot pose
			current_pose = self.move_group.get_current_pose()
			# Publish current robot pose
			self.EEF_pose_publisher.publish(current_pose)
		
		
		
	
	def cartesian_path(self, msg):
		'''
			\brief  cartesian planning (MoveL)
			\param[in] msg set of points for cartesian move
			\return fraction of plan that was executed 
		'''
		waypoints = []
		# append points as a pose message
		for i in range(len(msg)):
			tmp_point = self.make_point(msg[i].position, msg[i].orientation)
			waypoints.append(copy.deepcopy(tmp_point))
		# Plan cartesian path
		(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step = 0.005, jump_threshold = 0.0)
		# execute cartesian path
		self.move_group.execute(plan, wait=True)
		print "followed %s" % (fraction*100), "% of requested trajectory" 
		return fraction
		
	def make_point(self, position, orientation):
		'''
			\brief  make point as a geometrs_msgs/pose
			\param[in] position array with xyz
			param[in] orientation array with quaternions xyzw
			\return point point defined as geometry_msgs/Pose
		'''
		
		point= geometry_msgs.msg.Pose()
		point.position.x = position[0]
		point.position.y = position[1]
		point.position.z = position[2]
		point.orientation.x = orientation[0]
		point.orientation.y = orientation[1]
		point.orientation.z = orientation[2]
		point.orientation.w = orientation[3]
		
		return point
		
if __name__ == '__main__':
	# init node and action server class
	rospy.init_node('gui_moveit_interface')
	gui_node = gui_moveit_interface()
	# additional shutdown instructions
	rospy.on_shutdown(gui_node.myhook)
	rospy.spin()


		
