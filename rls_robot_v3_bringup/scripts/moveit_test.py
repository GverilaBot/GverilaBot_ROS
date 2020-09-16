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
from scratch_ros_lib.msg import position_control

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
  
## Class for Scratch custom blocks gui
class moveit_test(object):
	"""
		Class for testing moveit functions
	"""
	
	time = None
	old_time = None
	timer = None
	## Initialisation
	def __init__(self):	
		'''
			\brief Initialization functions
		'''
		super(moveit_test, self).__init__()
		## initialize `moveit_commander`_ and a:
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_test_functions_node', anonymous=True)

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
			
			
	def go_to_joint_state(self, j1=0, j2=0, j3=0,j4=0):
		'''
			\brief	Set joints position in rad
			\param: j1      first joint
			\param: j2      second joint
			\param: j3		third joint
			\param: j4     	fourth joint
			
		'''

		# We can get the joint values from the group and adjust some of the values:
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal[0] = j1
		joint_goal[1] = j2
		joint_goal[2] = j3
		joint_goal[3] = j4

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.move_group.stop()

		# For testing:
		current_joints = self.move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)
		
	def go_to_position_goal(self, x = 0.0014, y = -0.02 ,z = 0.4):
		'''
			Set position for end-effector. Any orientation is acceptable
			\param: x       X coordinate
			\param: y       Y coordinate
			\param: z       Z coordinate
			
		'''
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		move_group = self.move_group

		# Set position target
		self.move_group.set_position_target([x,y,z])

		## Now, we call the planner to compute the plan and execute it.
		plan = self.move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		self.move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		#self.move_group.clear_pose_targets()

		# For testing:
		current_pose = self.move_group.get_current_pose().pose
		#return all_close(pose_goal, current_pose, 0.05)
		
	# Print to terminal current position	
	def print_pose(self):
		current_pose = self.move_group.get_current_pose()
		print(current_pose)
	
	
	def test_cartesian_path(self, waypoints):
		'''
			\brief Function to test cartesian planning
			\param[in] waypoints set of points for cartesian move
			\return fraction of plan that was executed 
		'''
		# Plan cartesian path
		(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step = 0.005, jump_threshold = 0.0)
		# execute cartesian path
		self.move_group.execute(plan, wait=True)
		print "followed %s" % (fraction*100), "% of requested trajectory" 
		return fraction
		
	def odrive_comm_test(self):
		'''
		\brief Repeat 3 poses and print time
		'''
		while not rospy.is_shutdown():
			if self.time == None:
				self.old_time = rospy.Time.now()
				self.time = rospy.Time()
			else:
				tmp_time = rospy.Time.now()
				self.time = self.time + (tmp_time - self.old_time)
				self.old_time = tmp_time
			if (self.time.to_sec() // 60.0 % 1.0 == 0 ):
				if self.timer != (self.time.to_sec() // 60.0):
					self.timer = self.time.to_sec() // 60.0
					print "Communication test: ",self.timer, " minutes running"
			self.go_to_joint_state(-0.013, 1.36, -0.175, 1.185)
			self.go_to_joint_state(1.345, 1.721, -0.263, 2.720)
			self.go_to_joint_state(1.446, 0.496, -3.918, -0.0554)
			
		
		
def make_waypoints(test):
	'''
		\brief Make 2 points for Cartesian move from point to point
		\param[in] test object of  moveit_test class
		\return waypoints consist of start and end points appended together.
	'''
	waypoints = []
	print "Move robot to start position and press `Enter`"
	raw_input()
	pose_1 = test.move_group.get_current_pose().pose
	waypoints.append(copy.deepcopy(pose_1))
	print "Move robot to final position and press `Enter`"
	raw_input()
	pose_2 = test.move_group.get_current_pose().pose
	waypoints.append(copy.deepcopy(pose_2))
	return waypoints

def main():
	'''
		\brief Main function
	'''
	
	try:
		repeat = 'yes'
		while repeat == 'yes':
			test = moveit_test()
			print "----------------------------------------------------------"
			print "Welcome to MoveIt test "
			print "----------------------------------------------------------"
			print "Press Ctrl-D to exit at any time"
			print ""
			print "============ Press `1` to test Cartesian_path planner ============ "
			print "============ Press `2` to print EEF pose  ======================== "
			print "============ Press `3` to loop 3 poses  ========================== "
			mode_selected = raw_input('Select mode:')
			if mode_selected == '1':
				waypoints = make_waypoints(test)
				print "Move robot to desired position and press `Enter` to start cartesian move"
				raw_input()
				test.test_cartesian_path(waypoints)
				
			elif mode_selected == '2':
				test.print_pose()
			elif mode_selected == '3':
				test.odrive_comm_test()
			else:
				print "Invalid input"
			repeat = raw_input('\n Repeat the test: yes/no:')
			
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return	 
if __name__ == '__main__':
	main()

	
	#while not rospy.is_shutdown():
		# for more than just a callback
		#
	# On exit	
