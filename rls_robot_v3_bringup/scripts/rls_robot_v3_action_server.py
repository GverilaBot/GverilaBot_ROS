#! /usr/bin/env python
import rospy
import time
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
'''
	\file		rls_robot_v3_action_server.py
	\brief 		This module creates action service interface between move_group and controller
	\author 	
'''


# Variables
## ROS parameter for simulation without real robot (bool). 
simulation = rospy.get_param('rviz_simulation_', 'false')				
## Desired robot joint states (sensor_msgs/JointState Message).
desired_joint_states = JointState() 
## Actual robot joint state (sensor_msgs/JointState Message)
robot_joint_states = JointState()	
##	Defined joint names. 							
robot_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']		## \brief datatype list 
## Currently used controller type
controller_type = rospy.get_param('controller_type', 3)

class moveit_action(object):
	'''
		\brief	FollowJointTrajectory action server
	'''
	# Initialization
	def __init__(self, name):
		'''
			\brief 		Constructor for class moveit_action
			\param[in] 	name Name of subscribed topic. 
		'''
		# Action variables
		## Action service result 
		self.ac_result = FollowJointTrajectoryResult()
		## Action service feedback
		self.ac_feedback = FollowJointTrajectoryFeedback()
		
		# Action Initialization 
		## Topic name
		self.action_name = name
		## Action server
		self.action_server = actionlib.SimpleActionServer(self.action_name, FollowJointTrajectoryAction, execute_cb=self.action_callback, auto_start = False)
		self.action_server.start()
	
	# Action callback
	def action_callback(self, goal):
		'''
			\brief 		Action service callback
			\details 	Callback takes planned path, which consists of multiple points. 
						In sends points to controller in predefined time intervals. 
						It also provides feedback and result for Action service and handles interrupt routine.
			\param[in] 	goal Generated Path plan for robot.
		'''
		## Generated path plan with points
		goal = self.check_names_order(goal)
		trajectory = goal.trajectory
		
		# trajectory needs at least 2 points 
		if ( len(trajectory.points) < 2 ):
			return
			
		#self.make_list(goal.trajectory)
		
		## ROS Time instance
		t = rospy.Time.from_sec(time.time())
		## Prtevious point 
		last_point = trajectory.points[0]
		# Loop through all points
		for current_point in trajectory.points:
			
			# wait according to path planned
			rospy.sleep(current_point.time_from_start - last_point.time_from_start)
			
			# True if the client has requested that we stop pursuing the goal (abort)
			if self.action_server.is_preempt_requested():
				# send empty trajectory (None)
				simulation_publisher( JointState() )
				self.action_server.set_preempted()
				return
				
			# --------------------- Feedback --------------------- #
			self.ac_feedback.joint_names = trajectory.joint_names
			self.ac_feedback.desired = current_point
			self.ac_feedback.actual = self.set_actual_ac_feedback(self.ac_feedback.actual, robot_joint_states, t)
			self.ac_feedback.error = self.set_error_ac_feedback(self.ac_feedback.error ,self.ac_feedback.actual, self.ac_feedback.desired)
			self.action_server.publish_feedback(self.ac_feedback)
			# ---------------------------------------------------- #
			
			# send point
			publisher_point.publish(current_point)
			# Set controller
			desired_joint_states.position = current_point.positions
			desired_joint_states.velocity = current_point.velocities
			desired_joint_states.effort = current_point.effort
			control_publisher(desired_joint_states)
			
			# save last point
			last_point = current_point
		
			
		
		# --------------------- RESULT --------------------- #
		self.ac_result.error_code = 0
		self.action_server.set_succeeded(self.ac_result)
		# -------------------------------------------------- #
		
	def check_names_order(self, goal):	
		'''
			\brief 		This function checks name order and raises warning if necessary.
			\details 	It is not necessary to use same joints name order for move.
						(It could also rearrange order?)
			\param[in]  goal Planned path
			\return 	Planned path
		'''
		if not goal.trajectory.joint_names == robot_joint_names:
			rospy.logwarn('Joint name order from MoveIt does not match order name defined in Action Server')
		return goal
		
	def set_actual_ac_feedback(self, actual_ac_feedback, robot_joint_states, t):
		'''
			\brief 		Funtion sets actual action service feedback
			\param[in]	actual_ac_feedback Actual part of action service feedback
			\param[in]	robot_joint_states Robot joint states
			\param[in] 	t Start time
			\return 	Actual part of action service feedback
		'''
		actual_ac_feedback.positions =  robot_joint_states.position
		actual_ac_feedback.velocities =  robot_joint_states.velocity
		actual_ac_feedback.effort =  robot_joint_states.effort	
		actual_ac_feedback.time_from_start = rospy.Time.from_sec(time.time()) - t
		return 	actual_ac_feedback
		
	def set_error_ac_feedback(self, error_ac_feedback, actual_ac_feedback, desired_ac_feedback):
		'''
			\brief 		Funtion sets error action service feedback
			\param[in]	error_ac_feedback Error part of action service feedback
			\param[in]	actual_ac_feedback Actual part of action service feedback
			\param[in] 	desired_ac_feedback Desired part of action service feedback
			\return 	error part of action service feedback
		'''
		# Check if both have values and calculate difference
		if desired_ac_feedback.positions and actual_ac_feedback.positions:
			error_ac_feedback.positions = tuple(map(lambda i, j: i - j, actual_ac_feedback.positions, desired_ac_feedback.positions))
		if desired_ac_feedback.velocities and actual_ac_feedback.velocities:
			error_ac_feedback.velocities = tuple(map(lambda i, j: i - j, actual_ac_feedback.velocities, desired_ac_feedback.velocities))
		if desired_ac_feedback.effort and actual_ac_feedback.effort:
			error_ac_feedback.effort = tuple(map(lambda i, j: i - j, actual_ac_feedback.effort, desired_ac_feedback.effort))
		error_ac_feedback.time_from_start = actual_ac_feedback.time_from_start - desired_ac_feedback.time_from_start
		return 	error_ac_feedback
		
		
	
# --------------------- Functions and callbacks --------------------- #
def control_publisher(desired_controll_val):
	'''
		\brief		This function publishes data to robot controller on topic trajectory_controller/command
		\param[in] desired_controll_val Desired robot joint states  (type: sensor_msgs/JointState)
	'''
	## Define position command for ros_control 
	command_val = Float64MultiArray();
	if controller_type == 0:		# voltage
		command_val.data = desired_controll_val.effort	
	elif controller_type == 2:		# velocity
		command_val.data = desired_controll_val.velocity
	elif controller_type == 3:		# position
		command_val.data = desired_controll_val.position
	else: 	
		rospy.logwarn('controller type error')
	# publish 
	publisher_command.publish(command_val)
	
	
def robot_joint_states_callback(data):
	'''
		\brief Callback function for subcriber on topic 'joint_states'
		\details Function sets global variable robot_joint_states 
		\param[in] data Income data (type: sensor_msgs/JointState)
	'''
	global robot_joint_states 
	robot_joint_states = data
	
		
if __name__ == '__main__':
	# init node and action server class
	rospy.init_node('follow_joint_trajectory_server')
	
	# Set Subscribtion to actual robot state
	rospy.Subscriber('joint_states', JointState, robot_joint_states_callback)
	
	## ROS publisher for ros_control
	publisher_command = rospy.Publisher('trajectory_controller/command',Float64MultiArray, queue_size=10)
	# publisher for trajectory position control
	publisher_point = rospy.Publisher('trajectory_controller/point', JointTrajectoryPoint, queue_size=1)
	## Define joint names
	desired_joint_states.name = robot_joint_names
	
	## Action server 
	server = moveit_action('rls_robot_v3/follow_joint_trajectory')
	rospy.loginfo("Action server initialised")
	rospy.spin()
