/**
 *	\file	rls_robot_v3_controller_node.h
 * 	\brief	Controller driver header file for rls_robot_v3 
 * 	\author	Simon Kajsner
 * 
*/
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "rls_robot_v3_controller/rls_robot_v3_hardware_interface.h"

class rls_robot_v3_controller {
	///
	/// \brief Class for robot controller
	///
	public:
		/// Constructor
		rls_robot_v3_controller(ros::NodeHandle& nh_);
		/// Destructor
		~rls_robot_v3_controller();
		/// Control loop update callback
		void control_loop_update(const ros::TimerEvent& t);
		
	private:
		// The shared_ptr class template stores a pointer to dynamically allocated object 
		// All threads can "read" same instance of class
		/// Controller manager and runner
		boost::shared_ptr<controller_manager::ControllerManager> cm;
		/// robot object 
		boost::shared_ptr<rls_robot_v3_hw> robot;
		/// Elapsed time 
		ros::Duration elapsed_time;
		/// Timer in hz
		double timer_hz;
		/// Control loop frequency
		ros::Duration control_loop_f;
		///  Timer manages a timer callback
		ros::Timer control_loop;
		/// interface for creating publishers and subscribers
		ros::NodeHandle nh_;
		/// Simulation parameter
		bool rviz_simulation;
		
	
};
