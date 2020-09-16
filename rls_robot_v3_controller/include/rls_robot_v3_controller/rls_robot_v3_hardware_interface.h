/**
 *	\file	rls_robot_v3_hardware_interface.h
 * 	\brief	Hardware interface header file for rls_robot_v3 
 * 	\author	Simon Kajsner
 * 
*/

/****************************************************************************************
* Include files
****************************************************************************************/
#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <odrive_comm/odrive_commanded_values.h> 
#include "rls_robot_v3_controller/serialport_0.h" 
#include "rls_robot_v3_controller/serialport_1.h"
#include "rls_robot_v3_controller/ODrive.h" 
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <cmath>
#include <string>
#include <thread>         // std::thread

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define _USE_MATH_DEFINES
#define COLOUR_RESET   "\033[0m"		/* Reset */
#define COLOUR_BLACK   "\033[30m"      	/* Black */
#define COLOUR_RED	   "\033[31m"      	/* Red */
#define COLOUR_GREEN   "\033[32m"      	/* Green */
#define COLOUR_YELLOW  "\033[33m"      	/* Yellow */	


class rls_robot_v3_hw : public hardware_interface::RobotHW {
///
/// \brief Class for robot Hardware Interface 
///
// Inherit all methods and members robot hardware interface and resource manager RobotHW	
	public:
		/// Constructor
		rls_robot_v3_hw(bool rviz_sim_only_);
		/// Destructor
		~rls_robot_v3_hw();
		/// Initialise hardware interfaces
		void init();
		/// Read current robot state
		void read();
		/// Write desired robot state
		void write();
		/// Initialize and calibrate motor drivers
		void init_driver();
		// Set pose to home position
		void set_pose_to_home_position();
		
	protected:
		
		/// Initialize and start odrive communication session
		uint8_t init_odrive_session(uint8_t odrv_number, uint32_t appSessionType, void const * appSessionSettings, uint32_t appTransportType, void const * appTransportSettings);
		// Check for odrive current state
		uint8_t get_odrive_state(uint8_t axis);
		// Write position
		void write_position();
		// Clear odrive errors
		void clear_driver_errors();
		/// Configure parameters for odrive driver
		void config();
		/// Calibrate all motors
		void calibrate();
		/// Idle state
		void state_idle();
		/// Closed loop control
		void state_closed_loop_control();
		/// Terminate session
		void terminate_odrive_session();
		/// Set starting offset for setpoint
		void set_setpoint_offset(uint8_t i);
		// Callback function
		void desired_point_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);
		/// Set trajectory limits
		void trajectory_limits();
		/// Set position trajectory mode
		void odrive_input_mode(uint8_t mode);
		
		double joint_pos[4]; 			/// current position 
        double joint_vel[4];			/// current velocity
        double joint_acc[4];			/// current acceleration
        double joint_eff[4];			/// current effort
        double joint_pos_cmd[4];		/// position command
        double joint_vel_cmd[4];		/// velocity command
        double joint_acc_cmd[4];		/// acceleration command
        double joint_eff_cmd[4];		/// effort command
		
		double k = 0.15;					/// Filter constant for position cmd
		int32_t joint_setpoint_old[4];	/// position command setpoint old value
		
		// Rviz simulation mode without robot
		bool rviz_sim_only;
		
		/// Hardware interface
		hardware_interface::JointStateInterface joint_state_interface;
		/// Joint position hardware interface
		hardware_interface::PositionJointInterface position_joint_interface;
		// To be implemented
        //hardware_interface::EffortJointInterface effort_joint_interface;
        
        //joint_limits_interface::JointLimits joint_limits;
        //joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface;
        //joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface;
        
        
        /// odrive commanded value publisher
		ros::Publisher odrive_cmd;
		/// Current desired point from planner
		ros::Subscriber desired_point;
		// Robots home position
		double home_position[4] = {0, 0, -1.5708, -1.5708};
		/// Encoder cpr for position control and setpoint
		double encoder_cpr[4] = {16384, 16384, 16384, 16384};
		/// Gear ration on each joint 
		double gear_ratio[4] = {21.66, 20.01, 20.01, 18.13};
		/// Rotation direction clockwise/anticlockwise
		int8_t rotation_direction[4] = {-1, 1, -1, -1};
		/// Motor ready
		bool motor_ready[4] = {1, 1, 1, 1};
		// Defined axis
		uint8_t odrive_axis[4] = {0, 1, 0, 1};
		// Skip joint
		bool skip_joint[4] = {0, 0, 0, 0};
		// Setpoint offset
		int32_t setpoint_offset[4] = {-5800, 900, -2780, 4663};
		// Odrive limits min and max
		double vel_limit[2] = {20000, 80000};
		double acc_limit[2] = { 5000, 30000};
		double dec_limit[2] = { 5000, 30000};
		/// control type 
		//(default: position control = 3)
		int control_type = 3;
		/// interface for creating publishers and subscribers      
        ros::NodeHandle nh_;
        
        // odrive session and transport settings
        uint32_t appSessionType = LIBODRIVE_SESSION_ASCII;
		uint32_t appTransportType = LIBODRIVE_TRANSPORT_RS232;
        t_libODrive_session_settings_ascii appSessionSettings_0;
        t_libODrive_session_settings_ascii appSessionSettings_1;
		t_libODrive_transport_settings_rs232 appTransportSettings_0;
		t_libODrive_transport_settings_rs232 appTransportSettings_1;
		// Session flag
		uint32_t odrive_result_0_ok = LIBODRIVE_RESULT_OK;
		uint32_t odrive_result_1_ok = LIBODRIVE_RESULT_OK;
		// write Buffer 
		uint8_t line[256];
		// Buffer length
		uint32_t len = 0;
		// Ascii protovol bool
		bool ascii_protocol = 1;
		
	
};
