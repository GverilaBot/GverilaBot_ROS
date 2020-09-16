/**
 *	\file	simple_robot_hardware_interface.cpp 
 * 	\brief	Hardware interface  file for rls_robot_v3 
 * 	\author	Simon Kajsner
 * 
*/
#include "rls_robot_v3_controller/rls_robot_v3_hardware_interface.h"



/************************************************************************************//**
** \brief		Constructor.
**
****************************************************************************************/
rls_robot_v3_hw::rls_robot_v3_hw(bool rviz_sim_only_) {
	
	rviz_sim_only = rviz_sim_only_;
	init();			/// Initialise all joint interfaces and joint handles
	ROS_INFO("Hardware interface started");
}

/************************************************************************************//**
** \brief		Destructor.
**
****************************************************************************************/
rls_robot_v3_hw::~rls_robot_v3_hw() {
	// Set idle state on exit
	state_idle();
	// terminate session
	terminate_odrive_session();
}

/************************************************************************************//**
** \brief		Initalization function for Hardware Interface.
**
****************************************************************************************/
void rls_robot_v3_hw::init() {

	// Create joint state interface and register it
	hardware_interface::JointStateHandle joint_state_handle_1("joint_1", &joint_pos[0], &joint_vel[0], &joint_eff[0]);
	joint_state_interface.registerHandle(joint_state_handle_1);
	hardware_interface::JointStateHandle joint_state_handle_2("joint_2", &joint_pos[1], &joint_vel[1], &joint_eff[1]);
	joint_state_interface.registerHandle(joint_state_handle_2);
	hardware_interface::JointStateHandle joint_state_handle_3("joint_3", &joint_pos[2], &joint_vel[2], &joint_eff[2]);
	joint_state_interface.registerHandle(joint_state_handle_3);
	hardware_interface::JointStateHandle joint_state_handle_4("joint_4", &joint_pos[3], &joint_vel[3], &joint_eff[3]);
	joint_state_interface.registerHandle(joint_state_handle_4);
    
    /// register all interfaces
	registerInterface(&joint_state_interface);
	
	// Create joint position interface and register it
	hardware_interface::JointHandle joint_position_handle_1(joint_state_interface.getHandle("joint_1"), &joint_pos_cmd[0]);
	position_joint_interface.registerHandle(joint_position_handle_1);
	hardware_interface::JointHandle joint_position_handle_2(joint_state_interface.getHandle("joint_2"), &joint_pos_cmd[1]);
	position_joint_interface.registerHandle(joint_position_handle_2);
	hardware_interface::JointHandle joint_position_handle_3(joint_state_interface.getHandle("joint_3"), &joint_pos_cmd[2]);
	position_joint_interface.registerHandle(joint_position_handle_3);
	hardware_interface::JointHandle joint_position_handle_4(joint_state_interface.getHandle("joint_4"), &joint_pos_cmd[3]);
	position_joint_interface.registerHandle(joint_position_handle_4);
    
    /// register all interfaces
	registerInterface(&position_joint_interface);
    
    /// Initialise odrive publisher
    odrive_cmd = nh_.advertise<odrive_comm::odrive_commanded_values>("odrive_set", 1);
    
    /// Initialize point subscriber
    desired_point = nh_.subscribe<trajectory_msgs::JointTrajectoryPoint>("trajectory_controller/point", 1, &rls_robot_v3_hw::desired_point_callback, this);
    
    
    /// Set settings for odrive session and transport (odrive_0 and odrive_1)
    appSessionSettings_0.timeout = 20;
    appTransportSettings_0.baudrate = 115200;
	appTransportSettings_0.portName = "/dev/ttyUSB0";
	appSessionSettings_1.timeout = 20;
    appTransportSettings_1.baudrate = 115200;
	appTransportSettings_1.portName = "/dev/ttyUSB1";
	//appTransportSettings_1.portName = "/dev/ttyACM0";
    /// Initialize, start and configure odrive communication with separate thread.
    if (ascii_protocol && !rviz_sim_only ) {
		std::thread driver(&rls_robot_v3_hw::init_driver, this);
		driver.detach();
	}
}

/************************************************************************************//**
** \brief		Function callback for desired point from planner. 
** 				Save desired velocity and accelaration for each joint.
** 
** \param     	msg Ros message (type: trajectory_msgs/JointTrajectoryPoint) 
**
****************************************************************************************/
void rls_robot_v3_hw::desired_point_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg) {
	for (int i = 0; i < 4; i++) {
		joint_vel_cmd[i] = msg->velocities[i];		/// velocity command
		joint_acc_cmd[i] = msg->accelerations[i];		/// acceleration command
		// test
		if (ascii_protocol && motor_ready[0] && motor_ready[1] && motor_ready[2] && motor_ready[3]) {
			trajectory_limits();
		}
	}
}
/************************************************************************************//**
** \brief		Send desired command values to driver. 
** 				Different control types are or will be supported.
**
****************************************************************************************/
void rls_robot_v3_hw::write() {
	if (!rviz_sim_only) { 
		// set odrive commanded value
		switch(control_type) {
			case 0: 		// Voltage control
				ROS_WARN("Voltage_control not implemented"); 
				break;
			case 1: 		// Current control
				ROS_WARN("Current control not implemented"); 
				break;
			case 2: 		// Velocity control
				ROS_WARN("Velocity control not implemented"); 
				break;
			case 3: 		// Position control
				if (ascii_protocol && motor_ready[0] && motor_ready[1] && motor_ready[2] && motor_ready[3]) {
					// Set new trajectory limits
					//trajectory_limits();
					// Write new setpoint to motor driver
					write_position();
				}
				
				break;
			case 4: 		// Trajectory control
				ROS_WARN("Trajectory control not implemented"); 
				break;
			default: 		
				ROS_ERROR("Undefined Control type"); 
		}
	}

}

/************************************************************************************//**
** \brief     Write position setpoints to odrive
** 
**
****************************************************************************************/
void rls_robot_v3_hw::write_position() {
	// ros::Time begin; 	/// Time instance
	
	int32_t joint_setpoint;
	int32_t joint_setpoint_filtered;
	
	odrive_comm::odrive_commanded_values odrive_commanded_values;	/// Commanded values
	
	nh_.getParam("/control_type", control_type);	/// set type (parameter)
	
	odrive_commanded_values.control_type = control_type;	/// set control type for ros message
	for (int i = 0;i < 4; i++) {
		joint_setpoint = joint_pos_cmd[i]/ (2*M_PI) * encoder_cpr[i] * gear_ratio[i] * rotation_direction[i] + setpoint_offset[i];	/// transformation from rad to setpoint
		// Filtering
		joint_setpoint_filtered = k * joint_setpoint + (1 - k) * joint_setpoint_old[i];
		odrive_commanded_values.robot_joints[i] = joint_setpoint_filtered;
		
		if (skip_joint[i]) {
			goto skip_to_end;
		}
		
		// Send value (setpoint) to the appropriate odrive axis 
		len = sprintf((char*)line, "p %i %i", odrive_axis[i], joint_setpoint_filtered);
		line[len] = '\n';
		len +=1;
		
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		
		
		skip_to_end: ; // Skip when needed
		// Save old joint setpoint
		joint_setpoint_old[i] = joint_setpoint_filtered; //joint_setpoint_filtered
				
	}

	odrive_cmd.publish(odrive_commanded_values);	/// publish commanded values and control type
}
/************************************************************************************//**
** \brief     Read current robot values
** 
** \param     rviz_sim Flag for simulation without a real robot.
**
****************************************************************************************/
void rls_robot_v3_hw::read() {
	
	if(rviz_sim_only) {
		for (int i = 0; i < 4; i++ ) {
			joint_pos[i] = joint_pos_cmd[i]; 	/// Read current joint position for simulation without a real robot
		}
	}
	else {
		// Temporary
		for (int i = 0; i < 4; i++ ) {
			joint_pos[i] = joint_pos_cmd[i]; 	/// Read current joint position for simulation without a real robot
		}
		// Read actual robot position
		// To be implemented
	}
	
}

/************************************************************************************//**
** \brief     Set joint_pos_cmd to home_position
** 
****************************************************************************************/
void rls_robot_v3_hw::set_pose_to_home_position() {
	
	for (int i = 0; i < 4; i++ ) {
		joint_pos_cmd[i] = home_position[i]; 	/// Read current joint position for simulation without a real robot
	}
	
}

/************************************************************************************//**
** \brief     Initialize motor drivers, start session, calibrate and config parameters.
**
****************************************************************************************/
void rls_robot_v3_hw::init_driver() {
	//tSerialPortBaudrate_0 baud_0;
	//tSerialPortBaudrate_1 baud_1;
	odrive_result_0_ok = (uint16_t)!SerialPortOpen_0(appTransportSettings_0.portName, SERIALPORT_0_BR115200);
	odrive_result_1_ok = (uint16_t)!SerialPortOpen_1(appTransportSettings_1.portName, SERIALPORT_1_BR115200);
	//calibration
	if (odrive_result_0_ok == LIBODRIVE_RESULT_OK && odrive_result_1_ok == LIBODRIVE_RESULT_OK) {
			ROS_INFO_STREAM("odrives connection status: " << "[" << COLOUR_GREEN << "ok" << COLOUR_RESET <<"]");
			// Clear errors
			clear_driver_errors();
			// Calibrate 
			//calibrate();
			// position pass through 
			//odrive_input_mode(1);
			// Enter closed loop control
			state_closed_loop_control();
			
	}
	ROS_INFO_STREAM(COLOUR_GREEN << "Odrive driver initialized" << COLOUR_RESET);
}

/************************************************************************************//**
** \brief     Initialize and start session for a specific communication
**            protocol and transport layer.
** \param     appSessionType The communication protocol to use for this session. It should
**            be a LIBODRIVE_SESSION_xxx value.
** \param     appSessionSettings Pointer to a structure with communication protocol specific
**            settings.
** \param     appTransportType The transport layer to use for the specified communication
**            protocol. It should be a LIBODRIVE_TRANSPORT_xxx value.
** \param     appTransportSettings Pointer to a structure with transport layer specific
**            settings.
**
****************************************************************************************/
//uint8_t rls_robot_v3_hw::init_odrive_session(uint8_t odrv_number, uint32_t appSessionType, void const * appSessionSettings, uint32_t appTransportType, void const * appTransportSettings) {
//	/* Temporary flag for sucessful session start */
//	bool tmp_flag;
//	/* Initialize session */
//	switch (odrv_number) {
//		case 0:
//			LibODriveSessionInit_0(appSessionType, appSessionSettings, appTransportType, appTransportSettings);
//			tmp_flag = LibODriveSessionStart_0() != LIBODRIVE_RESULT_OK;
//			break;
//		case 1:
//			LibODriveSessionInit_1(appSessionType, appSessionSettings, appTransportType, appTransportSettings);
//			tmp_flag = LibODriveSessionStart_1() != LIBODRIVE_RESULT_OK;
//			break;	
//	}
//    /* Start session. */
//    if ( tmp_flag ) {
//		ROS_INFO_STREAM("[" << COLOUR_YELLOW << "Timeout" << COLOUR_RESET <<"] Retrying to connect (reset system if this takes too long)" );
//		/* Retry to start the session */
//		/*  Once ros::ok() returns false, the node has finished shutting down. */
//		while ( ros::ok() && (tmp_flag) ) {
//			switch (odrv_number) {
//				case 0:
//					tmp_flag = LibODriveSessionStart_0() != LIBODRIVE_RESULT_OK;
//					/* Delay a bit to not pump up the CPU load. */
//					LibODriveUtilTimeDelayMs_0(20);
//					break;
//				case 1:
//					tmp_flag = LibODriveSessionStart_1() != LIBODRIVE_RESULT_OK;
//					/* Delay a bit to not pump up the CPU load. */
//					LibODriveUtilTimeDelayMs_1(20);
//					break;	
//			}
//			/* can't exit normally */
//			if ( !ros::ok() ) {
//				return LIBODRIVE_RESULT_ERROR_GENERIC;
//			}
//		}
//	}
//    ROS_INFO_STREAM("odrive connection status: " << "[" << COLOUR_GREEN << "ok" << COLOUR_RESET <<"] on odrive" << odrv_number);
//    // Clear all error on each connection.
//    
//	return LIBODRIVE_RESULT_OK;
//}

/************************************************************************************//**
** \brief     Configure odrive driver with default parameters
**
****************************************************************************************/
void rls_robot_v3_hw::config() {
	for (int i = 0; i < 4; i++) {
		
		if (i == 0 || i == 3) {
			goto skip_to_end;
		}

		len = sprintf((char*)line, "w axis%i.motor.current_control.i_gain 100", odrive_axis[i]);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		
		skip_to_end: ; // Skip when needed
	}
}

/************************************************************************************//**
** \brief     Set odrive unput mode
**
****************************************************************************************/
void rls_robot_v3_hw::odrive_input_mode(uint8_t mode) {
	for (int i = 0; i < 4; i++) {
		
		len = sprintf((char*)line, "w axis%i.controller.config.input_mode %i", odrive_axis[i], mode);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		
	}
}

/************************************************************************************//**
** \brief     Calibrate all robot joints
* 				WARNING MISSING READ FUNCTIONALITY
**
****************************************************************************************/
void rls_robot_v3_hw::calibrate() {
	uint8_t state;
	for (int i = 0; i < 4; i++) {
		
		if (skip_joint[i]) {
			goto skip_to_end;
		}
		
		len = sprintf((char*)line, "w axis%i.requested_state 3", odrive_axis[i]);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		//AXIS_STATE_ENCODER_OFFSET_CALIBRATION
		state = 7;
		
		while (state == 7) {
			
			// check if calibration is still in progress
			len = sprintf((char*)line, "r axis%i.current_state", odrive_axis[i]);
			line[len] = '\n';
			len +=1;
			(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
			state = strtod((char*)line,NULL);
			
		}
		
		skip_to_end: ; // Skip when needed
	}

}

/************************************************************************************//**
** \brief     set Idle state for all robot joints
**
****************************************************************************************/
void rls_robot_v3_hw::state_idle() {
	for (int i = 0; i < 4; i++) {
		
		if (skip_joint[i]) {
			goto skip_to_end;
		}
		
		len = sprintf((char*)line, "w axis%i.requested_state 1", odrive_axis[i]);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		
		skip_to_end: ; // Skip when needed
	}
}

/************************************************************************************//**
** \brief     set closed loop control for all robot joints
**
****************************************************************************************/
void rls_robot_v3_hw::state_closed_loop_control() {
	for (int i = 0; i < 4; i++) {
		
		if (skip_joint[i]) {
			goto skip_to_end;
		}
		
		len = sprintf((char*)line, "w axis%i.requested_state 8", odrive_axis[i]);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		
		/*Set setpoint*/
		//set_setpoint_offset(i);
		
		// configure
		config();
		
		motor_ready[i] = 1;
		
		skip_to_end: ; // Skip when needed
		
		
	}
	//ROS_INFO_STREAM("**********************************Closed loop");
	//for (uint64_t i = 0; i < 0xFFFFFFFF; i++);
	//odrive_input_mode(5);
	//ROS_INFO_STREAM("**********************************Closed loop");
	//for (uint64_t i = 0; i < 0xFFFFFFFF; i++);
	//for (int i = 0; i < 4; i++) {
	//	motor_ready[i] = 1;
	
}

/************************************************************************************//**
** \brief     Terminated odrive session
**
****************************************************************************************/
void rls_robot_v3_hw::terminate_odrive_session() {
	SerialPortClose_0();
	SerialPortClose_1();
}


/************************************************************************************//**
** \brief     	Clear odrive errors.
**
****************************************************************************************/
void rls_robot_v3_hw::clear_driver_errors() {
	for (int i = 0; i < 4; i++) {
		
		if (skip_joint[i]) {
			goto skip_to_end;
		}
		
		len = sprintf((char*)line, "w axis%i.error 0", odrive_axis[i]);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		
		skip_to_end: ; // Skip when needed
	}
}

/************************************************************************************//**
** \brief     	Set setpoint offset for desired joint.
**
****************************************************************************************/
void rls_robot_v3_hw::set_setpoint_offset(uint8_t i) {
		len = sprintf((char*)line, "r axis%i.encoder.shadow_count", odrive_axis[i]);
		line[len] = '\n';
		len +=1;
		(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
		setpoint_offset[i] = strtod((char*)line,NULL);
}
/************************************************************************************//**
** \brief     	Set velocity and acceleration limits for trajectory position control
**
****************************************************************************************/
void rls_robot_v3_hw::trajectory_limits() {
		double joint_vel_cmd_[4];
		double joint_acc_cmd_[4];
		// Set velocity limit
		for (int i = 0; i < 4; i++) {
			joint_vel_cmd_[i] = joint_vel_cmd[i];
			joint_acc_cmd_[i] = joint_acc_cmd[i];
			// minimum
			if (joint_vel_cmd_[i] < 0.1 && joint_vel_cmd_[i] > -0.1)  joint_vel_cmd_[i] = 0.1;
			
			len = sprintf((char*)line, "w axis%i.trap_traj.config.vel_limit %f", odrive_axis[i], abs(joint_vel_cmd_[i])*vel_limit[1]);
			line[len] = '\n';
			len +=1;
			(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);

			
			// Set accelaration limit
			// Acceleration
			if (joint_acc_cmd_[i] >= 0) {
				// minimum
				if (joint_acc_cmd_[i] < 0.1) joint_acc_cmd_[i] = 0.1;
				
				for (int j = 0; j < 2; j++) {
					if (j==0) {
						len = sprintf((char*)line, "w axis%i.trap_traj.config.accel_limit %f", odrive_axis[i], joint_acc_cmd_[i]*acc_limit[1]);
					}
					else {
						//len = sprintf((char*)line, "w axis%i.trap_traj.config.decel_limit %f", odrive_axis[i], pow (2, 32) - 1);
						len = sprintf((char*)line, "w axis%i.trap_traj.config.decel_limit %f", odrive_axis[i], 50*acc_limit[1]);
					}
					line[len] = '\n';
					len +=1;
					(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
				}
			}
			// Decceleration
			else {
				if (joint_acc_cmd_[i] > -0.1)  {
					// mimimum
					joint_acc_cmd_[i] = -0.1;
				} 
				for (int j = 0; j < 2; j++) {
					if (j==0) {
						//len = sprintf((char*)line, "w axis%i.trap_traj.config.accel_limit %f", odrive_axis[i], pow (2, 32) - 1);
						len = sprintf((char*)line, "w axis%i.trap_traj.config.accel_limit %f", odrive_axis[i], 50*dec_limit[1]);
					}
					else {
						len = sprintf((char*)line, "w axis%i.trap_traj.config.decel_limit %f", odrive_axis[i], abs(joint_acc_cmd_[i])*dec_limit[1]);
					}
					line[len] = '\n';
					len +=1;
					(i < 2) ? SerialPortWrite_0(line, len) : SerialPortWrite_1(line, len);
				}
			}
	}
}
