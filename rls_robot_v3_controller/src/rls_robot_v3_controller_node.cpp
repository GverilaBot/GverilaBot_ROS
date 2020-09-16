#include "rls_robot_v3_controller/rls_robot_v3_controller_node.h"

rls_robot_v3_controller::rls_robot_v3_controller(ros::NodeHandle& nh) : nh_(nh) {
	/**
	 * \brief Constructor
	 * \param[in] nh node handle
	*/
	/// Set simulation parameter
	nh_.getParam("/rviz_simulation", rviz_simulation);
	ROS_INFO("Rviz simulation parameter: %d", rviz_simulation);
	// calls nodeHandle constructor with namespace nh
	/// start hardware interface
	robot.reset(new rls_robot_v3_hw(rviz_simulation));
	
	///  start the controller manager
	cm.reset(new controller_manager::ControllerManager(robot.get(), nh_));
	
	///  set desired timer frequency 
	timer_hz = 50;
	///  set control loop frequency 
	control_loop_f = ros::Duration(1.0/timer_hz);
	/// non real time loop with control_loop_update callback
	control_loop = nh_.createTimer(control_loop_f, &rls_robot_v3_controller::control_loop_update, this);
	
	
	if(rviz_simulation){
		robot->set_pose_to_home_position();
	}
	else {
		// Set starting pose, reading robot's current position isn't implemented yet 
		robot->set_pose_to_home_position();
	}
		
	
}

rls_robot_v3_controller::~rls_robot_v3_controller() {
	/**
	 * \brief Desstructor
	*/
	
	ros::shutdown();
}

void rls_robot_v3_controller::control_loop_update(const ros::TimerEvent& t) {
	/**
	 * \brief Control lopp update function
	 * \param[in] t time
	*/
	
	/// read current robot state
	robot->read();	
	/// set elapsed time
	elapsed_time = ros::Duration(t.current_real - t.last_real);
	/// Update all active controllers (time,period,reset_controllers)
	cm->update(ros::Time::now(), elapsed_time);
	/// set desired robot state
	robot->write();			
}



//main
int main(int argc, char** argv) {
	
	/// init ros node
	ros::init(argc, argv, "rls_robot_v3_controller");
	/// interface for creating publishers and subscribers
	ros::NodeHandle nh;	
	/// paralel threads
	ros::AsyncSpinner spinner(2);
	/// create an object of rls_robot_v3_controller
	rls_robot_v3_controller controller(nh);
	ROS_INFO("rls robot v3 controller initialized");
	/// Start spinner
	spinner.start();
	/// Wait for program to be terminated
	ros::waitForShutdown();
}

