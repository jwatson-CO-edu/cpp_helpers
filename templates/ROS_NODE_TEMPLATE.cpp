/*
ROS_NODE_TEMPLATE.cpp
James Watson , YYYY MONTHNAME

% ROS Node %
A ONE-LINE DESCRIPTION OF THE NODE
Publises To ----> :
Subscribes To <-- :

Dependencies: ROS , Cpp_Helpers , ROS_Helpers
Template Version: 2018-06-06
*/

// === Init ================================================================================================================================

// ~~~ Includes ~~~
// ~~ ROS ~~
#include <ros/ros.h> // -------------- ROS , Publishers , Subscribers
// ~~ Local ~~
#include <Cpp_Helpers.h> // C++ Utilities and Shortcuts
#include <ROS_Helpers.h> // ROS Utilities and Shortcuts

// ___ End Init ____________________________________________________________________________________________________________________________


/// ######### Node Vars #########

string NODE_NAME = "NODE_NAME";
int    RATE_HZ   = 100;

/// ********* End Vars *********


// === Program Functions & Classes ===


// ___ End Functions & Classes ___


// === Program Vars ===


// ___ End Vars ___


// === main ================================================================================================================================

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments
	srand( time( 0 ) ); // Random seed based on the current clock time
	
	// 0. Init ROS  &&  Register node
	ros::init( argc , argv , NODE_NAME );
	
	// 1. Fetch handle to this node
	ros::NodeHandle nh;
	
	// 2. Init node rate
	ros::Rate heartbeat( RATE_HZ );
	
	// 3. Notify
	ros_log( "[" + NODE_NAME + "] Init OK and about to run ..." , INFO );
	
	// N. Main loop
	while( ros::ok() ){ // While neither node nor ROS has been shut down
		
		/// == NODE WORK ===================================================================================================================
		
		
		
		/// __ END WORK ____________________________________________________________________________________________________________________
		
		ros::spinOnce(); // - Process messages
		heartbeat.sleep(); // Sleep for remainder of period
	}
	
	ros_log( "[" + NODE_NAME + "] Exit OK, Goodbye!" , INFO );
	
	return 0; // I guess everything turned out alright at the end!
}

// ___ End main ____________________________________________________________________________________________________________________________


/* === Spare Parts =========================================================================================================================



   ___ End Spare ___________________________________________________________________________________________________________________________
*/

