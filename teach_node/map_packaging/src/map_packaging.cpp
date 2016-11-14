/* map_packacing.cpp 
   
Desc:	Reads in a point cloud (.pcl) and a text file with poses (time x,y,z,qx,qy,qz,qw)
		Defines a submap frame and publishes the submap frame to global map frame transform (TSM) along 
		with the displacement of submap frame rel. global map frame (SSM).
		
		Uses CV math library

Author: Jeffrey Devaraj
Date: 161108 

*/

// ------------------------- DECLARATIONS ------------------------------ //

// C/C++ Includes
#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
// ROS Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "actionlib/client/simple_action_client.h"
// Hector includes
#include "hector_uav_msgs/LandingAction.h"
// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;


// ---------------------- GLOBAL VARIABLES -----------------------------//
//..


// ------------------------- CODE ------------------------------ //

int main(int argc, char **argv){
	// Initialise ROS
	ros::init(argc, argv, "map_packaging");
	// Create node handle
	ros::NodeHandle nh;
	
	// Local variables
	//..
	
	// Read in point cloud
	
	// Read in poses (what frame are they in?)
	
	// Use initial camera pose to define submap frame. Based on features seen in first pose?
	// Set TCS to eye(3) i.e. S frame aligned to (initial) cam frame.
	TCS = Mat::eye(3); 
	SCSS = Mat::zeros(3,1,CV_64F);
	SCAS = TCS.t()*TCA*SCAA;		// SCAA = first pose in pose file, 
	SASS = -SCAS;
	TAS = TCA.t();					// TCA = first transform in pose file.
	
	// Loop
	// for each pose (Cx) stored in some arbitrary frame SCAA/QCAA: 
		// SCAA << file
		// TCA << file
		// TCS = TCA*TAS;
		// SCAS = TCS.t()*TCA*SCAA;
		// SCSS = SCAS + SASS;
		
	
		// Calculate transform TSM where S is submap frame and M is global map frame

	
		// Calculate SSM 
	
		// Calculate SBSS from input poses and TSM(?)
	
	// end Loop
	
	// Export text file with SBSS and QBSS (quaternion) for the given submap
	
	// 
	
}
