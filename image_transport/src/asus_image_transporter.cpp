/* asus_image_transporter.cpp 
   
Desc:	Intercepts images on the /camera/rgb/image_raw and /camera/depth/image_raw topics
 	for the asus camera on Hector Quadrotor

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
// OpenCV Includes
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// Hector includes
#include "hector_uav_msgs/LandingAction.h"

// ---------------------- GLOBAL VARIABLES -----------------------------//

typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
bool quitImageTransport = false;


// ------------------------- CODE ------------------------------ //

// -------------------------------------------------------------
/*
rgbCallback - 	Called when rgb image is received. Invokes
		feature extraction.

		Derived from: http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

Author: JDev 161108
	
*/
// -------------------------------------------------------------
void rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
try {
	// Show image
	cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    	cv::waitKey(30);
	
	// Perform image manipulation operations
	// ..
  }
  catch (cv_bridge::Exception& e) {
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

// -------------------------------------------------------------
/*
landingCallBack- 	Called when landing action is called.
					Quits main loop.

Author: JDev 161109
	
*/
// -------------------------------------------------------------
void landingCallBack(const geometry_msgs::PoseStamped::ConstPtr& landingPose){
	// Set quitImageTransport to true
	quitImageTransport = true;
}



// -------------------------------------------------------------
/*
main -	Main execution loop

Author: JDev 161108
	
*/
// -------------------------------------------------------------
int main(int argc, char **argv){
	// Initialise ROS
	ros::init(argc, argv, "asus_image_listener");
	// Create node handle
	ros::NodeHandle nh;
	// Initialise OpenCV
	cv::namedWindow("view");
	

	// Local Variables
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub;
	ros::Subscriber land_sub;					// Landing subscriber
		
		
	// Start CV and subscribe to images
	cv::startWindowThread();
	imgSub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
	// Subscribe to landing
	land_sub = nh.subscribe("action/landing", 1, landingCallBack);
	ROS_INFO("[image_transporter] Landing client initialised.");
  
	// Loop until images are no longer available or landing flag has been set.
	while (ros::ok() && !quitImageTransport){
			ros::spinOnce();
	}
	
	// Destroy view
	cv::destroyWindow("view");
}





