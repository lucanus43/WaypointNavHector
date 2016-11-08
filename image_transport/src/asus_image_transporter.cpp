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
// ROS Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
// OpenCV Includes
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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
  image_transport::Subscriber sub;

  // Process
  cv::startWindowThread();
  sub = it.subscribe("camera/image", 1, imageCallback);
  
  
  // Loop until images are no longer available or landing flag has been set.
  // ..
  ros::spin();
  cv::destroyWindow("view");
}





