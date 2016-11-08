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
void rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
