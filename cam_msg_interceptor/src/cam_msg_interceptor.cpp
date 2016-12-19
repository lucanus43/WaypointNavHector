/* cam_msg_interceptor.cpp 
   
Desc:	Reads in camera messages on /camera/rgb/camera_info, /camera/depth/camera_info,
		/camera/rgb/image_raw and /camera/depth/image_raw topics and fixes their timestamps.
		

Author: Jeffrey Devaraj
Date: 161125 

*/

// ------------------------- DECLARATIONS ------------------------------ //

// C/C++ Includes
#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
// ROS Includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include "actionlib/client/simple_action_client.h"
// OpenCV Includes
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
// Hector includes
#include "hector_uav_msgs/TakeoffAction.h"
#include "hector_uav_msgs/LandingAction.h"
#include "hector_uav_msgs/PoseAction.h"
// Geometry Msgs
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
// TF includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"	// To convert between geometry_msgs and TF libraries
#include <rtabmap_ros/ResetPose.h>

using namespace cv;

// ---------------------- GLOBAL VARIABLES -----------------------------/

// Main variables
	std::string camera_topic_prefix = "";
	sensor_msgs::Image outputRGBImg;
	sensor_msgs::CameraInfo outputRGBCamInfo;
	sensor_msgs::Image outputDepthImg;
	sensor_msgs::CameraInfo outputDepthCamInfo;
	ros::Time lastTime;
	bool broadcastRGBInfo = false;
	bool broadcastDepthInfo = false;
	bool broadcastRGBImg = false;
	bool broadcastDepthImg = false;


// ----------------------- CALLBACK FUNCTIONS ---------------------------//

// -------------------------------------------------------------
/*
rgbImageCallBack- 	Called when RGB image received
					

Author: JDev 161215
	
*/
// -------------------------------------------------------------
void rgbImageCallBack(const sensor_msgs::ImageConstPtr& rgbImg){
	// Local variables
	// ..
	
	// Process
	if (rgbImg->header.stamp.sec == 0 && rgbImg->header.stamp.nsec == 0){
		ROS_INFO_STREAM("TIMESTAMP IS ZERO");
	} else {
		broadcastRGBImg = true;
		outputRGBImg = *rgbImg;
	}

}


// -------------------------------------------------------------
/*
depthImageCallBack- 	Called when depth image received
					

Author: JDev 161215
	
*/
// -------------------------------------------------------------
void depthImageCallBack(const sensor_msgs::ImageConstPtr& depthImg){
	// Local variables
	ros::Time now;
	int secs;
	int nsecs;
	
	// Process
		// Process
	if (depthImg->header.stamp.sec == 0 && depthImg->header.stamp.nsec == 0){
		ROS_INFO_STREAM("TIMESTAMP IS ZERO");
	} else {
		broadcastDepthImg = true;
		outputDepthImg = *depthImg;
	}
}


// -------------------------------------------------------------
/*
rgbCameraInfoBack- 	Called when rgb camera info received
					

Author: JDev 161215
	
*/
// -------------------------------------------------------------
void rgbCameraInfoCallBack(const sensor_msgs::CameraInfoConstPtr& rgbCamInfo){
	// Local variables
	ros::Time now;
	int secs;
	int nsecs;
	
	// Process
	if (rgbCamInfo->header.stamp.sec == 0 && rgbCamInfo->header.stamp.nsec == 0){
		ROS_INFO_STREAM("TIMESTAMP IS ZERO");
	} else {
		broadcastRGBInfo = true;
		outputRGBCamInfo = *rgbCamInfo;
		
	}
	
	
}

// -------------------------------------------------------------
/*
depthCameraInfoBack- 	Called when depth camera info received
					

Author: JDev 161215
	
*/
// -------------------------------------------------------------
void depthCameraInfoCallBack(const sensor_msgs::CameraInfoConstPtr& depthCamInfo){
	// Local variables
	// ..
	
	// Process
	if (depthCamInfo->header.stamp.sec == 0 && depthCamInfo->header.stamp.nsec == 0){
		ROS_INFO_STREAM("TIMESTAMP IS ZERO");
	} else {
		broadcastDepthInfo = true;
		outputDepthCamInfo = *depthCamInfo;
	}
}

// -------------------------------------------------------------
/*
main -	Main execution loop

Author: JDev 161125
	
*/
// -------------------------------------------------------------
int main(int argc, char **argv){
	// Initialise ROS
	ros::init(argc, argv, "cam_msg_interceptor");
	// Create node handle
	ros::NodeHandle nh;
	
	// Local  ROS elements (subscribers, listeners, publishers, etc.)
	ros::Subscriber rgbImageSub;
	ros::Subscriber rgbCameraInfoSub;
	ros::Subscriber depthImageSub;
	ros::Subscriber depthCameraInfoSub;
	ros::Publisher rgbImagePub;
	ros::Publisher rgbCameraInfoPub;
	ros::Publisher depthImagePub;
	ros::Publisher depthCameraInfoPub;
	
	// Resolve ros params
	nh.getParam("/cam_msg_interceptor/camera_topic", camera_topic_prefix);
	
	// Set up subscribers
	rgbImageSub = nh.subscribe(camera_topic_prefix + "/rgb/image_raw", 1, rgbImageCallBack);
	rgbCameraInfoSub = nh.subscribe(camera_topic_prefix + "/rgb/camera_info", 1, rgbCameraInfoCallBack);
	depthImageSub = nh.subscribe(camera_topic_prefix + "/depth/image_raw", 1, depthImageCallBack);
	depthCameraInfoSub = nh.subscribe(camera_topic_prefix + "/depth/camera_info", 1, depthCameraInfoCallBack);
	
	// Set up publishers
	rgbImagePub = nh.advertise<sensor_msgs::Image>("/cam_msg_interceptor/rgb/image_raw", 1);
	rgbCameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/cam_msg_interceptor/rgb/camera_info", 1);
	depthImagePub = nh.advertise<sensor_msgs::Image>("/cam_msg_interceptor/depth/image_raw", 1);
	depthCameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/cam_msg_interceptor/depth/camera_info", 1);		
	
	// Loop
	ros::Rate loopRate(50);
	while(nh.ok()){
		// Spin
		ros::spinOnce();
		// Publish fixed data
		if (broadcastRGBImg){
			rgbImagePub.publish(outputRGBImg);
			broadcastRGBImg = false;
		}
		if (broadcastRGBInfo){
			rgbCameraInfoPub.publish(outputRGBCamInfo);
			broadcastRGBInfo = false;
		}
		if (broadcastDepthImg){
			depthImagePub.publish(outputDepthImg);
			broadcastDepthImg = false;
		}
		if (broadcastDepthInfo){
			depthCameraInfoPub.publish(outputDepthCamInfo);
			broadcastDepthInfo = false;
		}
		loopRate.sleep();
	}
}

