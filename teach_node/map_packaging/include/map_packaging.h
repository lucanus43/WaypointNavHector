/* map_packacing.h
   
Desc:		Header file for the map_packaging program
		
		Uses CV math library

Author: Jeffrey Devaraj
Date: 161114 

*/

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

using namespace std;
using namespace cv;

// ------------------------- DECLARATIONS ------------------------------ //
// Variables
fstream poseFile;
vector<Mat> vecSCAA;
vector<Mat> vecQCA;

// Functions
void packageMap(string poseFileLocation, Mat TCM);
void extractPosesFromFile(poseFile);
