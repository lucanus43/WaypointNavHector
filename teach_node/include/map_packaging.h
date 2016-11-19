/* map_packaging.h
   
Desc:		Header file for the map_packaging program
		
		Uses CV math library

Author: Jeffrey Devaraj
Date: 161114 

*/
#ifndef mapPackaging__H
#define mapPackaging__H

// C/C++ Includes
#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "cvUtility.h"

using namespace std;
using namespace cv;

// ------------------------- DECLARATIONS ------------------------------ //
// Variables
fstream poseFile;
ofstream outputFile;
vector<Mat> vecSCAA;
vector<Mat> vecQCA;
Mat QCS = Mat::zeros(4,1,CV_64F);
Mat SCSS = Mat::zeros(3,1,CV_64F);

// Functions
void packageMap(vector<Mat> vecSCAA, vector<Mat> vecTCM);
void packageMap(string poseFileLocation, Mat TCM);
bool extractPosesFromFile(fstream poseFile);


#endif
