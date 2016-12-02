/* cvUtility.h -				Header file for cvUtility.cpp and others. Contains all necessary includes.


Author: Jeffrey Devaraj
Date: 07/09/16
*/

#ifndef cvUtility__H
#define cvUtility__H

// C/C++ Includes
#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
// Includes OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>


// Namespaces
using namespace cv;
using namespace std;


// Constants
double const RAD = CV_PI / 180;		// Convert deg->rad
double const DEG = 180 / CV_PI;		// Convert rad->deg

// Functions
Mat quatnormalise(Mat Q);
Mat dcm2angle(Mat DCM);
Mat dcm2quat(Mat DCM);
Mat quat2eul(Mat QUAT);
Mat quat2dcm(Mat quat);
Mat eul2quat(Mat Eul);
Mat ang2dcm(Mat Eul);
double integrate(const double &dydx_new, const double &dydx, const double &y, const double &int_step);
Mat integrate(Mat DYDX_NEW, Mat DYDX, Mat Y, const double int_step);
Mat skew(Mat vec);
Mat trans(Mat MATRIX);
Mat quatmultiply(Mat Quat1, Mat Quat2);
void wrapto2pi(double& value);
void decompHomTrans(Mat homT, Mat &Transform, Mat &Translate);
Mat buildHomTrans(Mat Transform, Mat Translate);
// Line operations
vector<Point2f> getSamples(Vec4i Line);
Mat normal2d(Vec4i Line);
Mat normal3d(Point3d L1P1, Point3d L1P2, Point3d L2P1, Point3d L2P2);
bool CheckIntersection(vector<Point2f> Samples, Vec4i Line, Mat Normal, vector<Point2f> &Intersections, Mat &residuals);
#endif
