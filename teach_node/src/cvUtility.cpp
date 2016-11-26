// utility.cpp :				Utility functions
//								
//								TODO:
//	
//
// Author:						Jeffrey Devaraj
// Date:						08/07/16

#include "cvUtility.h"


//////////////////////////////////////////////////////////////////////////////////////////////
// Converts a DCM to angles - if the DCM is TBI, then the returned angles are Euler angles.
//
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat dcm2angle(Mat DCM) {
	// Local variables
	Mat Ang = Mat::zeros(3, 1, CV_64F);

	Ang.at<double>(2, 0) = atan2(DCM.at<double>(0, 1), DCM.at<double>(0, 0));		//yaw
	Ang.at<double>(1, 0) = -asin(DCM.at<double>(0, 2));								// pitch
	Ang.at<double>(0, 0) = atan2(DCM.at<double>(1, 2), DCM.at<double>(2, 2));		// roll


																					/*Ang.at<double>(2, 0) = atan2(DCM.at<double>(1, 0), DCM.at<double>(0, 0));		//yaw
																					Ang.at<double>(1, 0) = -asin(DCM.at<double>(2, 0));								// pitch
																					Ang.at<double>(0, 0) = atan2(DCM.at<double>(2, 1), DCM.at<double>(2, 2));		// roll
																					*/

	return Ang;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Converts a DCM to quaternion
// Output quaternion as [x,y,z,w] format
//
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat dcm2quat(Mat DCM) {
	// local variables
	Mat Q = Mat::zeros(4, 1, CV_64F);
	Mat Eul = Mat::zeros(3, 1, CV_64F);

	// Convert DCM to eul and output as quaternion
	//Eul = dcm2angle(DCM);
	//Q = eul2quat(Eul);
	
	Q.at<double>(3) = 0.5*sqrt(trace(DCM).val[0]+1);
	Q.at<double>(0) = (DCM.at<double>(1,2) - DCM.at<double>(2,1))/(4*Q.at<double>(3));
	Q.at<double>(1) = (DCM.at<double>(2,0) - DCM.at<double>(0,2))/(4*Q.at<double>(3));
	Q.at<double>(2) = (DCM.at<double>(0,1) - DCM.at<double>(1,0))/(4*Q.at<double>(3));

	return Q;
}



//////////////////////////////////////////////////////////////////////////////////////////////
// Converts a quaternion to DCM
// Input quaternion as [x,y,z,w] format
//
// Jeffrey Devaraj 21/11/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat quat2dcm(Mat quat) {
	// local variables
	Mat DCM = Mat::zeros(3, 3, CV_64F);
	Mat Q = Mat::zeros(4,1,CV_64F);

	// Input quaternion is in form [x,y,z,w]. Change to [w,x,y,z] and perform
	// conversion as per MATLAB quat2dcm()
	Q.at<double>(0) = quat.at<double>(3);
	Q.at<double>(1) = quat.at<double>(0);
	Q.at<double>(2) = quat.at<double>(1);
	Q.at<double>(3) = quat.at<double>(2);
	
	DCM.at<double>(0,0) = pow(Q.at<double>(0),2) + pow(Q.at<double>(1),2) - pow(Q.at<double>(2),2) - pow(Q.at<double>(3),2);
	DCM.at<double>(0,1) = 2*(Q.at<double>(1)*Q.at<double>(2) + Q.at<double>(0)*Q.at<double>(3));
	DCM.at<double>(0,2) = 2*(Q.at<double>(1)*Q.at<double>(3) - Q.at<double>(0)*Q.at<double>(2));
	DCM.at<double>(1,0) = 2*(Q.at<double>(1)*Q.at<double>(2) - Q.at<double>(0)*Q.at<double>(3));
	DCM.at<double>(1,1) = pow(Q.at<double>(0),2) - pow(Q.at<double>(1),2) + pow(Q.at<double>(2),2) - pow(Q.at<double>(3),2);
	DCM.at<double>(1,2) = 2*(Q.at<double>(2)*Q.at<double>(3) + Q.at<double>(0)*Q.at<double>(1));	
	DCM.at<double>(2,0) = 2*(Q.at<double>(1)*Q.at<double>(3) + Q.at<double>(0)*Q.at<double>(2));
	DCM.at<double>(2,1) = 2*(Q.at<double>(2)*Q.at<double>(3) - Q.at<double>(0)*Q.at<double>(1));
	DCM.at<double>(2,2) = pow(Q.at<double>(0),2) - pow(Q.at<double>(1),2) - pow(Q.at<double>(2),2) + pow(Q.at<double>(3),2);
	
	return DCM;
}


////////////////////////////////////////////////////////////////////////////////
//Returns a quaternion constructed from Euler angles
//
//160519 Created by Jeffrey Devaraj
//160720 Edited to work with cv::Mat - JDev
////////////////////////////////////////////////////////////////////////////////
Mat eul2quat(Mat Eul) {
	// local variables
	Mat QUAT = Mat::zeros(4, 1, CV_64F);
	Mat c = Mat::zeros(3, 1, CV_64F);
	Mat s = Mat::zeros(3, 1, CV_64F);

	for (int i = 0; i < 3; i++) {
		c.at<double>(i, 0) = cos(Eul.at<double>(i, 0) / 2);
		s.at<double>(i, 0) = sin(Eul.at<double>(i, 0) / 2);
	}


	QUAT.at<double>(0, 0) = c.at<double>(0, 0)*c.at<double>(1, 0)*c.at<double>(2, 0) + s.at<double>(0, 0)*s.at<double>(1, 0)*s.at<double>(2, 0);
	QUAT.at<double>(1, 0) = c.at<double>(0, 0)*c.at<double>(1, 0)*s.at<double>(2, 0) - s.at<double>(0, 0)*s.at<double>(1, 0)*c.at<double>(2, 0);
	QUAT.at<double>(2, 0) = c.at<double>(0, 0)*s.at<double>(1, 0)*c.at<double>(2, 0) + s.at<double>(0, 0)*c.at<double>(1, 0)*s.at<double>(2, 0);
	QUAT.at<double>(3, 0) = s.at<double>(0, 0)*c.at<double>(1, 0)*c.at<double>(2, 0) - c.at<double>(0, 0)*s.at<double>(1, 0)*s.at<double>(2, 0);


	return QUAT;
}

////////////////////////////////////////////////////////////////////////////////
//Returns a DCM constructed from Euler angles
//
//160521 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
Mat ang2dcm(Mat Eul) {
	// Local variables
	Mat c = Mat::zeros(3, 1, CV_64F);
	Mat s = Mat::zeros(3, 1, CV_64F);
	Mat DCM = Mat::zeros(3, 3, CV_64F);

	// Calculate sine and cosine of Eul
	/*for (int i = 0; i < 3; i++) {
	c.at<double>(i, 0) = cos(Eul.at<double>(i, 0) / 2);
	s.at<double>(i, 0) = sin(Eul.at<double>(i, 0) / 2);
	}

	DCM.at<double>(0, 0) = c.at<double>(1)*c.at<double>(0);
	DCM.at<double>(0, 1) = c.at<double>(1)*s.at<double>(0);
	DCM.at<double>(0, 2) = -s.at<double>(1);
	DCM.at<double>(1, 0) = s.at<double>(2)*s.at<double>(1)*c.at<double>(0) - c.at<double>(2)*s.at<double>(0);
	DCM.at<double>(1, 1) = s.at<double>(2)*s.at<double>(1)*s.at<double>(0) + c.at<double>(2)*c.at<double>(0);
	DCM.at<double>(1, 2) = s.at<double>(2)*c.at<double>(1);
	DCM.at<double>(2, 0) = c.at<double>(2)*s.at<double>(1)*c.at<double>(0) + s.at<double>(2)*s.at<double>(0);
	DCM.at<double>(2, 1) = c.at<double>(2)*s.at<double>(1)*s.at<double>(0) - s.at<double>(2)*c.at<double>(0);
	DCM.at<double>(2, 2) = c.at<double>(2)*c.at<double>(1);
	*/


	double spsi = sin(Eul.at<double>(2));
	double cpsi = cos(Eul.at<double>(2));
	double stht = sin(Eul.at<double>(1));
	double ctht = cos(Eul.at<double>(1));
	double sphi = sin(Eul.at<double>(0));
	double cphi = cos(Eul.at<double>(0));

	DCM.at<double>(0, 0) = cpsi*ctht;
	DCM.at<double>(1, 0) = cpsi*stht*sphi - spsi*cphi;
	DCM.at<double>(2, 0) = cpsi*stht*cphi + spsi*sphi;
	DCM.at<double>(0, 1) = spsi*ctht;
	DCM.at<double>(1, 1) = spsi*stht*sphi + cpsi*cphi;
	DCM.at<double>(2, 1) = spsi*stht*cphi - cpsi*sphi;
	DCM.at<double>(0, 2) = -stht;
	DCM.at<double>(1, 2) = ctht*sphi;
	DCM.at<double>(2, 2) = ctht*cphi;


	return DCM;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Converts quaternion to Euler angles
//
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat quat2eul(Mat QUAT)
{
	// local variables
	Mat Eul = Mat::zeros(3, 1, CV_64F);
	double qw, qx, qy, qz;

	// Assign variables from QUAT to qw, qx, qy, qz
	qw = QUAT.at<double>(0, 0); qx = QUAT.at<double>(1, 0); qy = QUAT.at<double>(2, 0); qz = QUAT.at<double>(3, 0);

	// Determine Euler angle vector
	Eul.at<double>(0, 0) = atan2(2 * (qx*qy + qw*qz), pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2));
	Eul.at<double>(1, 0) = asin(-2 * (qx*qz - qw*qy));
	Eul.at<double>(2, 0) = atan2(2 * (qy*qz + qw*qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2));


	return Eul;
}


//////////////////////////////////////////////////////////////////////////////////////////////
// Computes scalar integration
//
// Derived from Peter Zipfel CADAC++ implementaton of function
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
double integrate(const double &dydx_new, const double &dydx, const double &y, const double &int_step)
{
	return y + (dydx_new + dydx)*int_step / 2;
}


//////////////////////////////////////////////////////////////////////////////////////////////
// Computes vector integration
//
// Derived from Peter Zipfel CADAC++ implementaton of function
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat integrate(Mat DYDX_NEW, Mat DYDX, Mat Y, const double int_step)
{
	int nrow = Y.rows; int nrow1 = DYDX_NEW.rows; int nrow2 = DYDX.rows;
	int ncol = Y.cols; int ncol1 = DYDX_NEW.cols; int ncol2 = DYDX.cols;

	if (nrow != nrow1 || nrow != nrow2)
	{
		std::cerr << " *** Error: incompatible row-dimensions in 'integrate()' *** \n"; exit(1);
	}
	if (ncol != ncol1 || ncol != ncol2)
	{
		std::cerr << " *** Error: incompatible column-dimensions in 'integrate()' *** \n"; exit(1);
	}

	Mat RESULT = Mat::zeros(nrow, ncol, CV_64F);
	for (int r = 0; r < nrow; r++) {
		for (int c = 0; c < ncol; c++) {
			RESULT.at<double>(r, c) = integrate(DYDX_NEW.at<double>(r, c), DYDX.at<double>(r, c), Y.at<double>(r, c), int_step);
		}
	}

	return RESULT;

}

//////////////////////////////////////////////////////////////////////////////////////////////
// Computes skew symmetric matrix representation of a vector quantity
//
// Derived from Peter Zipfel CADAC++ implementaton of function
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat skew(Mat vec) {
	// Local variables
	double a, b, c;
	Mat SKEW = Mat::zeros(3, 3, CV_64F);

	// Assign vec to a, b and c
	a = vec.at<double>(0, 0);
	b = vec.at<double>(1, 0);
	c = vec.at<double>(2, 0);

	// Assign values to SKEW
	SKEW.at<double>(0, 1) = -c;
	SKEW.at<double>(0, 2) = b;
	SKEW.at<double>(1, 0) = c;
	SKEW.at<double>(1, 2) = -a;
	SKEW.at<double>(2, 0) = -b;
	SKEW.at<double>(2, 1) = a;
	return SKEW;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Computes transpose of a matrix
//
// Jeffrey Devaraj 12/07/16
//////////////////////////////////////////////////////////////////////////////////////////////
Mat trans(Mat MATRIX) {
	// Local variables
	Mat RESULT = Mat::zeros(MATRIX.cols, MATRIX.rows, CV_64F);
	int num_row = MATRIX.cols;
	int num_col = MATRIX.rows;

	// Perform transpose
	for (int r = 0; r < num_row; r++) {
		for (int c = 0; c < num_col; c++) {
			RESULT.at<double>(r, c) = MATRIX.at<double>(c, r);
		}
	}
	return RESULT;
}

////////////////////////////////////////////////////////////////////////////////
// Performs the multiplication of two quaternions
//
//160519 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
Mat quatmultiply(Mat Quat1, Mat Quat2) {
	// local variables
	Mat QOut = Mat::zeros(4, 1, CV_64F);
	double t0(0); double t1(0); double t2(0); double t3(0);
	double q0(0); double q1(0); double q2(0); double q3(0);
	double r0(0); double r1(0); double r2(0); double r3(0);

	q0 = Quat1.at<double>(0, 0); q1 = Quat1.at<double>(1, 0); q2 = Quat1.at<double>(2, 0); q3 = Quat1.at<double>(3, 0);
	r0 = Quat2.at<double>(0, 0); r1 = Quat2.at<double>(1, 0); r2 = Quat2.at<double>(2, 0); r3 = Quat2.at<double>(3, 0);

	t0 = (r0*q0 - r1*q1 - r2*q2 - r3*q3);
	t1 = (r0*q1 + r1*q0 - r2*q3 + r3*q2);
	t2 = (r0*q2 + r1*q3 + r2*q0 - r3*q1);
	t3 = (r0*q3 - r1*q2 + r2*q1 + r3*q0);

	QOut.at<double>(0, 0) = t0;
	QOut.at<double>(1, 0) = t1;
	QOut.at<double>(2, 0) = t2;
	QOut.at<double>(3, 0) = t3;

	return QOut;
}

////////////////////////////////////////////////////////////////////////////////
// Wraps an angle in radians to 2*pi
//
//160519 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
void wrapto2pi(double& value) {
	// Local variables
	bool input = (value > 0);
	double newVal;

	// Calculation
	newVal = remainder(value, (2 * CV_PI));
	if (newVal <= 0.01 && newVal >= -0.01 && input == true) {
		value = 2 * CV_PI;
	}

}

////////////////////////////////////////////////////////////////////////////////
// Decompose homogenous transform into a transformation matrix and a
// translation vector.
//
//160912 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
void decompHomTrans(Mat homT, Mat &Transform, Mat &Translate) {
	// Loop to populate Transformation matrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Transform.at<double>(i, j) = homT.at<double>(i,j);
		}
	}
	// Populate translation vector
	Translate = homT.col(3);
	Translate.resize(3);
}


////////////////////////////////////////////////////////////////////////////////
// Build homogenous transform from a transformation matrix and a
// translation vector.
//
//160912 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
Mat buildHomTrans(Mat Transform, Mat Translate) {
	// Local variables
	Mat HomT = Mat::zeros(4,4,CV_64F);

	// Add transform to HomT
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			HomT.at<double>(i, j) = Transform.at<double>(i,j);
		}
	}
	// Add translation info to HomT
	for (int i = 0; i < 3; i++) {
		HomT.at<double>(i, 3) = Translate.at<double>(i);
	}

	// Add last row of homogenous matrix
	HomT.at<double>(3, 3) = 1.0;

	// Return
	return HomT;
	
}

////////////////////////////////////////////////////////////////////////////////
// Given a line (vector<Vec4i> array), return a vector<Point> of samples.
//
//160919 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
vector<Point2f> getSamples(Vec4i Line) {
	// Local variables
	int lineLength = 0;
	Point pt1;
	Point pt2;
	vector<Point2f> Samples;
	double num_x; double num_y;
	double m;
	double SampleSpaces;
	int numSamples = 9;			// Pick 8 samples between start and end points. 9 because indexing starts at 0.
	// Process
	// Obtain 10 samples from the line, if possible.
	pt1 = Point2f(Line[0], Line[1]);
	pt2 = Point2f(Line[2], Line[3]);
	// Use y=mx+b
	
	
	// DEBUG //
	//cout << "pt1: " << pt1 << endl;
	//cout << "pt1.x: " << pt1.x << endl;
	//cout << "pt1.y: " << pt1.y << endl;
	//cout << "pt2: " << pt2 << endl;
	//cout << "(pt1.y - pt2.y)" << (pt1.y - pt2.y) << endl;
	//cout << "(pt1.x - pt2.x)" << (pt1.x - pt2.x) << endl;
	//cout << "(pt1.y - pt2.y) / (pt1.x - pt2.x)" << (double) (pt1.y - pt2.y) / (double) (pt1.x - pt2.x) << endl;
	// DEBUG //
	
	
	m = (double) (pt1.y - pt2.y) / (double) (pt1.x - pt2.x);

	// DEBUG //
	//cout << "m: " << m << endl;

	num_x = (abs((double)(pt2.x - pt1.x)));
	num_y = (abs((double)(pt2.y - pt1.y)));

	// Push back the first point
	Samples.push_back(pt1);
	if (num_x > 0) {		// Will also handle horizontal lines (m = 0)
		// There are num_x elements. Pick 10.
		SampleSpaces = num_x / numSamples;
		/*cout << "Samples[0]" << Samples[0] << endl;*/
		for (int i = 1; i < numSamples; i++) {
			/*cout << "i:" << i << endl;
			cout << "SampleSpaces: " << SampleSpaces << endl;
			cout << "Point2f(pt1.x + i*SampleSpaces, (double) pt1.y + m*(i*SampleSpaces)): " << Point2f(pt1.x + i*SampleSpaces, (double)pt1.y + m*(i*SampleSpaces)) << endl;*/
			Samples.push_back(Point2f(pt1.x + i*SampleSpaces, (double)pt1.y + m*(i*SampleSpaces)));
		}
	}
	else if (num_y > 0) {		// num_x = 0, m = inf; vertical line
		// There are num_y elements. Pick 10.
		SampleSpaces = num_y / numSamples;
		// m = inf so we need sign of m.
		m = (double)(pt2.y - pt1.y) / fabs(pt1.y - pt2.y);	// if pt1.y > pt2.y then m must be negative
		for (int i = 1; i < numSamples; i++) {
			/*cout << "i:" << i << endl;
			cout << "SampleSpaces: " << SampleSpaces << endl;
			cout << "Point2f(pt1.x, (double)pt1.y + m*i*SampleSpaces): " << Point2f(pt1.x, (double)pt1.y + m*i*SampleSpaces) << endl;*/
			Samples.push_back(Point2f(pt1.x, (double)pt1.y + m*i*SampleSpaces));
		}
	}
	// Push back the last point
	Samples.push_back(pt2);
	/*cout << "Samples[9]" << Samples[9] << endl;*/
	
	return	Samples;
}


////////////////////////////////////////////////////////////////////////////////
// Given a line (vector<Vec4i> array), return its normal.
//
//160920 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
Mat normal2d(Vec4i Line) {
	// Local variables
	double dx, dy;
	Point2f pt1, pt2;
	Mat Normal = Mat::zeros(2,1,CV_64F);
	// Processing
	pt1 = Point2f(Line[0], Line[1]);
	pt2 = Point2f(Line[2], Line[3]);
	//Turn Line into a vector using y = mx + b; [y,x]
	dx = (double)(pt2.x - pt1.x);
	dy = (double)(pt2.y - pt1.y);
	Normal.at<double>(0, 0) = -dy;
	Normal.at<double>(0, 1) = dx;

	return Normal;

}

////////////////////////////////////////////////////////////////////////////////
// Given a 3D line in the form of 4 points, return its normal.
// L1P1: Line 1, Point 1.
// L2P2: Line 2, Point 2.
//
//161010 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////
Mat normal3d(Point3d L1P1, Point3d L1P2, Point3d L2P1, Point3d L2P2) {
	// Local variables
	Mat vec1 = Mat::zeros(3, 1, CV_64F);
	Mat vec2 = Mat::zeros(3, 1, CV_64F);
	Mat Normal = Mat::zeros(3, 1, CV_64F);

	// Create two vectors from the 4 input points
	vec1 = Mat(L1P2 - L1P1);
	vec2 = Mat(L2P2 - L2P1);
	// Normal to the plane is given by the cross product of two lines on the plane (vec1 and vec2)
	Normal = vec1.cross(vec2);

	return Normal;

}


////////////////////////////////////////////////////////////////////////////////
// Given samples, a line and its normal, function determines where a line
// drawn parallel to the supplied normal, at each of the sample points,
// intersects the input line.
// from http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
//
//160920 Created by Jeffrey Devaraj
////////////////////////////////////////////////////////////////////////////////

bool CheckIntersection(vector<Point2f> Samples, Vec4i Line, Mat Normal, vector<Point2f> &Intersections, Mat &residuals) {
	// Local variables
	Mat P = Mat::zeros(3, 1, CV_64F);
	Mat R = Mat::zeros(3, 1, CV_64F);
	Mat C = Mat::zeros(3, 1, CV_64F);
	Mat S = Mat::zeros(3, 1, CV_64F);
	Mat Q = Mat::zeros(3, 1, CV_64F);
	Point2f pt1, pt2;
	Mat pt1_Mat, pt2_Mat;
	Mat normLine = Mat::zeros(3, 1, CV_64F);
	Mat LineVec = Mat::zeros(3, 1, CV_64F);
	double u;
	double min_u = 50000;
	Mat Intersection;
	double res;			// Residual
	int counter = 0;
	bool returnVal = false;
	vector<double> residualVec; // Vector of residuals corresponding to intersections

	// Initialise input matrices
	//residuals = Mat::zeros(1,Samples.size(),CV_64F);

	// Convert line to points
	pt1 = Point2f(Line[0], Line[1]);
	pt2 = Point2f(Line[2], Line[3]);
	pt1_Mat = Mat(pt1, CV_64F);
	// Clear the intersections vector
	Intersections.clear();

	// Process
	// 1. Determine where lines starting at Sample point intersect Line along Normal
	// Equation of line: p + r, where p is the starting point of the line, and r is the direction.
	// intersection between two lines, p+r and q+s occurs at p+tr = q+us. Need to determine either t or u.
	// from http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect :
	// u = (q − p) × r / (r × s)
	for (int i = 0; i < Samples.size(); i++) {
		// Allocate Sample point to P
		P.at<double>(0, 0) = Samples[i].x;
		P.at<double>(0, 1) = Samples[i].y;
		// Allocate Normal to R
		R.at<double>(0, 0) = Normal.at<double>(0);
		R.at<double>(0, 1) = Normal.at<double>(1);
		C = P + R;		// p + r

		// Allocate pt1 of the input line to Q
		Q.at<double>(0, 0) = (double) pt1.x;
		Q.at<double>(0, 1) = (double) pt1.y;
		// Allocate the direction of the line (pt2-pt1) to LineVec
		LineVec.at<double>(0, 0) = (double) pt2.x - pt1.x;	// q + s where pt1 is q
		LineVec.at<double>(0, 1) = (double) pt2.y - pt1.y;
		double magLineVec = norm(LineVec);
		LineVec = LineVec * (1/magLineVec);	// Normalise
		
		// Allocate LineVec to S
		S.at<double>(0, 0) = LineVec.at<double>(0, 0);
		S.at<double>(0, 1) = LineVec.at<double>(0, 1);
		S.at<double>(0, 2) = LineVec.at<double>(0, 2);

		//cout << "S " << S << endl;
		// p = p, r = normal, q = pt1, s = LineVec.
		// Convert Q, P, R, S to 3D vectors with Z = 0
		Q.at<double>(0, 2) = 0; P.at<double>(0, 2) = 0; R.at<double>(0, 2) = 0; S.at<double>(0, 2) = 0;
		// Define the 2-dimensional vector cross product v × w to be vx wy − vy wx (this is the magnitude of the 3-dimensional cross product).
		u = norm((Q - P).cross(R));		//skew(Q - P)*R;
		u = u / norm(R.cross(S));		//skew(R)*S);
		//cout << "norm(R.cross(S)): " << norm(R.cross(S)) << endl;
		//cout << "u " << u << endl;
		//cout << "S " << S << endl;
		if (u < min_u) {
			min_u = u;
		}
		// r × s ≠ 0 then intersection, residual <= 1 units to ensure that intersections are constrained to approximately parallel lines
		Intersection = Q + u*S;
		// If the residuals are less than the threshold value, an intersection has been found
		if (((norm(R.cross(S)) > 0.0) || (norm(R.cross(S)) < 0.0)) && (norm(P - Intersection) <= 5)) {
		// Intersection point at q + us
			
			/*cout << "Sample number: " << i << endl;
			
			cout << "u: " << u << endl;
			cout << "S " << S << endl;
			cout << "Q " << Q << endl;
			cout << "Intersection: " << Intersection << endl;*/
			Intersections.push_back(Point2f(Intersection.at<double>(0),Intersection.at<double>(1)));
			// Output the residuals
			residualVec.push_back(norm(P - Intersection));
			counter++;
		}
	}
	//
	// Store the residuals in residuals Mat
	residuals = Mat(residualVec);

	//cout << "Q+min_u*S " << Q + min_u*S << endl;
	// Check counter. If at least 3 samples intersect along the normal, then return true.
	if (counter >= 3) {
		//cout << "counter: " << counter << endl;
		return true;
	}

	return returnVal;
}
