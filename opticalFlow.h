/*
 * opticalFlow.h
 *
 *  Created on: Jan 25, 2015
 *      Author: tesfu
 */

#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_

#include<math.h>
#include<stdio.h>
#include<math.h>
//#include<algorithm.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/contrib/contrib.hpp"

#define EPSILON 0.001
CvMat* X0_noniter;
CvMat* X_iter;

CvMat* rotationXX;
CvMat* rotationYY;
CvMat* rotationZZ;
CvMat* rotationZYX;
IplImage* dst;

void optical_flow_estimation(CvMat* rotation, CvMat* translation,
		CvMat* distortion);

void triangulatePoints(CvMat* first, CvMat* second, CvMat* P, CvMat* P1,
		int count);
float testTriangulation(CvMat* cloud, int c);
void cvMouse_callback(int event, int x, int y, int flags, void* params);
void euler_rotation(float thetaZ, float thetaY, float thetaX);

/*void point_cloud();
 CvMat* rotation;
 CvMat* translation;

 CvMat* translation;
 CvMat* distortion;
 */



#endif /* OPTICALFLOW_H_ */
