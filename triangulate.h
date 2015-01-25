/*
 * triangulate.h
 *
 *  Created on: Jan 24, 2015
 *      Author: tesfu
 */

#ifndef TRIANGULATE_H_
#define TRIANGULATE_H_


#include<stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/video/video.hpp"
#include "opticalFlow.h"
#include "iterative_triangulation.h"
CvMat* camera_matrix;
CvMat* rotationInMatrix;
CvMat* translation_vector;
CvMat* distortion_coefficients;

//void printMat();
void parameters();
//void optical_flow_estimation(CvMat* rotation, CvMat* translation,
//		CvMat* distorition);



#endif /* TRIANGULATE_H_ */
