/*
 * iterative_triangulation.h
 *
 *  Created on: Jan 25, 2015
 *      Author: tesfu
 */
#include<math.h>
#include<stdio.h>
#include<math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/contrib/contrib.hpp"
#ifndef ITERATIVE_TRIANGULATION_H_
#define ITERATIVE_TRIANGULATION_H_


#include "iterative_triangulation.h"
#include "opticalFlow.h"

void triangulation_construc(CvMat* first, CvMat* second,
		CvMat* previous_image_projec, CvMat* next_image_projec,
		int corner_count);

//  void  triangulation_operation( CvMat* U, CvMat* P,CvMat* U1,CvMat* P1);
void iterate_triangulation_LS(CvMat* U, CvMat* U1, CvMat* P, CvMat* P1);

#endif /* ITERATIVE_TRIANGULATION_H_ */
