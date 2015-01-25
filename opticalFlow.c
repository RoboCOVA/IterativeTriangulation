/*
 * opticalFlow.c
 *
 *  Created on: Jan 25, 2015
 *      Author: tesfu
 */

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include <stdio.h>
//#include <iostream.h>
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/legacy/legacy.hpp"
////#include "opencv2/video/video.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/features2d/features2d.hpp"

//#include "opencv2/nonfree/nonfree.hpp"

#include"opticalFlow.h"
#include "triangulate.h"
#include "iterative_triangulation.h"
//#define M_PI  3.14145
//========================================================================================================================
const int MAX_CORNERS = 5000;
CvMat* first_point;
CvMat* second_point;
CvMat* projec_1;
CvMat* projec_2;
CvMat* point_cloud;
CvMat* cloudX;
CvMat* cloudY;
CvMat* cloudZ;
//==============================================================================================================
char* file1 = "shape1.bmp";
char* file2 = "shape2.bmp";
IplImage* image1;
IplImage* image2;
double alpha = 1.0;
double beta = 0;
int tABC = 0;
CvMat* optional = 0;
CvPoint* image_point1;
CvPoint image_point2;
int count_point = 10;

//==============================================================================================================

// pixel value storage function for triangulation application
/*void left_cvMouse_callback(int event, int x, int y, int flags, void* params) {

 image_point1 = (CvPoint*) malloc(count_point * sizeof(CvPoint));

 int j;

 for (j = 0; j < 10; j++) {
 if (event == CV_EVENT_LBUTTONDOWN) {
 image_point1[j].x = x;
 image_point1[j].y = y;
 }

 //printf("\nXX=%i\t\tYY = %i\n", image_point1[j].x, image_point1[j].y);

 }

 free(image_point1);
 }*/

/*void right_cvMouse_callback(int event, int x, int y, int flags, void* params) {

 image_point2 = (CvPoint*) malloc(count_point * sizeof(CvPoint));

 int j;
 for (j = 0; j < 10; j++) {

 if (event == CV_EVENT_LBUTTONDOWN) {
 image_point2[j].x = x;
 image_point2[j].y = y;
 }

 printf("\nx=%i\t\ty = %i\n", image_point2[j].x, image_point2[j].y);

 }
 free(image_point2);
 }*/
void markImage(int event, int x, int y, int flags, void *param )// void *param = NULL,
{
    if(event == CV_EVENT_LBUTTONDOWN)
    {

      // CvPoint center = Point(x,y);
        cvCircle(dst, cvPoint(x,y),2,CV_RGB(255,0,0),3 , 8, 0);
        printf("\n%i\t%i\n", x,y);
    }

}

//======================================================================================================

void optical_flow_estimation(CvMat* rotation, CvMat* translation,
		CvMat* distorition)

{

	image1 = cvLoadImage(file1, CV_LOAD_IMAGE_UNCHANGED);
	image2 = cvLoadImage(file2, CV_LOAD_IMAGE_UNCHANGED);
	cvNamedWindow("IMAGE1", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("IMAGE2", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("BOTH_VIEW",CV_WINDOW_AUTOSIZE);
	if (!image1 & !image2) {
		printf("the image is not loaded\n");

	}
	int dstWidth= image1->width + image2->width;

	int dstHeight=image1->height;
 dst=cvCreateImage(cvSize(dstWidth,dstHeight),IPL_DEPTH_8U,3);

	// Copy first image to dst
	cvSetImageROI(dst, cvRect(0, 0,image1->width,image1->height) );
	cvCopy(image1,dst,NULL);
	cvResetImageROI(dst);

	// Copy second image to dst
	cvSetImageROI(dst, cvRect(image2->width, 0,image2->width,image2->height) );
	cvCopy(image2,dst,NULL);
	cvResetImageROI(dst);

	cvSetMouseCallback("BOTH_VIEW",  markImage ,NULL);

	cvShowImage("BOTH_VIEW",dst);
	//IplImage* image1_gray = cvCreateImage(cvGetSize(image1),IPL_DEPTH_8U,1);
	//IplImage* image2_gray =cvCreateImage(cvGetSize(image2),IPL_DEPTH_8U,1);;

	CvMat* image1_gray;
	CvMat* image2_gray;

	image1_gray = cvCreateMat(image1->height, image1->width, CV_8UC1);
	image2_gray = cvCreateMat(image1_gray->rows, image1_gray->cols,
			image1_gray->type);

	CvMat* flow_image = cvCreateMat(image1_gray->rows, image1_gray->cols,
			CV_32FC2);
	CvMat* flow_cee = cvCreateMat(image1_gray->rows, image1_gray->cols,
			CV_8UC3);

//===================================================================================

	CvMat* dst_mat1 = cvCreateMat(image1->height, image1->width, CV_8UC3);
	CvMat* dst_mat2 = cvCreateMat(image2->height, image2->width, CV_8UC3);
	;
	cvGetMat(image1, dst_mat1, NULL, 0);
	cvGetMat(image2, dst_mat2, NULL, 0);

	cvCvtColor(image1, image1_gray, CV_BGR2GRAY);
	cvCvtColor(image2, image2_gray, CV_BGR2GRAY);

	//cvSetMouseCallback("IMAGE1", left_cvMouse_callback, (void*) image1_gray);
	//cvSetMouseCallback("IMAGE2", right_cvMouse_callback, (void*) image2_gray);

	if (!image1_gray && !image2_gray) {
		printf("the gray image is not loaded\n");
	}

	CvSize img_sz = cvGetSize(image1);
	int win_size = 15;

	flow_image = cvCreateMat(image1_gray->rows, image1_gray->cols, CV_32FC2); // = cvCreateImage(cvGetSize(image1_gray), IPL_DEPTH_32F,1);

	cvCalcOpticalFlowFarneback(image1_gray, image2_gray, flow_image, 0.5, 3, 12,
			3, 5, 1.2, 0);

	cvCvtColor(image1_gray, flow_cee, CV_GRAY2BGR);
	//drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
	// cvShowImage("flow", cflow);
	int x, y;
	int step = 6;
	double scale = 1.5;
	(void) scale;
	CvScalar color = CV_RGB(0,255,0);
	int nana = flow_cee->rows * flow_cee->cols;

	CvMat* left = cvCreateMat(flow_cee->rows * flow_cee->cols, 1, CV_32FC2);
	CvMat* right = cvCreateMat(flow_cee->rows * flow_cee->cols, 1, CV_32FC2);
	printf("\n THE CORNER\n");

	int n =0;
	for (y = 0; y < flow_cee->rows; y += step){
		for (x = 0; x < flow_cee->cols; x += step) {

			CvPoint2D32f fxy = CV_MAT_ELEM(*flow_image, CvPoint2D32f, y,x);
			if (fabs(fxy.x < 0.1) && fabs(fxy.y < 0.1))
				continue;



			left->data.fl[y * 2] = x;
			left->data.fl[y * 2 + 1] = y;
			right->data.fl[y * 2] = x + fxy.x;
			right->data.fl[y * 2 + 1] = y + fxy.y;
		//	printf("\n%f\t%f\t%f\t%f\nleft->data.fl[y * 2],left->data.fl[y * 2 +1],right->data.fl[y * 2],right->data.fl[y * 2 +1]);

			cvLine(flow_cee, cvPoint(x, y),
					cvPoint(cvRound(x + fxy.x), cvRound(y + fxy.y)), color, 1,
					8, 0);
			cvCircle(flow_cee, cvPoint(x, y), 2, color, -1, 8, 0);

    n++;
		}
	}

	//CvMat* leftF = cvCreateMat(flow_cee->rows * flow_cee->cols, 1, CV_32FC2);
	//CvMat* right = cvCreateMat(flow_cee->rows * flow_cee->cols, 1, CV_32FC2);
//===============================================================================================

	IplImage* image_result;
	// image_result = cvLoadImage("this.jpg", CV_LOAD_IMAGE_UNCHANGED);
	image_result = cvCloneImage(image1);

	IplImage* eig_image = cvCreateImage(img_sz, IPL_DEPTH_32F, 1);
	IplImage* tmp_image = cvCreateImage(img_sz, IPL_DEPTH_32F, 1);
//================================================================================================
	int corner_count1 = MAX_CORNERS;
	// CvPoint2D32f   *cornersB[ MAX_CORNERS ];
	// CvPoint2D32f  *cornersA[MAX_CORNERS];
	/* int t;
	 for (t=0;t<MAX_CORNERS;t++)
	 {*/

	CvPoint2D32f *cornersA = (CvPoint2D32f*) malloc(
			MAX_CORNERS * sizeof(CvPoint2D32f));
	CvPoint2D32f *cornersB = (CvPoint2D32f*) malloc(
			MAX_CORNERS * sizeof(CvPoint2D32f));

	//=============================================================================================
	cvGoodFeaturesToTrack(image1_gray, eig_image, tmp_image, cornersA,
			&corner_count1, 0.01, 5.0, 0, 3, 0, 0.04);

	cvFindCornerSubPix(image1_gray, cornersA, corner_count1,
			cvSize(win_size, win_size), cvSize(-1, -1),
			cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	//=================================================================================================
	// feature found calling

	char features_found[MAX_CORNERS];
	float feature_errors[MAX_CORNERS];

	CvSize pyr_sz = cvSize(image1->width + 8, image2->height / 3);
	IplImage* pyrA = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);
	IplImage* pyrB = cvCreateImage(pyr_sz, IPL_DEPTH_32F, 1);

	//==================================================================================================

	cvCalcOpticalFlowPyrLK(image1, image2, pyrA, pyrB, cornersA, cornersB,
			corner_count1, cvSize(win_size, win_size), 5, features_found,
			feature_errors,
			cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3), 0);

//======================================================================================================
	int i;
	//  printf("\n THE CORNERS\n");
	for (i = 0; i < corner_count1; i++) {
		if (features_found[i] == 0 || feature_errors[i] > 30) {

			//printf("ERROR IS %f\n", feature_errors[i]);

			continue;
		} else {

			// printf("\n%f\t\t%f\t\t%f\t\t%f\n", cornersA[i].x,cornersA[i].y, cornersB[i].x, cornersB[i].y);

		}
//=======================================================================================================

		//  printf("PRINT THE RESULT\n");
		CvPoint p0 = cvPoint(cvRound(cornersA[i].x), cvRound(cornersA[i].y));
		CvPoint p1 = cvPoint(cvRound(cornersB[i].x), cvRound(cornersB[i].y));
		int lineType = 8;
		cvLine(image_result, p0, p1, CV_RGB(255,0,0), 1, lineType, 0);

		// printf("\n PRINT THE VALUE OF CORNERS \n");
	}

	cvNamedWindow("OPTICAL_FLOW", CV_WINDOW_AUTOSIZE);
	cvShowImage("IMAGE1", image1);
	cvShowImage("IMAGE2", image2);
	cvShowImage("OPTICAL_FLOW", flow_cee); //flow_cee);
	//cvShowImage("IMAGE RESULT ",image_result);


	int c = cvWaitKey(100000);
	if (c == 'p') {
		c = 0;
		while (c != 'p' && c != 27) {
			c = cvWaitKey(2500);
		}
	}
	if (c == 27) {
		printf("done");
	}
	cvReleaseImage(&dst);
	cvReleaseMat(&flow_cee);
	cvReleaseMat(&flow_image);
	cvReleaseImage(&eig_image);
	cvReleaseImage(&tmp_image);
	cvReleaseMat(&image1_gray);
	cvReleaseImage(&pyrA);
	cvReleaseImage(&pyrB);
	cvReleaseMat(&image2_gray);
	cvReleaseImage(&image1);
	cvReleaseImage(&image2);
	cvDestroyWindow("IMAGE1");
	cvDestroyWindow("IMAGE2");
	cvDestroyWindow("OPTICAL_FLOW");


	//=========================================================================================
	//un-distort images based on camera matrix and distortion

	first_point = cvCreateMat(corner_count1, 1, CV_32FC2);
	second_point = cvCreateMat(corner_count1, 1, CV_32FC2);
	int tt;
	for (tt = 0; tt < corner_count1; tt++) {
		first_point->data.fl[tt * 2] = cornersA[tt].x;
		first_point->data.fl[tt * 2 + 1] = cornersA[tt].y;

		second_point->data.fl[tt * 2] = cornersB[tt].x;
		second_point->data.fl[tt * 2 + 1] = cornersB[tt].y;
		printf("\n%f\t%f\t%f\t%f\n", first_point->data.fl[tt * 2],first_point->data.fl[tt * 2 + 1],
		 second_point->data.fl[tt * 2] , second_point->data.fl[tt * 2 + 1] );

	}

//==================================================================================================
	// undistort the corners
	// ideal point coordinate from observed point coordinate
	// int p_size = corner_count1;
	int p_size = corner_count1;
	printf("\np_size = %i\n", p_size);

	//  CvMat* undist_first = cvCreateMat(p_size, 1,CV_32FC2);
	// CvMat* undist_second = cvCreateMat(p_size, 1,CV_32FC2);
	CvMat* undist_first = cvCreateMat(p_size, 1, CV_32FC2);
	CvMat* undist_second = cvCreateMat(p_size, 1, CV_32FC2);

	cvUndistortPoints(first_point, undist_first, camera_matrix,
			distortion_coefficients, 0, 0);
	cvUndistortPoints(second_point, undist_second, camera_matrix,
			distortion_coefficients, 0, 0);

	/*cvUndistortPoints(left, undist_first, camera_matrix,
	 distortion_coefficients, 0, 0);
	 cvUndistortPoints(right, undist_second, camera_matrix,
	 distortion_coefficients, 0, 0);*/

	CvMat* undist_first_non = cvCreateMat(p_size, 2, CV_32FC1);
	CvMat* undist_second_non = cvCreateMat(p_size, 2, CV_32FC1);

	cvReshape(undist_first, undist_first_non, 1, p_size);
	cvReshape(undist_second, undist_second_non, 1, p_size);

//============================================================================================================
	//find the non ideal pixel coordinate
	/*CvMat* ideal_one = cvCreateMat(p_size, 3, CV_32FC1);
	CvMat* ideal_second = cvCreateMat(p_size, 3, CV_32FC1);

	int ad;

	for (ad = 0; ad < p_size; ad++)

	{
		ideal_one->data.fl[ad * 3] = (float)cvmGet(undist_first_non, ad, 0);
		ideal_one->data.fl[ad * 3 + 1] = (float)cvmGet(undist_first_non, ad, 1);
		ideal_one->data.fl[ad * 3 + 2] = 1;

		ideal_second->data.fl[ad * 3] = (float)cvmGet(undist_second_non, ad, 0);
		ideal_second->data.fl[ad * 3 + 1] = (float)cvmGet(undist_second_non, ad, 1);
		ideal_second->data.fl[ad * 3 + 2] = 1;

	}

	CvMat* ideal_trans1 = cvCreateMat(3, p_size, CV_32FC1);
	CvMat* ideal_trans2 = cvCreateMat(3, p_size, CV_32FC1);

	cvTranspose(ideal_one, ideal_trans1);
	cvTranspose(ideal_second, ideal_trans2);

	CvMat* undist_recovered1 = cvCreateMat(3, p_size, CV_32FC1);
	CvMat* undist_recovered2 = cvCreateMat(3, p_size, CV_32FC1);

	cvGEMM(camera_matrix, ideal_trans1, alpha, optional, beta,
			undist_recovered1, tABC);
	cvGEMM(camera_matrix, ideal_trans2, alpha, optional, beta,
			undist_recovered2, tABC);

	CvMat* undist_trans1 = cvCreateMat(p_size, 3, CV_32FC1);
	CvMat* undist_trans2 = cvCreateMat(p_size, 3, CV_32FC1);

	cvTranspose(undist_recovered1, undist_trans1);
	cvTranspose(undist_recovered2, undist_trans2);

	CvMat* undist_new1 = cvCreateMat(p_size, 2, CV_32FC1);
	CvMat* undist_new2 = cvCreateMat(p_size, 2, CV_32FC1);
	int dd;
	for (dd = 0; dd < p_size; dd++) {
		undist_new1->data.fl[dd * 2] = (float)cvmGet(undist_trans1, dd, 0);
		undist_new1->data.fl[dd * 2 + 1] = (float)cvmGet(undist_trans1, dd, 1);

		undist_new2->data.fl[dd * 2] = (float)cvmGet(undist_trans2, dd, 0);
		undist_new2->data.fl[dd * 2 + 1] =(float)cvmGet(undist_trans2, dd, 1);

	}

	CvMat* undist_chan12 = cvCreateMat(p_size, 1, CV_32FC2);
	CvMat* undist_chan22 = cvCreateMat(p_size, 1, CV_32FC2);

	cvReshape(undist_new1, undist_chan12, 2, p_size);
	cvReshape(undist_new2, undist_chan22, 2, p_size);
	int tta;
	for (tta = 0; tta < p_size; tta++) {

		printf("\n%f\t%f\n", undist_chan12->data.fl[tta * 2],
				undist_chan12->data.fl[tta * 2 + 1]);

	}*/
	/* int test;
	 printf("\n THE NON-IDEAL UNDISTORTED POINT COORDINATES\n");


	 for (test=0;test<p_size; test++)
	 {

	 printf("\n%f\t%f\t%f\t%f\n",undist_new1->data.fl[test*2], undist_new1->data.fl[test*2 +1],
	 undist_new2->data.fl[test*2],undist_new2->data.fl[test*2+1]);

	 }*/

//=======================================================================================================
	int method = CV_FM_RANSAC;

	CvMat* essential;
	CvMat* fundamental_matrix;
	CvMat* semi_esse;
	/*double minv;
	 double maxval;
	 CvPoint tre;
	 CvPoint teee;
	 cvMinMaxLoc(undist_chan12, &maxval, &minv, &tre, &teee, 0);*/

	fundamental_matrix = cvCreateMat(3, 3, CV_32FC1);
	semi_esse = cvCreateMat(3, 3, CV_32FC1);
	essential = cvCreateMat(3, 3, CV_32FC1);

	int check = cvFindFundamentalMat(undist_first, undist_second,
			fundamental_matrix, method, 1.0, 0.99, NULL);
	if (check == 0) {
		printf("\n the configuration is degenerate\n");

	}
	//=====================================================================================================

	CvMat* TT = cvCreateMat(2, p_size, CV_32FC1);
	CvMat* YY = cvCreateMat(2, p_size, CV_32FC1);
	cvTranspose(undist_first_non, TT);
	cvTranspose(undist_second_non, YY);

	CvMat* corr1 = cvCreateMat(1, p_size, CV_32FC2);
	CvMat* corr2 = cvCreateMat(1, p_size, CV_32FC2);

	cvReshape(TT, corr1, 2, 1);
	cvReshape(YY, corr2, 2, 1);

	CvMat* correct_points1 = cvCreateMat(1,p_size, CV_32FC2);
	CvMat* correct_points2 = cvCreateMat(1, p_size, CV_32FC2);

	//=========================================================================================================

	cvCorrectMatches(fundamental_matrix,corr1,corr2, correct_points1,
			correct_points2);

  CvMat* FF = cvCreateMat(2, p_size, CV_32FC1);
  CvMat* GG = cvCreateMat(2, p_size, CV_32FC1);
	CvMat* HH = cvCreateMat( p_size,2, CV_32FC1);
	CvMat* II = cvCreateMat( p_size,2,CV_32FC1);

	cvReshape(correct_points1, FF, 1, 2);
	cvReshape(correct_points2, GG, 1, 2);
	cvTranspose(FF, HH);
	cvTranspose(GG, II);
	CvMat* HH_new = cvCreateMat(p_size, 1, CV_32FC2);
	CvMat* II_new = cvCreateMat(p_size, 1, CV_32FC2);
	cvReshape(HH, HH_new, 2, p_size);
	cvReshape(II, II_new, 2, p_size);
	double max;
	double min;
	CvPoint maxp;
	CvPoint minp;

	cvMinMaxLoc(HH, &max, &min, &maxp, &minp, 0);

	int check2 = cvFindFundamentalMat(HH_new, II_new, fundamental_matrix,
			method, 0.006 * max, 0.99, 0);

	if (check2 == 0)
		printf("\n wrong fundamental mat\n");

	//========================================================================================================

	CvMat* cam_transpose = cvCreateMat(3, 3, CV_32FC1);

	cvTranspose(camera_matrix, cam_transpose);
	cvGEMM(fundamental_matrix, camera_matrix, alpha, optional, beta, semi_esse,
			tABC);
	cvGEMM(cam_transpose, semi_esse, alpha, optional, beta, essential, tABC);

	//========================================================================================================
	int yz;
	printf("\n FUNDAMETAL MATRIX\n");
	for (yz = 0; yz < 3; yz++) {

		printf("\n %f\t\t%f\t\t%f\n", fundamental_matrix->data.fl[yz * 3],
				fundamental_matrix->data.fl[yz * 3 + 1],
				fundamental_matrix->data.fl[yz * 3 + 2]);
	}
	printf("\n THE ESSESNTIAL MATRIX\n");
	int te;

	for (te = 0; te < 3; te++) {

		printf("\n %f\t\t%f\t\t%f\n", essential->data.fl[te * 3],
				essential->data.fl[te * 3 + 1], essential->data.fl[te * 3 + 2]);
	}
	/*int tte;
	 printf("\n PRINT THE CORRECT MATCH\n");
	 for(tte=0;tte<corner_count;tte++)
	 {
	 printf("\n%f\t\t%f\t\t%f\t\t%f\n", correct_points1->data.fl[tte*2],correct_points1->data.fl[tte*2+1],
	 correct_points2->data.fl[tte*2],correct_points2->data.fl[tte*2+1]);
	 }*/

//=============================================================================================================
	CvMat* U;
	CvMat* V;
	CvMat* W;
	CvMat* V_trans;
	CvMat* W_inv;

	// cvCalcOpticalFlowFarneback()

	V_trans = cvCreateMat(3, 3, CV_32FC1);

	//int flags = CV_SVD_U_T;
	U = cvCreateMat(3, 3, CV_32FC1);
	V = cvCreateMat(3, 3, CV_32FC1);
	W = cvCreateMat(3, 3, CV_32FC1); // singular value decomposition
	W_inv = cvCreateMat(3, 3, CV_32FC1);
	CvMat* W_ske = cvCreateMat(3, 3, CV_32FC1);
	cvSVD(essential, W, U, V, CV_SVD_MODIFY_A); // essential = UWV'

	cvmSet(W_ske, 0, 0, 0);
	cvmSet(W_ske, 0, 1, -1);
	cvmSet(W_ske, 0, 2, 0);
	cvmSet(W_ske, 1, 0, 1);
	cvmSet(W_ske, 1, 1, 0);
	cvmSet(W_ske, 1, 2, 0);
	cvmSet(W_ske, 2, 0, 0);
	cvmSet(W_ske, 2, 1, 0);
	cvmSet(W_ske, 2, 2, 1);

	cvmSet(W_inv, 0, 0, 0);
	cvmSet(W_inv, 0, 1, 1);
	cvmSet(W_inv, 0, 2, 0);
	cvmSet(W_inv, 1, 0, -1);
	cvmSet(W_inv, 1, 1, 0);
	cvmSet(W_inv, 1, 2, 0);
	cvmSet(W_inv, 2, 0, 0);
	cvmSet(W_inv, 2, 1, 0);
	cvmSet(W_inv, 2, 2, 1);
//===========================================================================================================
	//printf("\ndeterminant of essential and fundamental matrix\n");
	//printf("%f\t\t%f\n", cvDet(fundamental_matrix),cvDet( essential));

//============================================================================================================
	// find projection matrix
	CvMat* rot_1;
	CvMat* rot_2;
	CvMat* trans_1;
	CvMat* trans_2;

	rot_1 = cvCreateMat(3, 3, CV_32FC1);
	rot_2 = cvCreateMat(3, 3, CV_32FC1);
	trans_1 = cvCreateMat(3, 1, CV_32FC1);
	trans_2 = cvCreateMat(3, 1, CV_32FC1);

	projec_1 = cvCreateMat(3, 4, CV_32FC1);
	projec_2 = cvCreateMat(3, 4, CV_32FC1);
//===========================================================================================================
	CvMat* rot_semi1 = cvCreateMat(3, 3, CV_32FC1);
	CvMat* rot_semi2 = cvCreateMat(3, 3, CV_32FC1);
	cvmSet(projec_1, 0, 0, 1);
	cvmSet(projec_1, 0, 1, 0);
	cvmSet(projec_1, 0, 2, 0);
	cvmSet(projec_1, 1, 0, 0);
	cvmSet(projec_1, 1, 1, 1);
	cvmSet(projec_1, 1, 2, 0);
	cvmSet(projec_1, 2, 0, 0);
	cvmSet(projec_1, 2, 1, 0);
	cvmSet(projec_1, 2, 2, 1);
	cvmSet(projec_1, 0, 3, 0);
	cvmSet(projec_1, 1, 3, 0);
	cvmSet(projec_1, 2, 3, 0);
	cvTranspose(V, V_trans);

	cvGEMM(U, W_ske, alpha, optional, beta, rot_semi1, tABC);
	cvGEMM(rot_semi1, V_trans, alpha, optional, beta, rot_1, tABC);

	printf("\n the determinant of rot\n %f\n", cvDet(rot_1));

	cvGEMM(U, W_inv, alpha, optional, beta, rot_semi2, tABC);
	cvGEMM(rot_semi2, V_trans, alpha, optional, beta, rot_2, tABC);

/////================================refine essential matrix==================================================

	if (cvDet(rot_1) + 1.0 < 0.00001) {
		printf("the determinant of rot_1 is = 1 and flip E sign");

		CvMat* ess_new = cvCreateMat(3, 3, CV_32FC1);
		int g;
		for (g = 0; g < 9; g++)
		{
			ess_new->data.fl[g] = -essential->data.fl[g];

		   }
		cvSVD(ess_new, W, U, V, CV_SVD_MODIFY_A); // essential = UWV'

		cvGEMM(U, W_ske, alpha, optional, beta, rot_semi1, tABC);
		cvGEMM(rot_semi1, V_trans, alpha, optional, beta, rot_1, tABC);

		printf("\n the determinant of rot\n %f\n", cvDet(rot_1));

		cvGEMM(U, W_inv, alpha, optional, beta, rot_semi2, tABC);
		cvGEMM(rot_semi2, V_trans, alpha, optional, beta, rot_2, tABC);
	}

	trans_1->data.fl[0] = cvmGet(U, 0, 2);
	trans_1->data.fl[1] = cvmGet(U, 1, 2);
	trans_1->data.fl[2] = cvmGet(U, 2, 2);

	trans_2->data.fl[0] = -cvmGet(U, 0, 2);
	trans_2->data.fl[1] = -cvmGet(U, 1, 2);
	trans_2->data.fl[2] = -cvmGet(U, 2, 2);

	projec_2->data.fl[0] = cvmGet(rot_1, 0, 0);
	projec_2->data.fl[1] = cvmGet(rot_1, 0, 1);
	projec_2->data.fl[2] = cvmGet(rot_1, 0, 2);
	projec_2->data.fl[3] = cvmGet(trans_1, 0, 0);
	projec_2->data.fl[4] = cvmGet(rot_1, 1, 0);
	projec_2->data.fl[5] = cvmGet(rot_1, 1, 1);
	projec_2->data.fl[6] = cvmGet(rot_1, 1, 2);
	projec_2->data.fl[7] = cvmGet(trans_1, 1, 0);
	projec_2->data.fl[8] = cvmGet(rot_1, 2, 0);
	projec_2->data.fl[9] = cvmGet(rot_1, 2, 1);
	projec_2->data.fl[10] = cvmGet(rot_1, 2, 2);
	projec_2->data.fl[11] = cvmGet(trans_1, 2, 0);
	int afaf, atat;

	for (afaf = 0; afaf < 3; afaf++) {
		printf("\n %f\t%f\t%f\t%f\n", projec_1->data.fl[afaf * 4],
				projec_1->data.fl[afaf * 4 + 1],
				projec_1->data.fl[afaf * 4 + 2],
				projec_1->data.fl[afaf * 4 + 3]);

	}

	for (atat = 0; atat < 3; atat++) {
		printf("\n %f\t%f\t%f\t%f\n", projec_2->data.fl[atat * 4],
				projec_2->data.fl[atat * 4 + 1],
				projec_2->data.fl[atat * 4 + 2],
				projec_2->data.fl[atat * 4 + 3]);

	}

	//=============================================================================================================

	// check the consistency of the solution
	float percent1, percent2;
	float mean_error_value1, mean_error_value2;

	//triangulatePoints(HH, II, projec_1, projec_2, p_size);

	//printf("\nthis\n");
	//percent1 = testTriangulation(point_cloud, p_size);
  // triangulatePoints(II, HH, projec_2, projec_1, p_size);
	//  percent2 = testTriangulation(point_cloud, p_size);

	/*printf(
	 "\n THE FIRST CONFIGURATION\n  mean erro1\tmean error2\tpercent1\tpercent2\n %f\t%f\t%f\t%f\n",
	 mean_error_value1, mean_error_value2, percent1, percent2);*/

	int a =8;

	//cvSave("output1.txt", point_cloud, NULL, NULL, attributes);
	if (a>3) {
		projec_2->data.fl[3] = cvmGet(trans_2, 0, 0);
		projec_2->data.fl[7] = cvmGet(trans_2, 1, 0);
		projec_2->data.fl[11] = cvmGet(trans_2, 2, 0);

		triangulatePoints(HH, II, projec_1, projec_2,
			p_size);
		/*percent1 = testTriangulation(point_cloud, p_size);
		 triangulatePoints(II, HH, projec_2, projec_1,
		 p_size);
		 percent2 = testTriangulation(point_cloud, p_size);*/

//		printf(
//				"\n THE SECOND CONFIGURATION\n  mean erro1\tmean error2\tpercent1\tpercent2\n %f\t%f\t%f\t%f\n",
//				mean_error_value1, mean_error_value2, percent1, percent2);
////

		if (percent1 < 75 || percent2 < 75 || mean_error_value1 > 70
				|| mean_error_value2 > 70) {

			projec_2->data.fl[0] = cvmGet(rot_2, 0, 0);
			projec_2->data.fl[1] = cvmGet(rot_2, 0, 1);
			projec_2->data.fl[2] = cvmGet(rot_2, 0, 2);
			projec_2->data.fl[3] = cvmGet(trans_1, 0, 0);
			projec_2->data.fl[4] = cvmGet(rot_2, 1, 0);
			projec_2->data.fl[5] = cvmGet(rot_2, 1, 1);
			projec_2->data.fl[6] = cvmGet(rot_2, 1, 2);
			projec_2->data.fl[7] = cvmGet(trans_1, 1, 0);
			projec_2->data.fl[8] = cvmGet(rot_2, 2, 0);
			projec_2->data.fl[9] = cvmGet(rot_2, 2, 1);
			projec_2->data.fl[10] = cvmGet(rot_2, 2, 2);
			projec_2->data.fl[11] = cvmGet(trans_1, 2, 0);

			/* triangulatePoints(HH, II, projec_1, projec_2,
			 p_size);
			 percent1 = testTriangulation(point_cloud, p_size);
			 triangulatePoints(II, HH, projec_2, projec_1,
			 p_size);
			 percent2 = testTriangulation(point_cloud, p_size);*/
			printf(
					"\n THE THIRD CONFIGURATION\n  mean erro1\tmean error2\tpercent1\tpercent2\n %f\t%f\t%f\t%f\n",
					mean_error_value1, mean_error_value2, percent1, percent2);

			if (percent1 < 75 || percent2 < 75 || mean_error_value1 > 70
					|| mean_error_value2 > 70) {

				projec_2->data.fl[3] = cvmGet(trans_2, 0, 0);
				projec_2->data.fl[7] = cvmGet(trans_2, 1, 0);
				projec_2->data.fl[11] = cvmGet(trans_2, 2, 0);

				/*triangulatePoints(HH, II, projec_1,
				 projec_2, p_size);
				 percent1 = testTriangulation(point_cloud, p_size);
				 triangulatePoints(II, HH, projec_2,
				 projec_1, p_size);
				 percent2 = testTriangulation(point_cloud, p_size);*/
				printf(
						"\n THE SECOND CONFIGURATION\n  mean erro1\tmean error2\tpercent1\tpercent2\n %f\t%f\t%f\t%f\n",
						mean_error_value1, mean_error_value2, percent1,
						percent2);

				if (percent1 < 75 || percent2 < 75 || mean_error_value1 > 70
						|| mean_error_value2 > 70) {
					printf("\n the result is fake\n");

				}

			}
		}
	}

//==================================================================================================================
	int tst;

	printf("\n THE PROJECTION MATRIX\n");
	for (tst = 0; tst < 3; tst++) {

		printf("\n %f\t\t%f\t\t%f\t\t%f\n", projec_1->data.fl[tst * 4],
				projec_1->data.fl[tst * 4 + 1], projec_1->data.fl[tst * 4 + 2],
				projec_1->data.fl[tst * 4 + 3]);
	}
	int tft;
	for (tft = 0; tft < 3; tft++) {

		printf("\n %f\t\t%f\t\t%f\t\t%f\n", projec_2->data.fl[tft * 4],
				projec_2->data.fl[tft * 4 + 1], projec_2->data.fl[tft * 4 + 2],
				projec_2->data.fl[tft * 4 + 3]);
	}

	cvReleaseMat(&first_point);
	cvReleaseMat(&rot_1);
	cvReleaseMat(&second_point);
	cvReleaseMat(&rot_2);
	cvReleaseMat(&projec_1);
	cvReleaseMat(&essential);
	cvReleaseMat(&projec_2);
	cvReleaseMat(&fundamental_matrix);
	cvReleaseMat(&U);
	cvReleaseMat(&semi_esse);
	cvReleaseMat(&V);
	cvReleaseMat(&rot_semi1);
	cvReleaseMat(&W);
	cvReleaseMat(&rot_semi2);
	cvReleaseMat(&trans_1);
	cvReleaseMat(&projec_1);
	cvReleaseMat(&trans_2);
	cvReleaseMat(&projec_2);
	cvReleaseMat(&HH);
	cvReleaseMat(&II);
	cvReleaseMat(&GG);
	cvReleaseMat(&FF);
 /*   cvReleaseMat(&ideal_one);
	cvReleaseMat(&undist_new1);
	cvReleaseMat(&undist_new1);
	cvReleaseMat(&undist_chan12);
	cvReleaseMat(&undist_chan22);*/
	cvReleaseMat(&undist_second);
	cvReleaseMat(&undist_first_non);
	cvReleaseMat(&undist_second_non);
	/*cvReleaseMat(&ideal_one);
	cvReleaseMat(&ideal_second);
	cvReleaseMat(&ideal_trans1);
	cvReleaseMat(&ideal_trans2);
	cvReleaseMat(&undist_recovered1);
	cvReleaseMat(&undist_recovered2);*/
	cvReleaseMat(&HH_new);
	cvReleaseMat(&II_new);
	cvReleaseMat(&undist_first);
	cvReleaseMat(&left);
	cvReleaseMat(&right);
	cvReleaseMat(&undist_first);
//===============================================================================================================
//free(cornersA);free(cornersB);
}
//================================================================================================================

//==============================================================================================================

void triangulatePoints(CvMat* first, CvMat* second, CvMat* P, CvMat* P1,int countpoint)

{

       rotationZZ = cvCreateMat(3,3, CV_32FC1);
       rotationYY = cvCreateMat(3,3,CV_32FC1);
       rotationXX = cvCreateMat(3,3,CV_32FC1);
       rotationZYX = cvCreateMat(3,3,CV_32FC1);



	float mm[] = { 666.7, -217.6, 506.3, -168.8, 28.5, 166.7 };
	float mmmt[] = { 662.5, -246.6, 497.0, -168.3, 26.1, 171.4 };




	/*int beda;
	 float rototrans[6]=0;
	 for(beda = 0 ; beda <6;beda++)
	 {
	 rototrans[beda] = mmmt[beda]- mm[beda];

	 }*/
	float matmat[] = { 1.841, 2.4433, 217.584, 0, 0, 180 }; //EZ, EY,EX -FROM CALIBRATION OF ROBOT

	CvMat* jacobian = 0;

	CvMat* relative_rot = cvCreateMat(3, 1, CV_32FC1);
	CvMat* rotomat = cvCreateMat(3, 3, CV_32FC1);

	cvmSet(relative_rot, 0, 0, (mmmt[5] - mm[5]));
	cvmSet(relative_rot, 1, 0, (mmmt[4] - mm[4]));
	cvmSet(relative_rot, 2, 0, (mmmt[3] - mm[3]));

	cvRodrigues2(relative_rot, rotomat, jacobian);

	euler_rotation((mmmt[3] - mm[3]) *M_PI/180, (mmmt[4] - mm[4])* M_PI/180, (mmmt[5] - mm[5]) *M_PI/180);
	CvMat* promat2 = cvCreateMat(3, 4, CV_32FC1);
		int dush, shu;
		for (dush = 0; dush < 3; dush++) {
			for (shu = 0; shu < 3; shu++) {
				cvmSet(promat2, dush, shu, cvmGet(rotationZYX, dush, shu));

			}

		}
		cvmSet(promat2, 0, 3, (mmmt[0] - mm[0]));
		cvmSet(promat2, 1, 3, (mmmt[1] - mm[1]));
		cvmSet(promat2, 2, 3, (mmmt[2] - mm[2]));






	CvMat* rotation_n1 = cvCreateMat(3, 1, CV_32FC1);
	CvMat* rotation_mat1 = cvCreateMat(3, 3, CV_32FC1);

	CvMat* rotto = cvCreateMat(3, 1, CV_32FC1); //
	cvmSet(rotto, 0, 0, 0);
	cvmSet(rotto, 1, 0, 0);
	cvmSet(rotto, 2, 0, 180.0 * M_PI / 180);
	CvMat* relateROTO = cvCreateMat(3, 3, CV_32FC1);

	cvRodrigues2(rotto, relateROTO, jacobian);

//  AGAIN CONVERT EULER ANGLE TO ROTATION MATRIX


	euler_rotation(180.0 *M_PI/180, 0 , 0);
	 printf ("\nROTATION ZYX and and \n");
	    int nnnn;
	    for (nnnn = 0; nnnn<3;nnnn++)
	    {
	    	printf("\n%f\t%f\t%f\n",rotationZYX->data.fl[nnnn*3],rotationZYX->data.fl[nnnn*3 +1],
	    			rotationZYX->data.fl[nnnn*3 +2]);

	    }

	CvMat* tranta = cvCreateMat(4, 4, CV_32FC1);
	int ata, ataya;

	for (ata = 0; ata < 3; ata++) {
		for (ataya = 0; ataya < 3; ataya++) {
			cvmSet(tranta, ata, ataya, cvmGet(rotationZYX, ata, ataya));

		}
	}

	cvmSet(tranta, 0, 3, 1.841);
	cvmSet(tranta, 1, 3, 2.4433);
	cvmSet(tranta, 2, 3, 217.584);
	cvmSet(tranta, 3,0,0);
	cvmSet(tranta, 3,1,0);
	cvmSet(tranta, 3,2,0);
	cvmSet(tranta, 3,3,1);

	CvMat* rotation_n2 = cvCreateMat(3, 1, CV_32FC1);
	CvMat* rotation_mat2 = cvCreateMat(3, 3, CV_32FC1);




	cvmSet(rotation_n1, 0, 0, -168.8 * M_PI / 180);
	cvmSet(rotation_n1, 1, 0, 28.5 * M_PI / 180);
	cvmSet(rotation_n1, 2, 0, 166.7 * M_PI / 180);

	cvmSet(rotation_n2, 0, 0, -168.3 * M_PI / 180); // EZ
	cvmSet(rotation_n2, 1, 0, 26.1 * M_PI / 180); //EY
	cvmSet(rotation_n2, 2, 0, 171.4 * M_PI / 180); //EX

//============================input

	float wha[] = { 0.080403, 0.994247, -0.070769, 523.635254, 0.655698,
			0.000717, 0.755023, -38.123413, 0.750730, -0.107109, -0.651868,
			178.615097, 0.000, 0.000, 0.000, 1.000 };


	cvRodrigues2(rotation_n1, rotation_mat1, jacobian);
	float deterOfR = cvDet(rotation_mat1);
	cvRodrigues2(rotation_n2, rotation_mat2, jacobian);
	float deterOfR1 = cvDet(rotation_mat2);
	printf("\nthe determinant of rotation matrix is = %f\t%f\n  ", deterOfR,
			deterOfR1);

	CvMat* camera_end1 = cvCreateMat(4, 4, CV_32FC1);
	CvMat* camera_end2 = cvCreateMat(4, 4, CV_32FC1);

	int fu;
	for (fu = 0; fu < 4; fu++) {

		camera_end1->data.fl[fu * 4] = wha[fu * 4];
		camera_end1->data.fl[fu * 4 + 1] = wha[fu * 4 + 1];
		camera_end1->data.fl[fu * 4 + 2] = wha[fu * 4 + 2];
		camera_end1->data.fl[fu * 4 + 3] = wha[fu * 4 + 3];
		printf("\n%f\t%f\t%f\t%f", camera_end1->data.fl[fu * 4],
				camera_end1->data.fl[fu * 4 + 1],
				camera_end1->data.fl[fu * 4 + 2],
				camera_end1->data.fl[fu * 4 + 3]);

	}
	CvMat* projection1 = cvCreateMat(3, 4, CV_32FC1);
	CvMat* projection2 = cvCreateMat(3, 4, CV_32FC1);
	CvMat* TT1 = cvCreateMat(4, 4, CV_32FC1);
	CvMat* TT2 = cvCreateMat(4, 4, CV_32FC1);

	int ahah;
	for (ahah = 0; ahah < 12; ahah++) {

		projection1->data.fl[ahah] = camera_end1->data.fl[ahah];
	}

	//CALL THE EULER ROTATION FUNCTION TO FIND ROTATION MATRIX


	euler_rotation(-168.8 * M_PI / 180,  28.5 * M_PI / 180,  166.7 * M_PI / 180);

    printf ("\nROTATION ZYX \n");
    int nn;
    for (nn = 0; nn<3;nn++)
    {
    	printf("\n%f\t%f\t%f\n",rotationZYX->data.fl[nn*3],rotationZYX->data.fl[nn*3 +1], rotationZYX->data.fl[nn*3 +2]);

    }

	int gur;
	int inta;
	for (gur = 0; gur < 3; gur++) {
		for (inta = 0; inta < 3; inta++) {

			cvmSet(TT1, gur, inta, cvmGet(rotationZYX, gur, inta));


		}
	}
	//AGAIN PASS THE EULER ANGLES TO THE ROUTINE EULER_ROTATION()


	euler_rotation(-168.3 * M_PI / 180,  26.1 * M_PI / 180,  171.4 * M_PI / 180);
	 printf ("\nROTATION ZYX AND \n");
	    int nnn;
	    for (nnn = 0; nnn<3;nnn++)
	    {
	    	printf("\n%f\t%f\t%f\n",rotationZYX->data.fl[nnn*3],rotationZYX->data.fl[nnn*3 +1], rotationZYX->data.fl[nnn*3 +2]);

	    }

	int gurb;
		int intal;
		for (gurb = 0; gurb < 3; gurb++) {
			for (intal = 0; intal < 3; intal++) {



				cvmSet(TT2, gurb, intal, cvmGet(rotationZYX, gurb, intal));

			}
		}

	//FILL IN ROWS
	cvmSet(TT1, 3, 0, 0);
	cvmSet(TT1, 3, 1, 0);
	cvmSet(TT1, 3, 2, 0);

	cvmSet(TT2, 3, 0, 0);
	cvmSet(TT2, 3, 1, 0);
	cvmSet(TT2, 3, 2, 0);

	//FILL IN COLUMNS -TRANSLATION VALUES

	cvmSet(TT1, 0, 3, 666.7);
	cvmSet(TT1, 1, 3, -217.6);
	cvmSet(TT1, 2, 3, 506.3);
	cvmSet(TT2, 0, 3, 662.5);
	cvmSet(TT2, 1, 3, -246.6);
	cvmSet(TT2, 2, 3, 497.0);
	cvmSet(TT2, 3, 3, 1);
	cvmSet(TT1, 3, 3, 1);

	CvMat* inv_TT1 = cvCreateMat(4, 4, CV_32FC1);
	CvMat* cam_endd = cvCreateMat(4, 4, CV_32FC1);
	cvInvert(TT1, inv_TT1, CV_L2);

//==============================================================================================================
	cvGEMM(tranta, TT2, alpha, optional, beta, cam_endd, tABC);
	cvGEMM(cam_endd, inv_TT1, alpha, optional, beta, camera_end2, tABC);

	CvMat* PPP1 = cvCreateMat(4, 4, CV_32FC1);
	CvMat* PPP2 = cvCreateMat(4, 4, CV_32FC1);

	cvGEMM(tranta, TT1, alpha, optional, beta, PPP1, tABC);
	cvGEMM(tranta, TT2, alpha, optional, beta, PPP2, tABC);

	printf("\nthe projection in robot  base \n");
	int ii, iii;
	for (ii = 0; ii < 4; ii++) {
		printf("\n %f\t%f\t%f\t%f\n", PPP1->data.fl[ii * 4],
				PPP1->data.fl[ii * 4 + 1], PPP1->data.fl[ii * 4 + 2],
				PPP1->data.fl[ii * 4 + 3]);
	}
	printf("\n\n\n");
	for (iii = 0; iii < 4; iii++) {
		printf("\n %f\t%f\t%f\t%f\n", PPP2->data.fl[iii * 4],
				PPP2->data.fl[iii * 4 + 1], PPP2->data.fl[iii * 4 + 2],
				PPP2->data.fl[iii * 4 + 3]);
	}
	int adad;
	for (adad = 0; adad < 12; adad++) {

		projection2->data.fl[adad] = camera_end2->data.fl[adad];

	}

	CvMat* roto1 = cvCreateMat(3, 3, CV_32FC1);
	CvMat* roto2 = cvCreateMat(3, 3, CV_32FC1);

	int tess, hann;

	for (tess = 0; tess < 3; tess++) {
		for (hann = 0; hann < 3; hann++) {
			cvmSet(roto1, tess, hann, cvmGet(projection1, tess, hann));
			cvmSet(roto2, tess, hann, cvmGet(projection2, tess, hann));
		}
	}

	printf("\nthe determinant of rot1 and rot2 %f\t%f\n", cvDet(roto1),
			cvDet(roto2));

	printf("\nthe projection matrix\n");
	int badd;
	for (badd = 0; badd < 3; badd++) {
		printf("%f\t%f\t%f\t%f\n", projection1->data.fl[badd * 4],
				projection1->data.fl[badd * 4 + 1],
				projection1->data.fl[badd * 4 + 2],
				projection1->data.fl[badd * 4 + 3]);

	}
	printf("\n\n\n\n");
	int whoo;
	for (whoo = 0; whoo < 3; whoo++) {

		printf("%f\t%f\t%f\t%f\n", promat2->data.fl[whoo * 4],
				promat2->data.fl[whoo * 4 + 1], promat2->data.fl[whoo * 4 + 2],
				promat2->data.fl[whoo * 4 + 3]);

	}
	//============================================================================================

	// PROJECT OBJECT POINTS TO VIEW PLANE USING EXTRINSIC AND INSIC PARAMETERS

	//cvProjectPoints2(inv,rot_vector, trans_vector,K, distort,reproject_points,
	// NULL, NULL, NULL, NULL,NULL, 0		 );		      //12 parameters - NULL the other*/

	//==================================================================================================

	CvMat* input1 = cvCreateMat(countpoint, 2, CV_32FC1);
	CvMat* input2 = cvCreateMat(countpoint, 2, CV_32FC1);
	/*151, 267       136,294
	 181, 169       163,196
	 p3        489, 193       469,210
	 p4        463, 302       449,316
	 p5        469, 317       457,332
	 p6        159, 281       147,309*/

	//=================================================================================================
	//the block pixel values- planar surface


	int haha;
	for (haha = 0; haha < countpoint; haha++) {
		input1->data.fl[haha * 2] = first->data.fl[haha * 2];
		input1->data.fl[haha * 2 + 1] = first->data.fl[haha * 2 + 1];
		input2->data.fl[haha * 2] = second->data.fl[haha * 2];
		input2->data.fl[haha * 2 + 1] = second->data.fl[haha * 2 + 1];
	}

	////==========================================================================

	//the points are already undistorted
	//===========================================================================

//	CvMat* un_input1 = cvCreateMat(countpoint, 1, CV_32FC2);
//	CvMat* un_input2 = cvCreateMat(countpoint, 1, CV_32FC2);
//
//	cvUndistortPoints(input1, un_input1, camera_matrix, distortion_coefficients,
//			0, 0);
//
//	cvUndistortPoints(input2, un_input2, camera_matrix, distortion_coefficients,
//			0, 0);


	CvMat* one = cvCreateMat(2,countpoint, CV_32FC1);
	CvMat* two = cvCreateMat(2, countpoint, CV_32FC1);

	cvTranspose(input1, one);
	cvTranspose(input2, two);

	CvMat* point_3d_cloud = cvCreateMat(4, countpoint, CV_32FC1);

//=========================================================================================================
	CvMat* KP1 = cvCreateMat(3, 4, CV_32FC1);
	CvMat* KP0 = cvCreateMat(3, 4, CV_32FC1);
	cvGEMM(camera_matrix, P1, alpha, optional, beta, KP1, tABC);
	cvGEMM(camera_matrix, P, alpha, optional, beta, KP0, tABC);


	CvMat* K_inv1 = cvCreateMat(3, 3, CV_32FC1);

	CvMat* view1 = cvCreateMat(3, 4, CV_32FC1);
	CvMat* view2 = cvCreateMat(3, 4, CV_32FC1);
	int sali;
	for (sali = 0; sali < 12; sali++) {
		view1->data.fl[sali] = PPP1->data.fl[sali];
		view2->data.fl[sali] = PPP2->data.fl[sali];

	}

	cvTriangulatePoints(P, P1,one, two, point_3d_cloud);

	if (!(point_3d_cloud)) {
		printf("\n wrong result \n");
	}

	printf("\nPRINT THE POINT CLOUD  OF TRIANGULATION \n");
	int pt;
	for (pt = 0; pt < countpoint; pt++) {
		printf("%f\t\t%f\t\t%f\t\t%f\n", point_3d_cloud->data.fl[pt * 4],
				point_3d_cloud->data.fl[pt * 4 + 1],
				point_3d_cloud->data.fl[pt * 4 + 2],
				point_3d_cloud->data.fl[pt * 4 + 3]);

	}

	CvMat* inv = cvCreateMat(countpoint, 4, CV_32FC1);
	cvTranspose(point_3d_cloud, inv);
	CvMat* point_3d_cloud_hom = cvCreateMat(countpoint, 3, CV_32FC1);


//===================================================================================================
const char* filenamem = "/home/tesfu/Desktop/tesfu.txt";
  FILE* fil;
  fil= fopen(filenamem, "w+");
  if (fil == NULL)
  {

	  return;
  }

//=============================================================================================

	// reshape
	//  CvMat* inv_4d = cvCreateMat(count, 1,CV_32FC4);
	// cvReshape(inv,inv_4d,4,count);

	cvConvertPointsHomogeneous(inv, point_3d_cloud_hom);

	for (pt = 0; pt < countpoint; pt++)
	{

		printf("%f\t\t%f\t\t%f\n", point_3d_cloud_hom->data.fl[pt * 3],
				point_3d_cloud_hom->data.fl[pt * 3 + 1],
				point_3d_cloud_hom->data.fl[pt * 3 + 2]);

		fprintf(fil," %f %f %f\n",point_3d_cloud_hom->data.fl[pt * 3],
				point_3d_cloud_hom->data.fl[pt * 3 + 1], point_3d_cloud_hom->data.fl[pt * 3 + 2]);

	}
	fclose(fil);
	//fputs("\n THE TRIANGULATION RESULT\n",fil);


	CvMat* projeccam1 = cvCreateMat(3, 4, CV_32FC1);
	CvMat* projeccam2 = cvCreateMat(3, 4, CV_32FC1);

	cvGEMM(camera_matrix, projection1, alpha, optional, beta, projeccam1, tABC);
	cvGEMM(camera_matrix, projection2, alpha, optional, beta, projeccam2, tABC);

	cvInvert(camera_matrix, K_inv1, CV_LU);




//=========================================================================================================

	const char* filenamea =  "/home/tesfu/Desktop/tesfiter.txt";
	  FILE* filoff;
	  filoff= fopen(filenamea, "w+");
	  if (filoff == NULL)
	  {

		  return;
	  }


	  //===============================================================================================



	  CvMat* U_homogen = cvCreateMat(3, 1, CV_32FC1);
	  	CvMat* U1_homogen = cvCreateMat(3, 1, CV_32FC1);
	  	CvMat* U_result = cvCreateMat(3, 1, CV_32FC1);
	  	CvMat* U1_result = cvCreateMat(3, 1, CV_32FC1);

	point_cloud = cvCreateMat(countpoint, 3, CV_32FC1);
	CvMat* cloud_projection_error = cvCreateMat(countpoint, 1, CV_32FC1);


	int ts;
	for (ts = 0; ts < countpoint; ts++) {

		U_homogen->data.fl[0] = (float)cvmGet(first, ts, 0);   // input11
		U_homogen->data.fl[1] = (float)cvmGet(first, ts, 1);
		U_homogen->data.fl[2] = 1;

		U1_homogen->data.fl[0] = (float)cvmGet(second, ts, 0);  //input22
		U1_homogen->data.fl[1] = (float)cvmGet(second, ts, 1);
		U1_homogen->data.fl[2] = 1;

		cvGEMM(K_inv1, U_homogen, alpha, optional, beta, U_result, tABC);
		cvGEMM(K_inv1, U1_homogen, alpha, optional, beta, U1_result, tABC);


		iterate_triangulation_LS(U_homogen, U1_homogen, P, P1);



//============================================================================================================

		CvMat* x = cvCreateMat(3, 1, CV_32FC1);
		cvGEMM(P1, X_iter, alpha, optional, beta, x, tABC);
		//  printf("\n  P1*POINT %f\t\t%f\t\t%f\n",cvmGet(x, 0,0),cvmGet(x, 1,0),cvmGet(x, 2,0));

		// PROJECT TO SCREEN -ON 2D
		CvMat* XPt1_image = cvCreateMat(3, 1, CV_32FC1);
		cvGEMM(KP1, X_iter, alpha, optional, beta, XPt1_image, tABC);
		// printf("\n  POINT COORDINATE ON SCREEN %f\t\t%f\t\t%f\n",cvmGet(XPt1_image, 0,0),cvmGet(XPt1_image,
		//   1,0),cvmGet(XPt1_image, 2,0));



		point_cloud->data.fl[ts * 3] = (float)cvmGet(X_iter, 0, 0);
		point_cloud->data.fl[ts * 3 + 1] = (float)cvmGet(X_iter, 1, 0);
		point_cloud->data.fl[ts * 3 + 2] = (float)cvmGet(X_iter, 2, 0);


		CvMat* point_2d = cvCreateMat(1, 2, CV_32FC1);
		point_2d->data.fl[0] = (float)cvmGet(XPt1_image, 0, 0)
				/ cvmGet(XPt1_image, 2, 0);
		point_2d->data.fl[0] = (float)cvmGet(XPt1_image, 1, 0)
				/ cvmGet(XPt1_image, 2, 0);



//=============================================================================================================

		CvMat* XPt_image = cvCreateMat(3, 1, CV_32FC1);

		cvGEMM(KP1, X_iter, alpha, optional, beta, XPt_image, tABC);
		CvMat* normalized_value = cvCreateMat(2, 1, CV_32FC1);


		CvMat* U_value = cvCreateMat(2, 1, CV_32FC1);
		U_value->data.fl[0] = cvmGet(second, ts, 0);
		U_value->data.fl[1] = cvmGet(second, ts, 1);

		normalized_value->data.fl[0] = cvmGet(XPt_image, 0, 0)
				/ cvmGet(XPt_image, 2, 0);
		normalized_value->data.fl[1] = cvmGet(XPt_image, 1, 0)
				/ cvmGet(XPt_image, 2, 0);



		CvMat* reprojection_error = cvCreateMat(2, 1, CV_32FC1);

		cvSub(normalized_value, U_value, reprojection_error, NULL);
		float error = cvNorm(reprojection_error, NULL, CV_L2, NULL);
		cloud_projection_error->data.fl[ts] = error;
		cvReleaseMat(&normalized_value);


	}



	printf("\n THE CLOUD POINTS  USING ITERATIVE TRIANGULATION \n");
	int print;
	for (print = 0; print <countpoint; print++) {

		printf("\n %f\t%f\t%f\n", point_cloud->data.fl[print * 3],
				point_cloud->data.fl[print * 3 + 1],
				point_cloud->data.fl[print * 3 + 2]);

		fprintf(filoff, "%f %f %f\n",point_cloud->data.fl[print * 3],
				point_cloud->data.fl[print * 3 + 1], point_cloud->data.fl[print * 3 + 2]);


	}
fclose(filoff);
//==================================================================================================

//	  printf("\n DISTANCE VALUES \n");
//	  CvMat* sub = cvCreateMat(3,1, CV_32FC1);
//	  CvMat* SUBB =cvCreateMat(3,1, CV_32FC1);
//	  CvMat* dstt = cvCreateMat(3,1,CV_32FC1);
//	int  ti, tutu;
//	for (tutu = 0; tutu<6;tutu++)
//	{
//		cvCopy(sub, SUBB, NULL);
//		for (ti= 0; ti<3;ti++)
//		{
//			cvmSet(sub,ti, 0 , cvmGet(point_cloud,tutu, ti));
//
//		}
//	cvSub(sub, SUBB,dstt, NULL);
//	float result = cvNorm(dstt, NULL, CV_L2, NULL);
//		printf("\n%f\n", result);
//
//	}



//
//	cvReleaseMat(&sub);
//	cvReleaseMat(&SUBB);
//	cvReleaseMat(&dstt);

	cvReleaseMat(&rotationXX);
	cvReleaseMat(&rotationYY);
	cvReleaseMat(&rotationZZ);
	cvReleaseMat(&rotationZYX);



	cvReleaseMat(&rotto);
	cvReleaseMat(&relateROTO);
	cvReleaseMat(&tranta);
//
//	cvReleaseMat(&un_input1);
//	cvReleaseMat(&un_input2);
//	cvReleaseMat(&input11);
	//cvReleaseMat(&input22);

	cvReleaseMat(&view1);
	cvReleaseMat(&view2);

	cvReleaseMat(&PPP1);
	cvReleaseMat(&PPP2);
	cvReleaseMat(&projeccam1);
	cvReleaseMat(&projeccam2);
	cvReleaseMat(&rotation_n1);
	cvReleaseMat(&rotation_n2);
	cvReleaseMat(&rotation_mat1);
	cvReleaseMat(&rotation_mat2);
	cvReleaseMat(&camera_end1);
	cvReleaseMat(&camera_end2);
	cvReleaseMat(&U1_homogen);
	cvReleaseMat(&X_iter);
	cvReleaseMat(&KP1);
	cvReleaseMat(&KP0);
	cvReleaseMat(&U_homogen);
	cvReleaseMat(&TT1);
	cvReleaseMat(&TT2);
	cvReleaseMat(&projection1);
	cvReleaseMat(&projection2);
	cvReleaseMat(&roto1);
	cvReleaseMat(&roto2);

	//=======================================================================================================
	int aha;
	cloudX = cvCreateMat(countpoint, 1, CV_32FC1);
	cloudY = cvCreateMat(countpoint, 1, CV_32FC1);
	cloudZ = cvCreateMat(countpoint, 1, CV_32FC1);

	for (aha = 0; aha < countpoint; aha++) {

		cloudX->data.fl[aha] = (float)cvmGet(point_cloud, aha, 0);
		cloudY->data.fl[aha] = (float)cvmGet(point_cloud, aha, 1);
		cloudZ->data.fl[aha] =(float)cvmGet(point_cloud, aha, 2);

	}


// mean of the error
	CvScalar mean_error = cvAvg(cloud_projection_error, NULL);

	//depth map
	CvMat* depth = cvCreateMat(countpoint, 1, CV_32FC1);
	int tet;
	for (tet = 0; tet < countpoint; tet++) {

		depth->data.fl[tet] = (float)cvmGet(point_cloud, tet, 2);

	}

	//==========================================================================================================
	//  POINT COUD x(0), X(1), X(2)

	double minValue;
	double maxValue;
	// find global minimum and maximum and find their position
	CvPoint minloc;
	CvPoint maxloc;

	cvMinMaxLoc(depth, &minValue, &maxValue, &minloc, &maxloc, 0);
	IplImage* tmp;
	tmp = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	int i;
	// CvPoint2D32f* p;
	//cvCopy(points1,p, NULL)

	for (i = 0; i < countpoint; i++) {

		float d = fmax(fmin((cvmGet(point_cloud, i, 2) - minValue)
								/ (maxValue - minValue), 1.0), 0.0);

		cvCircle(tmp, cvPoint(cvmGet(first, i, 0), cvmGet(first, i, 1)), 1,
				cvScalar(255 * (1.0 + d), 180,  200, 80), 1, 8, 0);
	}
	cvNamedWindow("depth_map", CV_WINDOW_AUTOSIZE);

	cvCvtColor(tmp, tmp, CV_HSV2BGR);
	cvShowImage("Depth_Map", tmp);
	int c = cvWaitKey(100000);
	if (c == 'p') {
		c = 0;
		while (c != 'p' && c != 27) {
			c = cvWaitKey(2500);
		}
	}
	if (c == 27) {
		printf(" done");
	}

	cvReleaseImage(&tmp);
	cvReleaseMat(&U_homogen);
//cvReleaseMat(&image1_binary);

	cvReleaseMat(&point_3d_cloud);
	 cvReleaseMat(&depth);

	cvDestroyWindow("Depth_map");

}

//======================================================================================================
float testTriangulation(CvMat* point_cloud, int count) {
	CvMat* cloud = cvCreateMat(count, 3, CV_32FC1);
	cloud = cvCloneMat(point_cloud);
	int t;
	float status;
	CvMat* status_mat = cvCreateMat(count, 1, CV_32FC1);
	for (t = 0; t < cloud->rows; t++) {
		if (cvmGet(cloud, t, 2) > 0) {
			status = 1;
		} else {
			status = 0;
		}
		status_mat->data.fl[t] = status;

	}
	int cc = cvCountNonZero(status_mat);
	float percent = 100.0 * (float) cc / (float) point_cloud->rows;

//printf("the percentage of point cloud in front of camera \n %f", percent);
	cvReleaseMat(&cloud);
	return percent;

}
//=== PASS IN THE ANGLES IN RADIAN VALUES
void euler_rotation(float thetaZ, float thetaY, float thetaX)
{
float RotZ[]= {cos(thetaZ), -sin(thetaZ), 0 , sin(thetaZ), cos(thetaZ),0, 0 ,0 ,1};
float RotY[]= {cos(thetaY),0 , sin(thetaY),0,1, 0, -sin(thetaY),0, cos(thetaY)};
float RotX[]= {1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0 , sin(thetaX), cos(thetaX)};


int uu;
 for ( uu= 0; uu<9;uu++)
 {
	rotationZZ->data.fl[uu] =RotZ[uu];
	rotationYY->data.fl[uu] =RotY[uu];
	rotationXX->data.fl[uu]= RotX[uu];

 }
 CvMat* rotationYX = cvCreateMat(3,3, CV_32FC1);
	//rzyx = rz *ry * rx

	cvGEMM(rotationYY,rotationXX, alpha, optional, beta, rotationYX, tABC);
	cvGEMM(rotationZZ,rotationYX, alpha, optional, beta,rotationZYX, tABC);

   cvReleaseMat(&rotationYX);

}



