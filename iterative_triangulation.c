/*
 * iterative_triangulation.c
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
#include "iterative_triangulation.h"
void triangulation_construc(CvMat* first, CvMat* second, CvMat* p, CvMat* p1,int count)

{ //================================================================================================================

	//triangulatePoints(first, second, p,p1, count);

// ================================================================================================================
	//CvMat* P = cvCloneMat(p);
	//CvMat* P1 = cvCloneMat(p1);

	//CvMat*  U_homogen= cvCreateMat(3,1, CV_32FC1);
	//CvMat*  U1_homogen= cvCreateMat(3,1, CV_32FC1);
	//CvMat*  U_result= cvCreateMat(3,1, CV_32FC1);
	//CvMat*  U1_result= cvCreateMat(3,1, CV_32FC1);
	//==============================================================================================================
	/*int ts;
	 for (ts= 0;ts<count;ts++)
	 {
	 U_homogen->data.fl[0] = cornersA[ts].x;
	 U_homogen->data.fl[1] = cornersA[ts].y;
	 U_homogen->data.fl[2] = 1;

	 U1_homogen->data.fl[0] = cornersB[ts].x;
	 U1_homogen->data.fl[1] = cornersB[ts].y;
	 U1_homogen->data.fl[2] = 1;

	 // cvGEMM(K_inv,U_homogen,alpha, optional, beta,U_result ,tABC);
	 //cvGEMM(K_inv,U1_homogen,alpha, optional, beta,U1_result ,tABC);

	 //==========================================================================================================
	 //  iterate_triangulation_LS(U_homogen, P, U1_homogen, P1);

	 }*/

	//mean of re-projection error
//cvReleaseMat(&U_homogen);
//cvReleaseMat(&U1_homogen);
}


//===============================================================================================================

void iterate_triangulation_LS(CvMat* U, CvMat* U1, CvMat* P, CvMat* P1) {

	CvMat* A;
	CvMat* B;
	A = cvCreateMat(4, 3, CV_32FC1);
	B = cvCreateMat(4, 1, CV_32FC1);
	X_iter = cvCreateMat(4, 1, CV_32FC1);
	X0_noniter = cvCreateMat(3, 1, CV_32FC1);

	CvMat* AA;
	CvMat* BB;

	AA = cvCreateMat(4, 3, CV_32FC1);
	BB = cvCreateMat(4, 1, CV_32FC1);
//==============================================================================================================
	float wi = 1, wi1 = 1;
	int i;
	for (i = 0; i < 10; i++) { //Hartley suggests 10 iterations at most

		//triangulation_operation(U,P,U1,P1);

		//cvmGet
		A->data.fl[0] = CV_MAT_ELEM(*U, float, 0,0) * CV_MAT_ELEM(*P,float, 2,0)
				- CV_MAT_ELEM(*P,float, 0,0);
		A->data.fl[1] = CV_MAT_ELEM(*U,float, 0,0) * CV_MAT_ELEM(*P,float, 2,1)
				- CV_MAT_ELEM(*P,float, 0,1);
		A->data.fl[2] = CV_MAT_ELEM(*U,float, 0,0) * CV_MAT_ELEM(*P,float, 2,2)
				- CV_MAT_ELEM(*P,float, 0,2);
		A->data.fl[3] = CV_MAT_ELEM(*U,float, 1,0) * CV_MAT_ELEM(*P,float, 2,0)
				- CV_MAT_ELEM(*P,float, 1,0);
		A->data.fl[4] = CV_MAT_ELEM(*U,float, 1,0) * CV_MAT_ELEM(*P,float, 2,1)
				- CV_MAT_ELEM(*P,float, 1,1);
		A->data.fl[5] = CV_MAT_ELEM(*U,float, 1,0) * CV_MAT_ELEM(*P,float, 2,2)
				- CV_MAT_ELEM(*P,float, 1,2);
		A->data.fl[6] = CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,0) - CV_MAT_ELEM(*P1,float, 0,0);
		A->data.fl[7] = CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,1) - CV_MAT_ELEM(*P1,float, 0,1);
		A->data.fl[8] = CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,2) - CV_MAT_ELEM(*P1,float, 0,2);
		A->data.fl[9] = CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,0) - CV_MAT_ELEM(*P1,float, 1,0);
		A->data.fl[10] = CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,1) - CV_MAT_ELEM(*P1,float, 1,1);
		A->data.fl[11] = CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,2) - CV_MAT_ELEM(*P1,float, 1,2);

		B->data.fl[0] = -(CV_MAT_ELEM(*U,float, 0,0)
				* CV_MAT_ELEM(*P,float, 2,3) - CV_MAT_ELEM(*P,float, 0,3));
		B->data.fl[1] = -(CV_MAT_ELEM(*U,float, 1,0)
				* CV_MAT_ELEM(*P,float, 2,3) - CV_MAT_ELEM(*P,float, 1,3));
		B->data.fl[2] = -(CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,3) - CV_MAT_ELEM(*P1,float, 0,3));
		B->data.fl[3] = -(CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,3) - CV_MAT_ELEM(*P1,float, 1,3));

//========================================================================================================

		cvSolve(A, B, X0_noniter, CV_SVD);

		//	 printf("\n the world coordinate\n %f\t\t%f\t\t%f",X0_noniter->data.fl[0],X0_noniter->data.fl[1],
		//		 X0_noniter->data.fl[2]);
		// printf("\n image coordinate\n %f\t\t%f\t\t%f\n",U->data.fl[0],U->data.fl[1],
		//	U->data.fl[2]);

		X_iter->data.fl[0] = cvmGet(X0_noniter, 0, 0);
		X_iter->data.fl[1] = cvmGet(X0_noniter, 1, 0);
		X_iter->data.fl[2] = cvmGet(X0_noniter, 2, 0);
		cvmSet(X_iter, 3, 0, 1);

		//X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
		//recalculate weights

		//CvMat* row2P = cvCreateMat(4,1, CV_32FC1);
		//CvMat* row2P1 = cvCreateMat(4,1, CV_32FC1);

		float p2x = cvmGet(P, 2, 0) * cvmGet(X_iter, 0, 0)
				+ cvmGet(P, 2, 1) * cvmGet(X_iter, 1, 0)
				+ cvmGet(P, 2, 2) * cvmGet(X_iter, 2, 0)
				+ cvmGet(P, 2, 3) * cvmGet(X_iter, 3, 0);

		float p2x1 = cvmGet(P1, 2, 0) * cvmGet(X_iter, 0, 0)
				+ cvmGet(P1, 2, 1) * cvmGet(X_iter, 1, 0)
				+ cvmGet(P1, 2, 2) * cvmGet(X_iter, 2, 0)
				+ cvmGet(P1, 2, 3) * cvmGet(X_iter, 3, 0);

		//   printf("\n the weight value %f\t\t%f\n", p2x, p2x1);

		//breaking point
		if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON)
			break;

		wi = p2x;
		wi1 = p2x1;
//===============================================================================================================
		//re-weight equations and solve

		AA->data.fl[0] = (CV_MAT_ELEM(*U,float, 0,0)
				* CV_MAT_ELEM(*P,float, 2,0) - CV_MAT_ELEM(*P,float, 0,0)) / wi;
		AA->data.fl[1] = (CV_MAT_ELEM(*U,float, 0,0)
				* CV_MAT_ELEM(*P,float, 2,1) - CV_MAT_ELEM(*P,float, 0,1)) / wi;
		AA->data.fl[2] = (CV_MAT_ELEM(*U,float, 0,0)
				* CV_MAT_ELEM(*P,float, 2,2) - CV_MAT_ELEM(*P,float, 0,2)) / wi;
		AA->data.fl[3] = (CV_MAT_ELEM(*U,float, 1,0)
				* CV_MAT_ELEM(*P,float, 2,0) - CV_MAT_ELEM(*P,float, 1,0)) / wi;
		AA->data.fl[4] = (CV_MAT_ELEM(*U,float, 1,0)
				* CV_MAT_ELEM(*P,float, 2,1) - CV_MAT_ELEM(*P,float, 1,1)) / wi;
		AA->data.fl[5] = (CV_MAT_ELEM(*U,float, 1,0)
				* CV_MAT_ELEM(*P,float, 2,2) - CV_MAT_ELEM(*P,float, 1,2)) / wi;
		AA->data.fl[6] = (CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,0) - CV_MAT_ELEM(*P1,float, 0,0))
				/ wi1;
		AA->data.fl[7] = (CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,1) - CV_MAT_ELEM(*P1,float, 0,1))
				/ wi1;
		AA->data.fl[8] = (CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,2) - CV_MAT_ELEM(*P1,float, 0,2))
				/ wi1;
		AA->data.fl[9] = (CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,0) - CV_MAT_ELEM(*P1,float, 1,0))
				/ wi1;
		AA->data.fl[10] = (CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,1) - CV_MAT_ELEM(*P1,float, 1,1))
				/ wi1;
		AA->data.fl[11] = (CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,2) - CV_MAT_ELEM(*P1,float, 1,2))
				/ wi1;

		BB->data.fl[0] = -(CV_MAT_ELEM(*U,float, 0,0)
				* CV_MAT_ELEM(*P,float, 2,3) - CV_MAT_ELEM(*P,float, 0,3)) / wi;
		BB->data.fl[1] = -(CV_MAT_ELEM(*U,float, 1,0)
				* CV_MAT_ELEM(*P,float, 2,3) - CV_MAT_ELEM(*P,float, 1,3)) / wi;
		BB->data.fl[2] = -(CV_MAT_ELEM(*U1,float, 0,0)
				* CV_MAT_ELEM(*P1,float, 2,3) - CV_MAT_ELEM(*P1,float, 0,3))
				/ wi1;
		BB->data.fl[3] = -(CV_MAT_ELEM(*U1,float, 1,0)
				* CV_MAT_ELEM(*P1,float, 2,3) - CV_MAT_ELEM(*P1,float, 1,3))
				/ wi1;

		cvSolve(AA, BB, X0_noniter, CV_SVD);
		/* if(cvmGet(X_iter,2,0)< 0)

		 {
		 continue;
		 }*/

	}
	X_iter->data.fl[0] = (float)cvmGet(X0_noniter, 0, 0);
	X_iter->data.fl[1] = (float)cvmGet(X0_noniter, 1, 0);
	X_iter->data.fl[2] = (float)cvmGet(X0_noniter, 2, 0);
	cvmSet(X_iter, 3, 0, 1);
	cvReleaseMat(&A);
	cvReleaseMat(&B);
	cvReleaseMat(&AA);
	cvReleaseMat(&BB);
	cvReleaseMat(&X0_noniter);

}


