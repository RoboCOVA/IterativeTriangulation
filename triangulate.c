/*
 * triangulate.c
 *
 *  Created on: Jan 24, 2015
 *      Author: tesfu
 */


#include<stdio.h>
/*
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/video/background_segm.hpp"
*/

#include"triangulate.h"

//========================================================================================

void parameters() {

	float rotation_mat[] = { 0.859123, 0.511664, -0.010393, 0.151866, -0.235497,
			0.959936, 0.488717, -0.826282, -0.280025 };
	float translation[] = { -81.644470, 50.884407, 145.855942 };

	float cam_parameters[] = { 1167.7086 ,0.0 ,324.2511,0.0, 1168.1241,
			   239.8463, 0.0, 0.0, 1.0 };
	float distortion[] = { 0.01419,-0.12238,  -0.0063, -0.005697, 0.0 };

	distortion_coefficients = cvCreateMat(5, 1, CV_32FC1);

	translation_vector = cvCreateMat(3, 1, CV_32FC1);
	camera_matrix = cvCreateMat(3, 3, CV_32FC1);
	rotationInMatrix = cvCreateMat(3, 3, CV_32FC1);
	//================================================================================================================
	int k;
	for (k = 0; k < 3; k++) {
		camera_matrix->data.fl[k * 3] = cam_parameters[k * 3];
		camera_matrix->data.fl[k * 3 + 1] = cam_parameters[k * 3 + 1];
		camera_matrix->data.fl[k * 3 + 2] = cam_parameters[k * 3 + 2];

	}
	int x;
	for (x = 0; x < 3; x++) {
		rotationInMatrix->data.fl[x * 3] = rotation_mat[x * 3];
		rotationInMatrix->data.fl[x * 3 + 1] = rotation_mat[x * 3 + 1];
		rotationInMatrix->data.fl[x * 3 + 2] = rotation_mat[x * 3 + 2];

	}
	int y;
	for (y = 0; y < 3; y++) {
		translation_vector->data.fl[y] = translation[y];
	}
	int j;
	for (j = 0; j < 5; j++) {
		distortion_coefficients->data.fl[j] = distortion[j];

	}
}
//==========================================================================================================
int main(int argc, char** argv)

{

	// printMat();
	parameters();
	optical_flow_estimation(rotationInMatrix, translation_vector,
			distortion_coefficients);

	return 0;
}




