/**
* @file   util.h
* @Author Tick Son Wang, EE Heng Chen
* @date   05.07.2016
* @brief  Header file for the utility functions
*
* Detailed description of file.
*/



/******************************************************************************
Dependencies
******************************************************************************/

#include "dataDeclaration.h"

/******************************************************************************
Function Headers
******************************************************************************/

void depthImaging(
	Mat &depth_image,
	Mat depth_global);

void segmentHSV(
	Mat src,
	int *hs,
	Mat& seg_mask_noisefree,
	Rect& box);

void segmentHSVEDIT(
	Mat src,
	Mat& seg_mask,
	int h_top,
	int h_bot,
	int s_top,
	int s_bot);

void noiseRemove(
	Mat seg_mask,
	Mat& seg_mask_noisefree,
	Rect& box);

Rect detectFaceAndEyes(
	Mat &frame,
	CascadeClassifier face_cascade);

bool contactCheck(
	Mat obj1, 
	Mat obj2, 
	Rect box_obj1, 
	Rect box_obj2);

void pointCloudTrajectory(
	Mat cloud,
	Vec3f &point);

Vec4f RANSAC3DPlane(
	Mat cloud,
	Mat &plane,
	int iter,
	float *ratio,
	float threshold);

Vec3f computePlane(Vec3f A, Vec3f B, Vec3f C);

Vec3f normalization3D(Vec3f vec);

Vec3f crossProd(Vec3f A, Vec3f B);

float dotProd(Vec3f A, Vec3f B);

void normalPlaneCheck(Vec4f &plane_equation);



