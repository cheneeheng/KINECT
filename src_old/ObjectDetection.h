/**
* @file   ObjectDetection.h
* @Author Tick Son Wang, EE Heng Chen
* @date   05.07.2016
* @brief  Header file for the utility functions
*
* Detailed description of file.
*/

/******************************************************************************
Dependencies
******************************************************************************/
#include <iostream>
#include <list>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

/******************************************************************************
Function Headers
******************************************************************************/

void depthImaging(
	cv::Mat &depth_image,
	cv::Mat depth_global);

void segmentHSV(
	cv::Mat src,
	int *hs,
	cv::Mat& seg_mask_noisefree,
	cv::Rect& box);

void noiseRemove(
	cv::Mat seg_mask,
	cv::Mat& seg_mask_noisefree,
	cv::Rect& box);

void pointCloudTrajectory(
	cv::Mat cloud,
	cv::Vec3f &point);

