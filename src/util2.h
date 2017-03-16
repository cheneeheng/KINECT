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
#include "algo.h"

/******************************************************************************
Function Headers
******************************************************************************/

void showPrediction(
	Mat &imgHistogram,
	vector<int>data,
	vector<string>label);

void showPrediction(
	Mat &imgHistogram,
	vector<double>data,
	vector<string>label);

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
	Rect &box_,
	int iter,
	float *ratio,
	float threshold);

Vec3f computePlane(Vec3f A, Vec3f B, Vec3f C);

Vec3f normalization3D(Vec3f vec);

Vec3f crossProd(Vec3f A, Vec3f B);

float dotProd(Vec3f A, Vec3f B);

void normalPlaneCheck(Vec4f &plane_equation);


void getColorThreshold(Mat src, int (&hue_range)[2], int (&sat_range)[2]);

struct MouseCallBackHistData{
	int _hscale, //!< hue scale
		_sscale; //!< saturation scale
	int _hbin_scale, //!< hue bin scale
		_sbin_scale; //!< saturation bin scale
	cv::Mat	_hist; //!< contains the hist data
	int _h_mean; //!< mean hue
	int _s_mean; //!< mean saturation
};

struct TrackbarCBDataCalib{
	cv::Mat _hsv; //!< hsv data
	cv::Mat _seg_mask; //!< segmentation mask {0,255}
	cv::Mat _seg_out_rgb;  //!< segmented image in rgb
	cv::Mat _seg_in_rgb; //!< src_image for segmentation in rgb
	int 	_H_top, //!< upper bound for hue
			_H_bot, //!< lower bound for hue
			_S_top, //!< upper bound for saturation
			_S_bot; //!< lower bound for saturation
	std::string _wn_seg_mask; //!< window name for segmentation mask
	std::string _wn_seg_rgb; //!< window name for segmentated rgb image
};

void mouseCallBackHist(	int eventcode, 
						int x, int y, 
						int flags, 
						void* data);


void trackbarCallBackCalib(int trackpos, void* data);

void segmentHSV(cv::Mat src_hsv, cv::Mat src_rgb,
                cv::Mat& seg_mask, cv::Mat& seg_rgb,
                int h_top, int h_bot, int s_top, int s_bot);

