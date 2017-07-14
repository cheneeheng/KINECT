#include "ObjectDetection.h"

void depthImaging(
	cv::Mat &depth_image, 
	cv::Mat depth_global)
{
	uint16_t mGamma[2048];
	for(int i=0;i<2048;i++)
	{
		float v = i/2048.0; 
		v = powf(v,3)*6; 
		mGamma[i] = v*6*256;
	}

	u_char *ptr     = depth_image.data;
	uint16_t *depth = (uint16_t*)depth_global.data;

	for(int i=0;i<640*480;i++)
	{
		int pval = mGamma[depth[i]/2];
		int lb = pval & 0xff;
		switch ( pval >> 8 )
		{
			case 0:
				ptr[3*i+2] = 255;
				ptr[3*i+1] = 255-lb;
				ptr[3*i+0] = 255-lb;
				break;
			case 1:
				ptr[3*i+2] = 255;
				ptr[3*i+1] = lb;
				ptr[3*i+0] = 0;
				break;
			case 2:
				ptr[3*i+2] = 255-lb;
				ptr[3*i+1] = 255;
				ptr[3*i+0] = 0;
				break;
			case 3:
				ptr[3*i+2] = 0;
				ptr[3*i+1] = 255;
				ptr[3*i+0] = lb;
				break;
			case 4:
				ptr[3*i+2] = 0;
				ptr[3*i+1] = 255-lb;
				ptr[3*i+0] = 255;
				break;
			case 5:
				ptr[3*i+2] = 0;
				ptr[3*i+1] = 0;
				ptr[3*i+0] = 255-lb;
				break;
			default:
				ptr[3*i+2] = 0;
				ptr[3*i+1] = 0;
				ptr[3*i+0] = 0;
				break;
		}
	}
}

void noiseRemove(
	cv::Mat seg_mask,
	cv::Mat& seg_mask_noisefree,
	cv::Rect& box)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(seg_mask, contours, hierarchy,
	CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	std::vector<std::vector<cv::Point> > contours_poly(contours.size());
	std::vector<cv::Rect> box2(contours.size());
	double biggest_box = 0;
	int big1 = 0, big2 = 0; 
	for (int j=0;j<(int)contours.size();j++)
	{
		approxPolyDP(cv::Mat(contours[j]), contours_poly[j], 3, true);
		if (biggest_box < contourArea(contours[j]))
		{
			biggest_box = contourArea(contours[j]);
			box2[0] = boundingRect(cv::Mat(contours_poly[j]));
			big1 = j;
		}
	}
	cv::Mat tmp_img = cv::Mat::zeros(seg_mask.size(), CV_8UC1);
	drawContours( tmp_img, contours, big1, 1, -1);
	seg_mask_noisefree = tmp_img;
	box = box2[0];
}

void segmentHSV(
	cv::Mat src,
	int *hs,
	cv::Mat& seg_mask_noisefree,
	cv::Rect& box)
{
	cv::Mat src_hsv, seg_mask;
	cvtColor(src,src_hsv,CV_RGB2HSV);

	std::vector<cv::Mat> splitted_HSV;
	split(src_hsv, splitted_HSV);

	cv::Mat seg_mask1 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
	cv::Mat seg_mask2 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
	cv::Mat seg_mask3 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
	cv::Mat seg_mask4 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);

	seg_mask1 = splitted_HSV[0] <= (hs[0]);
	seg_mask2 = splitted_HSV[0] >= (hs[1]);
	seg_mask3 = splitted_HSV[1] <= (hs[2]);
	seg_mask4 = splitted_HSV[1] >= (hs[3]);

	if (hs[0] < hs[1])
		seg_mask = ((seg_mask1 > 0) | (seg_mask2 > 0)) & 
					(seg_mask3 > 0) & 
					(seg_mask4 > 0);
	else
		seg_mask =  (seg_mask1 > 0) & 
					(seg_mask2 > 0) & 
					(seg_mask3 > 0) & 
					(seg_mask4 > 0);
	seg_mask = seg_mask / 255; //scale to {0,1}

	//denoising
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(seg_mask, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	std::vector<std::vector<cv::Point> > contours_poly(contours.size());
	std::vector<cv::Rect> box2(contours.size());
	double biggest_box = 0;
	int big1 = 0;

	for (int j=0;j<(int)contours.size();j++)
	{
		approxPolyDP(cv::Mat(contours[j]), contours_poly[j], 3, true);
		if (biggest_box < contourArea(contours[j]))
		{
			biggest_box = contourArea(contours[j]);
			box2[0] = boundingRect(cv::Mat(contours_poly[j]));
			big1 = j;
		}
	}
	cv::Mat tmp_img = cv::Mat::zeros(seg_mask.size(), CV_8UC1);
	drawContours(tmp_img, contours, big1, 1, -1);
	seg_mask_noisefree = tmp_img;
	box = box2[0];

/*
	for (int j=0;j<(int)contours.size();j++)
	{
		cv::Mat tmp_img = cv::Mat::zeros(seg_mask.size(), CV_8UC1);
		cv::Mat seg_mask2 = cv::Mat::zeros(seg_mask.size(), CV_8UC1);
		approxPolyDP(cv::Mat(contours[j]), contours_poly[j], 3, true);
		drawContours(tmp_img, contours, j, 1, -1);
		seg_mask.copyTo(seg_mask2, tmp_img);
		if (biggest_box < sum(seg_mask2)[0])
		{
			biggest_box = sum(seg_mask2)[0];
			box2[0] = boundingRect(cv::Mat(contours_poly[j]));
			big1 = j;
		}
	}
	cv::Mat tmp_img = cv::Mat::zeros(seg_mask.size(), CV_8UC1);
	drawContours(tmp_img, contours, big1, 1, -1);
	seg_mask_noisefree = tmp_img;
	box = box2[0];
	imshow("rgb_m",tmp_img*255);
*/
}

void pointCloudTrajectory(
	cv::Mat cloud,
	cv::Vec3f &Point)
{
	cv::Vec3f single_Point,tmp_Point;
	float tmp = 5.0;
	int counter = 0;
	for(int i=0;i<cloud.size().height;i++)
	{
		for(int ii=0;ii<cloud.size().width;ii++)
		{
			single_Point = cloud.at<cv::Vec3f>(i,ii);
			if(single_Point[2]<tmp && single_Point[2] > 0)
				tmp = single_Point[2];
		}
	}
	for(int i=0;i<cloud.size().height;i++)
	{
		for(int ii=0;ii<cloud.size().width;ii++)
		{
			single_Point = cloud.at<cv::Vec3f>(i,ii);
			if(single_Point[2] < tmp+0.05 && single_Point[2] > tmp-0.05)
			{
				tmp_Point += single_Point; 
				counter += 1;
			}
		}
	}
	tmp_Point = tmp_Point/counter;  
	Point[0]  = tmp_Point[0];
	Point[1]  = tmp_Point[1];
	Point[2]  = tmp_Point[2];
}


