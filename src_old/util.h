/*
 * util.h
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <fstream>
#include <memory>
#include <iostream>
#include <pthread.h>
#include <vector>
#include <semaphore.h>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

// =============================================================================
// facedetect
// =============================================================================
cv::Rect detectFaceAndEyes(
	cv::Mat &frame,
	cv::CascadeClassifier face_cascade)
{
	std::vector<cv::Rect> faces;
	cv::Rect face;
	cv::Mat frame_gray;
	cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
	//equalizeHist( frame_gray, frame_gray );
	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 
								  1.2, 2, 0, cv::Size(60, 60), cv::Size(100, 100));
	for(size_t i=0;i<faces.size();i++)
	{
		if (faces[i].area() > face.area())
			face = faces[i];
		cv::Point center(faces[i].x + faces[i].width*0.5, 
					 faces[i].y + faces[i].height*0.5);
		cv::ellipse(frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 
				0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0);
	}
	return face;
}

// =============================================================================
// contactCheck
// =============================================================================
bool contactCheck(
	cv::Mat obj1, 
	cv::Mat obj2, 
	cv::Rect box_obj1, 
	cv::Rect box_obj2)
{
	bool contact = false;
	
	if (obj1.size().width*obj1.size().height > 20 && 
		obj2.size().width*obj2.size().height > 20)// &&
		//(box_obj1 & box_obj2).area() > 0)
	{
		int y,x,y2,x2,tally,counter = 0;
		for (y=0;y<obj1.rows;y++)
		{
			for (x=0;x<obj1.cols;x++)
			{
		  		tally = 0;
		  		if (obj1.data[obj1.cols*y + x]>0)
				{	
					for (y2=y-3;y2<=y+3;y2++)
					{
						for (x2=x-3;x2<=x+3;x2++)
						{
			  				if (obj2.data[obj1.cols*y2+x2]>0) 
							{
								tally = 1;
								break;
							}
						} 
						if (tally == 1) break;
					}				
		  		}
		  		counter += tally;
		  		if (counter > 10) 
				{
					contact = true; 
					break;
				}
			} 	
			if (counter > 10)
			{
				contact = true; 
				break;
			}
		}
  	}
	return contact;
}

// =============================================================================
// contactCheckBox
// =============================================================================
bool contactCheckBox(
	const cv::Mat &img_object_,
	const char *win_name_,
	const int &threshold_,
	bool flag_)
{

	int hs[4];
	hs[0] = 116; hs[1] = 98; hs[2] = 143; hs[3] = 51;

	cv::Mat rgb_tmp, mask;
	cv::Rect box_;
	segmentHSV(img_object_, hs, mask, box_);

	if(flag_)
	{
		//cv::Mat rgb_tmp;
		//img_object_.copyTo(rgb_tmp, mask);
		cv::imshow(win_name_, img_object_);
		cvWaitKey(1);
	}

	if (box_.size().width * box_.size().height > threshold_)
		return true;
	else
		return false;
}

// =============================================================================
// Read file
// =============================================================================
void readFile(
	const char *name,
	std::vector<std::vector<std::string> > &data_full,
	char delimiter)
{
	std::ifstream src_file( name );
	while (src_file)
	{
		std::string file_line_;
		if (!getline( src_file, file_line_ )) break;
		std::istringstream line_( file_line_ );
		std::vector <std::string> data_line_;
		while (line_)
		{
		   std::string word;
		   if (!getline( line_, word, delimiter)) break;
		   data_line_.push_back( word );
		}
		data_full.push_back( data_line_ );
	}

	if (!src_file.eof())
		std::cerr << "FILE ERROR!!!\n";
}


#endif /* UTIL_H_ */
