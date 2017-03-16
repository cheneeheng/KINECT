#include "util2.h"

// for location area
void showPrediction(
	Mat &imgHistogram,
	vector<int>data,
	vector<string>label)
{
	rectangle(
			imgHistogram, 
			Point(0,0), Point(640,480), Scalar(70,70,70),-1);
	int sum = std::accumulate(data.begin(), data.end(), 0, absAdd);
	int size = label.size(); 
	for (int i = 0; i < size; ++i) 
	{
		double val = sum > 0 ? data[i]/sum : 0; 
		double nor = 380 - val*330;
		rectangle(
				imgHistogram, 
				Point(((float)(i+0.15)/size*0.8+0.1)*640, 380), 
				Point(((float)(i+0.85)/size*0.8+0.1)*640, nor),
				Scalar(0,val*255,(1-val)*255), -1);

		if(val>0.0)
			rectangle(
					imgHistogram, 
					Point(((float)(i+0.15)/size*0.8+0.1)*640, 380), 
					Point(((float)(i+0.85)/size*0.8+0.1)*640, 50),
					Scalar(0,255,255), 3);
		putText(
				imgHistogram, label[i].c_str(),
				Point(((float)(i+0.17)/size*0.8+0.1)*640, 430), 
				FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);
		putText(
				imgHistogram, to_string((int)(val*100)).c_str(),
				Point(((float)(i+0.17)/size*0.8+0.1)*640, nor-5), 
				FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1,true);
	}
}

// for edges
void showPrediction(
	Mat &imgHistogram,
	vector<double>data,
	vector<string>label)
{
	rectangle(
			imgHistogram, 
			Point(0,0), Point(640,480), Scalar(70,70,70),-1);						
	int max_tmp =
			distance(
					data.begin(),
					max_element( data.begin(), data.end()));
	int size = label.size(); 
	for (int i = 0; i < size; ++i) 
	{
		double val = data[i]; 
		double nor = 380 - val*330;
		rectangle(
				imgHistogram, 
				Point(((float)(i+0.15)/size*0.8+0.1)*640, 380), 
				Point(((float)(i+0.85)/size*0.8+0.1)*640, nor),
				Scalar(0,val*255,(1-val)*255), -1);
		if(i==max_tmp)
			rectangle(
					imgHistogram, 
					Point(((float)(i+0.15)/size*0.8+0.1)*640, 380), 
					Point(((float)(i+0.85)/size*0.8+0.1)*640, 50),
					Scalar(0,255,255), 3);
		putText(
				imgHistogram, label[i].c_str(),
				Point(((float)(i+0.17)/size*0.8+0.1)*640, 430), 
				FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);
		putText(
				imgHistogram, to_string((int)(val*100)).c_str(),
				Point(((float)(i+0.17)/size*0.8+0.1)*640, nor-5), 
				FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1,true);
	}
}



void depthImaging(
	Mat &depth_image, 
	Mat depth_global)
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


void segmentHSVEDIT(
	Mat src,
	Mat& seg_mask,
	int *hs)
{
	Mat src_hsv;
	cvtColor(src,src_hsv,CV_RGB2HSV);
	vector<Mat> splitted_HSV;
	split(src_hsv, splitted_HSV);
	Mat seg_mask1 = Mat::zeros(src_hsv.size(), CV_8UC1);
	Mat seg_mask2 = Mat::zeros(src_hsv.size(), CV_8UC1);
	Mat seg_mask3 = Mat::zeros(src_hsv.size(), CV_8UC1);
	Mat seg_mask4 = Mat::zeros(src_hsv.size(), CV_8UC1);
	// Thresholding
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
}


void noiseRemove(
	Mat seg_mask,
	Mat& seg_mask_noisefree,
	Rect& box)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(seg_mask, contours, hierarchy,
	CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> box2(contours.size());
	double biggest_box = 0;
	int big1 = 0, big2 = 0; 
	for (int j=0;j<(int)contours.size();j++)
	{
		approxPolyDP(Mat(contours[j]), contours_poly[j], 3, true);
		if (biggest_box < contourArea(contours[j]))
		{
			biggest_box = contourArea(contours[j]);
			box2[0] = boundingRect(Mat(contours_poly[j]));
			big1 = j;
		}
	}
	Mat tmp_img = Mat::zeros(seg_mask.size(), CV_8UC1);
	drawContours( tmp_img, contours, big1, 1, -1);
	seg_mask_noisefree = tmp_img;
	box = box2[0];
}

void segmentHSV(
	Mat src,
	int *hs,
	Mat& seg_mask_noisefree,
	Rect& box)
{
	Mat src_hsv, seg_mask;
	cvtColor(src,src_hsv,CV_RGB2HSV);

	vector<Mat> splitted_HSV;
	split(src_hsv, splitted_HSV);

	Mat seg_mask1 = Mat::zeros(src_hsv.size(), CV_8UC1);
	Mat seg_mask2 = Mat::zeros(src_hsv.size(), CV_8UC1);
	Mat seg_mask3 = Mat::zeros(src_hsv.size(), CV_8UC1);
	Mat seg_mask4 = Mat::zeros(src_hsv.size(), CV_8UC1);

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
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(seg_mask, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> box2(contours.size());
	double biggest_box = 0;
	int big1 = 0;
	for (int j=0;j<(int)contours.size();j++)
	{
		approxPolyDP(Mat(contours[j]), contours_poly[j], 3, true);
		if (biggest_box < contourArea(contours[j]))
		{
			biggest_box = contourArea(contours[j]);
			box2[0] = boundingRect(Mat(contours_poly[j]));
			big1 = j;
		}
	}
	Mat tmp_img = Mat::zeros(seg_mask.size(), CV_8UC1);
	drawContours( tmp_img, contours, big1, 1, -1);
	seg_mask_noisefree = tmp_img;
	box = box2[0];
}

void segmentHSV(cv::Mat src_hsv, cv::Mat src_rgb,
                cv::Mat& seg_mask, cv::Mat& seg_rgb,
                int h_top, int h_bot, int s_top, int s_bot){
  std::vector<cv::Mat> splitted_HSV;
  cv::split(src_hsv, splitted_HSV);
  cv::Mat seg_mask1 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
  cv::Mat seg_mask2 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
  cv::Mat seg_mask3 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
  cv::Mat seg_mask4 = cv::Mat::zeros(src_hsv.size(), CV_8UC1);
  // Thresholding
  seg_mask1 = splitted_HSV[0] <= (h_top);
  seg_mask2 = splitted_HSV[0] >= (h_bot);
  seg_mask3 = splitted_HSV[1] <= (s_top);
  seg_mask4 = splitted_HSV[1] >= (s_bot);
  //handling cyclic range of hue

  /*cv::Mat seg_mask_t = (seg_mask1 > 0) & (seg_mask2 > 0) & (seg_mask3 > 0)
                          & (seg_mask4 > 0);*/
  cv::Mat seg_mask_t;
  if (h_top < h_bot){
    seg_mask_t = ((seg_mask1 > 0) | (seg_mask2 > 0)) & (seg_mask3 > 0)
                  & (seg_mask4 > 0);
  }
  else{
    seg_mask_t = (seg_mask1 > 0) & (seg_mask2 > 0) & (seg_mask3 > 0)
                 & (seg_mask4 > 0);
  }
  // mask
  /*cv::Mat seg_mask_t = (seg_mask1 > 0) & (seg_mask2 > 0) & (seg_mask3 > 0)
                         & (seg_mask4 > 0);*/
  cv::Mat seg_rgb_t = cv::Mat::zeros(src_rgb.size(), CV_8UC3);
  src_rgb.copyTo(seg_rgb_t, seg_mask_t);
  seg_mask = seg_mask_t;
  seg_rgb = seg_rgb_t;
}

Rect detectFaceAndEyes(
	Mat &frame,
	CascadeClassifier face_cascade)
{
	vector<Rect> faces;
	Rect face;
	Mat frame_gray;
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	//equalizeHist( frame_gray, frame_gray );
	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 
								  1.2, 2, 0, Size(60, 60), Size(90, 90));
	for(size_t i=0;i<faces.size();i++)
	{
		if (faces[i].area() > face.area())
			face = faces[i];
		Point center(faces[i].x + faces[i].width*0.5, 
					 faces[i].y + faces[i].height*0.5);
		ellipse(frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 
				0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0);
	}
	return face;
}

bool contactCheck(
	Mat obj1, 
	Mat obj2, 
	Rect box_obj1, 
	Rect box_obj2)
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


void pointCloudTrajectory(
	Mat cloud,
	Vec3f &point)
{
	Vec3f single_point,tmp_point;
	float tmp = 5.0;
	int counter = 0;
	for(int i=0;i<cloud.size().height;i++)
	{
		for(int ii=0;ii<cloud.size().width;ii++)
		{
			single_point = cloud.at<Vec3f>(i,ii);
			if(single_point[2]<tmp && single_point[2] > 0)
				tmp = single_point[2];
		}
	}
	for(int i=0;i<cloud.size().height;i++)
	{
		for(int ii=0;ii<cloud.size().width;ii++)
		{
			single_point = cloud.at<Vec3f>(i,ii);
			if(single_point[2] < tmp+0.05 && single_point[2] > tmp-0.05)
			{
				tmp_point += single_point; 
				counter += 1;
			}
		}
	}
	tmp_point = tmp_point/counter;  
	point[0]  = tmp_point[0];
	point[1]  = tmp_point[1];
	point[2]  = tmp_point[2];
}

Vec4f RANSAC3DPlane(
	Mat cloud,
	Mat &plane,
	Rect &box_,
	int iter,
	float *ratio,
	float threshold)
{
	int i,x1,x2,x3,y1,y2,y3,x,y,counter,counter_max;
	double d_def,d_def_best,d_tmp;
	Vec3f p1,p2,p3,p4,plane_norm,plane_best,p_check;

	counter_max = 0;
	srand(time(NULL));

	int stop_num = 100;
	while (counter_max == 0)
	{
		for (int i=0;i<iter;i++)
		{         
			// random points in image plane (##### table lies below the mid line #####)
			x1 = rand() % 640; y1 = rand() % 480;
			x2 = rand() % 640; y2 = rand() % 480;
			x3 = rand() % 640; y3 = rand() % 480;
//    x1 = rand() % 640; y1 = (rand() % 240) +240;
//    x2 = rand() % 640; y2 = (rand() % 240) +240;
//    x3 = rand() % 640; y3 = (rand() % 240) +240;
			// prevent picking the same points
			while(x2==x1 && y2==y1)
			{x2 = rand() % 640; y2 = rand() % 480;}       
			while(x3==x1 && y3==y1 && x3==x2 && y3==y2)   
			{x3 = rand() % 640; y3 = rand() % 480;}
			// random 3d points
			p1 = cloud.at<Vec3f>(y1,x1);  
			p2 = cloud.at<Vec3f>(y2,x2); 
			p3 = cloud.at<Vec3f>(y3,x3); 
			p_check = (p1 - p2).cross((p2 - p3)); // prevent degenerate case
			if (p_check[0]!=0 &&
				p_check[1]!=0 &&
				p_check[2]!=0 && 
				p1[2]>0 && p2[2]>0 && p3[2]>0 && 
				p1[2]<2 && p2[2]<2 && p3[2]<2 )
			{
				counter = 0; 

				// hypothesis
  				plane_norm = (p1 - p2).cross((p2 - p3));
  				plane_norm = plane_norm / norm(plane_norm);

				d_def = plane_norm[0]*p1[0]+plane_norm[1]*p1[1]+plane_norm[2]*p1[2];
				for(y=0;y<480;y++)
				{
					for(x=0;x<640;x++)
					{
						p4 = cloud.at<Vec3f>(y,x); 
						d_tmp = plane_norm.dot(p4);   //offset plane from origin            
						if(abs(d_tmp-d_def)<threshold) counter +=1;              
					}
				}   
				if (counter<ratio[1]*(640*480) && 
					counter>ratio[0]*(640*480) && 
					counter>counter_max)
				{counter_max = counter; plane_best = plane_norm; d_def_best = d_def;}
			}
		}

		// using the best points to build the mask
		for(y=0;y<480;y++)
		{
			for(x=0;x<640;x++)
			{
				p4 = cloud.at<Vec3f>(y,x);               
				d_tmp = plane_best.dot(p4);   //offset plane from origin    
				if (abs(d_tmp-d_def_best)>0 && 
					abs(d_tmp-d_def_best)<threshold && 
					p4[2]<2 && p4[2]>0)
				{
					plane.data[(y*640)+x] = 1;         
				}     
				else
				{
					plane.data[(y*640)+x] = 0;         
				}  
			}
		}

		//denoising
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(plane, contours, hierarchy,
			CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		vector<vector<Point> > contours_poly(contours.size());
		double biggest_box = 0;
		int big = 0;
		for (int j=0;j<(int)contours.size();j++)
		{
			approxPolyDP(Mat(contours[j]), contours_poly[j], 3, true);
			if (biggest_box < contourArea(contours[j]))
			{
				biggest_box = contourArea(contours[j]);
				box_ = boundingRect(Mat(contours_poly[j]));
				big = j;
			}
		}
		
		if(biggest_box>500)
		{	
			Mat tmp_img = Mat::zeros(plane.size(), CV_8UC1);
			drawContours(tmp_img, contours, big, 1, -1);
			plane = tmp_img;
			//imshow("tmp",tmp_img*255);
			break;
		}
		if (stop_num == 0)
		{
			plane *= 0;
			break;
		}
		else
		{
			plane *= 0;
			stop_num--;
			counter_max = 0;
		}
	}

	Vec4f plane_constants;
	plane_constants[0] = plane_best[0];
	plane_constants[1] = plane_best[1];
	plane_constants[2] = plane_best[2];
	plane_constants[3] = d_def_best;
    normalPlaneCheck(plane_constants);

	return plane_constants;
}

void normalPlaneCheck(Vec4f &plane_equation)
{
  Vec3f p1(0,0,1); 
  if (plane_equation[0]*p1[0]+
      plane_equation[1]*p1[1]+
      plane_equation[2]*p1[2]-
      plane_equation[3]< 0)
  {
    plane_equation = (-1) * plane_equation;
  }
}


//====================================================================================================================================
//[TOOLS]**********************************************************************

void mouseCallBackHist(	int eventcode,int x,int y,int flags,void* data){
  if(eventcode == CV_EVENT_LBUTTONDOWN){
    MouseCallBackHistData* mouse_data = (MouseCallBackHistData*)data;
    int x_hist = floor((float)x / (*mouse_data)._hscale);
    int y_hist = floor((float)y / (*mouse_data)._sscale);
    if (x_hist >= (*mouse_data)._hist.cols)
      x_hist = (*mouse_data)._hist.cols - 1;
    if (y_hist >= (*mouse_data)._hist.rows)
      y_hist = (*mouse_data)._hist.rows - 1;
    int h = round(x_hist* (*mouse_data)._hbin_scale);
    int s = round(y_hist* (*mouse_data)._sbin_scale);
    float i = (*mouse_data)._hist.at<float>(x_hist, y_hist);
    mouse_data->_h_mean = h;
    mouse_data->_s_mean = s;
    std::cout << "H: " << h 
              << " +/- " << (*mouse_data)._hbin_scale
              << ", S: " << s 
              << " +/- " << (*mouse_data)._sbin_scale
              << ", Intensity: " << i << std::endl;
  }
}

void trackbarCallBackCalib_H_top(int trackpos, void* data){
  TrackbarCBDataCalib* tb = (TrackbarCBDataCalib*)data;
  tb->_H_top = round(trackpos*179.0 / 100);
  segmentHSV( tb->_hsv, //in
              tb->_seg_in_rgb, //in
              tb->_seg_mask, //out
              tb->_seg_out_rgb, //out
              tb->_H_top, //in
              tb->_H_bot, //in
              tb->_S_top, //in
              tb->_S_bot); //in
  cv::imshow(tb->_wn_seg_mask, tb->_seg_mask);
  cv::imshow(tb->_wn_seg_rgb, tb->_seg_out_rgb);
  std::cout << "[" << "H_top=" << tb->_H_top << ", "
                   << "H_bot=" << tb->_H_bot << ", "
                   << "S_top=" << tb->_S_top << ", "
                   << "S_bot=" << tb->_S_bot << "]\n";
}

void trackbarCallBackCalib_H_bot(int trackpos, void* data){
  TrackbarCBDataCalib* tb = (TrackbarCBDataCalib*)data;
  tb->_H_bot = round(trackpos*179.0 / 100);
  segmentHSV( tb->_hsv, //in
              tb->_seg_in_rgb, //in
              tb->_seg_mask, //out
              tb->_seg_out_rgb, //out
              tb->_H_top, //in
              tb->_H_bot, //in
              tb->_S_top, //in
              tb->_S_bot); //in
  cv::imshow(tb->_wn_seg_mask, tb->_seg_mask);
  cv::imshow(tb->_wn_seg_rgb, tb->_seg_out_rgb);
  std::cout << "[" << "H_top=" << tb->_H_top << ", "
                   << "H_bot=" << tb->_H_bot << ", "
                   << "S_top=" << tb->_S_top << ", "
                   << "S_bot=" << tb->_S_bot << "]\n";
}

void trackbarCallBackCalib_S_top(int trackpos, void* data){
  TrackbarCBDataCalib* tb = (TrackbarCBDataCalib*)data;
  tb->_S_top = round(trackpos*255.0 / 100);
  segmentHSV( tb->_hsv, //in
              tb->_seg_in_rgb, //in
              tb->_seg_mask, //out
              tb->_seg_out_rgb, //out
              tb->_H_top, //in
              tb->_H_bot, //in
              tb->_S_top, //in
              tb->_S_bot); //in
  cv::imshow(tb->_wn_seg_mask, tb->_seg_mask);
  cv::imshow(tb->_wn_seg_rgb, tb->_seg_out_rgb);
  std::cout << "[" << "H_top=" << tb->_H_top << ", "
                   << "H_bot=" << tb->_H_bot << ", "
                   << "S_top=" << tb->_S_top << ", "
                   << "S_bot=" << tb->_S_bot << "]\n";
}

void trackbarCallBackCalib_S_bot(int trackpos, void* data){
  TrackbarCBDataCalib* tb = (TrackbarCBDataCalib*)data;
  tb->_S_bot = round(trackpos*255.0 / 100);
  segmentHSV( tb->_hsv, //in
              tb->_seg_in_rgb, //in
              tb->_seg_mask, //out
              tb->_seg_out_rgb, //out
              tb->_H_top, //in
              tb->_H_bot, //in
              tb->_S_top, //in
              tb->_S_bot); //in
  cv::imshow(tb->_wn_seg_mask, tb->_seg_mask);
  cv::imshow(tb->_wn_seg_rgb, tb->_seg_out_rgb);
  std::cout << "[" << "H_top=" << tb->_H_top << ", "
                   << "H_bot=" << tb->_H_bot << ", "
                   << "S_top=" << tb->_S_top << ", "
                   << "S_bot=" << tb->_S_bot << "]\n";
}

//**********************************************************************[TOOLS]



//====================================================================================================================================

void getColorThreshold(cv::Mat src, int (&hue_range)[2], int (&sat_range)[2]){
  // check and warn if type of src is not CV_8UC3
  if (src.type() != CV_8UC3){
    std::cout
      << "Error: [getColorThreshold()]: src must have type CV_8UC3!\n";
    std::cout << "press any key to exit\n";
    cv::waitKey(0);
    exit(1);
  }
  cv::imshow("src in RGB", src);
  // preproc do medianfilter*************************************************
  cv::Mat src_median_blurred; // median burred src in rgb
  cv::medianBlur(src, src_median_blurred, 5);
  cv::imshow("Blurred", src_median_blurred); //just for debugging

  // RGB -> HSV H=[0...179] S,V=[0...255]
  cv::Mat src_hsv; // median blurred src hsv
  cv::cvtColor(src_median_blurred, src_hsv, CV_RGB2HSV);

  // compute histogram*******************************************************
  int hbins = 30; // please ensure that this is a modulo of the whole range
  int sbins = 16; // please ensure that this is a modulo of the whole range
  int histSize[] = { hbins, sbins };
  float hranges[] = { 0, 180 };
  float sranges[] = { 0, 256 };
  const float* ranges[] = { hranges, sranges };
  int channels[] = { 0, 1 };
  cv::MatND hist;
  calcHist(&src_hsv, 1, channels, cv::Mat(), // do not use mask
               hist, 2, histSize, ranges,
               true, // the histogram is uniform
               false);
  double maxVal = 0;
  minMaxLoc(hist, 0, &maxVal, 0, 0);
  int scale = 20;
  std::vector<cv::Mat> col_hist(3);
  col_hist[0] = cv::Mat::zeros(sbins*scale, hbins * scale, CV_8UC1); //H
  col_hist[1] = cv::Mat::zeros(sbins*scale, hbins * scale, CV_8UC1); //S
  col_hist[2] = cv::Mat::zeros(sbins*scale, hbins * scale, CV_8UC1); //V
  for (int h = 0; h < hbins; h++){
    for (int s = 0; s < sbins; s++)
    {
      float binVal = hist.at<float>(h, s);
      int v_intensity;
      // this is a hack to intensify smaller peaks because if the
      // background is big and uniform then there is a top heavy effect
      if ((((float)binVal) / ((float)maxVal)) > 0.05)
        v_intensity = 255;
      else
        v_intensity = cvRound(binVal * 255 / maxVal);
        cv::rectangle(col_hist[0], cv::Point(h*scale, s*scale),
              cv::Point((h + 1)*scale - 1, (s + 1)*scale - 1),
              cv::Scalar::all(h*(180 / hbins)),
              CV_FILLED);
        cv::rectangle(col_hist[1], cv::Point(h*scale, s*scale),
              cv::Point((h + 1)*scale - 1, (s + 1)*scale - 1),
              //cv::Scalar::all(s*(256/ sbins)),
              cv::Scalar::all(256),
              CV_FILLED);
        cv::rectangle(col_hist[2], cv::Point(h*scale, s*scale),
              cv::Point((h + 1)*scale - 1, (s + 1)*scale - 1),
              cv::Scalar::all(v_intensity),
              CV_FILLED);
    }
  }

  cv::Mat colored_hist_hsv;
  cv::merge(col_hist, colored_hist_hsv);
  cv::Mat colored_hist_rgb;
  cv::cvtColor(colored_hist_hsv, colored_hist_rgb, CV_HSV2RGB);
  std::string hist_win_name = "Colored H-S Histogram";
  cv::imshow(hist_win_name, colored_hist_rgb);
  double s = cv::sum(hist)[0];
  MouseCallBackHistData mouse_data;
  mouse_data._hscale = scale;
  mouse_data._sscale = scale;
  mouse_data._hbin_scale = (180 / hbins);
  mouse_data._sbin_scale = (256 / sbins);
  mouse_data._hist = hist/s;
  cv::setMouseCallback(hist_win_name, mouseCallBackHist, &mouse_data);
  std::cout << "Click on " << hist_win_name
            << "to read off all the hsv values from the console\n";
  std::cout << "press any key to go further\n";
  cv::waitKey(0);
  int mean_h = mouse_data._h_mean;
  int mean_s = mouse_data._s_mean;
  std::cout << "mean hue = " << mean_h 
            << ", mean saturation = " << mean_s << "\n\n";
  cv::destroyAllWindows();
  // calibration*************************************************************
  std::cout << "\n\n...calibration phase...\n\n";
  std::cout << "WARNING [calibration phase]: hue range is polar(cyclic)!!\n";
  cv::imshow("Median Blurred SRC", src_median_blurred); //just for debugging
  //int range_h[2] = { 0, 0 };
  //int range_s[2] = { 0, 0 };
  // create trackbar
  int H_top = round(mean_h * 100 / 180) + 5; // starting value for bar1
  int H_bot = round(mean_h * 100 / 180) - 5; // starting value for bar2
  int S_top = round(mean_s * 100 / 180) + 5; // starting value for bar3
  int S_bot = round(mean_s * 100 / 180) - 5; // starting value for bar4
  std::string calib_win_name = "Color Range Calibration";
  cv::namedWindow(calib_win_name, 1);
  TrackbarCBDataCalib tb_calib;
  tb_calib._hsv = src_hsv;
  tb_calib._seg_in_rgb = src_median_blurred;
  tb_calib._seg_mask = cv::Mat();
  tb_calib._seg_out_rgb = cv::Mat();
  tb_calib._H_top = mean_h + 5;
  tb_calib._H_bot = mean_h - 5;
  tb_calib._S_top = mean_s + 5;
  tb_calib._S_bot = mean_s - 5;
  tb_calib._wn_seg_mask = "seg_mask";
  tb_calib._wn_seg_rgb = "seg_rgb";
  cv::createTrackbar("H_top", calib_win_name, &H_top, 100,
    trackbarCallBackCalib_H_top, &tb_calib);
  cv::createTrackbar("H_bot", calib_win_name, &H_bot, 100,
    trackbarCallBackCalib_H_bot, &tb_calib);
  cv::createTrackbar("S_top", calib_win_name, &S_top, 100,
    trackbarCallBackCalib_S_top, &tb_calib);
  cv::createTrackbar("S_bot", calib_win_name, &S_bot, 100,
    trackbarCallBackCalib_S_bot, &tb_calib);
  cv::resizeWindow(calib_win_name, 500, 100);
  cv::waitKey(30);
  char stopkey = 'a';
  std::cout << "press <s> to stop calibrating.\n";
  segmentHSV(tb_calib._hsv, //in
             tb_calib._seg_in_rgb, //in
             tb_calib._seg_mask, //out
             tb_calib._seg_out_rgb, //out
             tb_calib._H_top, //in
             tb_calib._H_bot, //in
             tb_calib._S_top, //in
             tb_calib._S_bot); //in
  cv::imshow(tb_calib._wn_seg_mask, tb_calib._seg_mask);
  cv::imshow(tb_calib._wn_seg_rgb, tb_calib._seg_out_rgb);
  std::cout << "[" << "H_top=" << tb_calib._H_top << ", "
                   << "H_bot=" << tb_calib._H_bot << ", "
                   << "S_top=" << tb_calib._S_top << ", "
                   << "S_bot=" << tb_calib._S_bot << "]\n";
  while (stopkey != 's'){stopkey = cv::waitKey(0);}
  // extract calibrated values
  cv::destroyAllWindows();
  hue_range[0] = tb_calib._H_bot;
  hue_range[1] = tb_calib._H_top;
  sat_range[0] = tb_calib._S_bot;
  sat_range[1] = tb_calib._S_top;
}




































