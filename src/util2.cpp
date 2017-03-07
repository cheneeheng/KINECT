#include "util2.h"

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
		vector<Rect> box(contours.size());
		double biggest_box = 0;
		int big = 0;
		for (int j=0;j<(int)contours.size();j++)
		{
			approxPolyDP(Mat(contours[j]), contours_poly[j], 3, true);
			if (biggest_box < contourArea(contours[j]))
			{
				biggest_box = contourArea(contours[j]);
				box[0] = boundingRect(Mat(contours_poly[j]));
				big = j;
			}
		}
		
		if(biggest_box>2000)
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


