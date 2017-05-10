/*
 * main.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: papazov
 */


#include "dataDeclaration.h"
#include "util.h"
#include "util2.h"

// image variables
Mat rgb_global1 = Mat::zeros(480,640,CV_8UC3);
Mat rgb_global2 = Mat::zeros(480,640,CV_8UC3);
Mat rgb_global3 = Mat::zeros(480,640,CV_8UC3);
Mat rgb_global4 = Mat::zeros(480,640,CV_8UC3);

Mat depth_global,cloud_global1,cloud_global2,cloud_global3;

Mat mask_hand_global = Mat::zeros(480,640,CV_8UC1);
Mat mask_obj_global  = Mat::zeros(480,640,CV_8UC1);

Rect box_obj_global, box_hand_global;
Rect face_global(0,0,0,0);

bool contact_obj = false;

Vec3f single_point_obj_global;
Vec3f single_point_face_global;

float frame_number_global = 0.0;

vector<vector<double> > plane_global;

// threads
int MAX = 5;
sem_t mutex1,mutex2,mutex3,mutex4,mutex5,mutex6,mutex7;
sem_t lock_t1,lock_t2,lock_t3,lock_t4,lock_t5,lock_t6;


// option flags
//#define FLAG_RGB // shows window for rgb
//#define FLAG_DEPTH // shows window for depth
//#define FLAG_PLANE // detecting planes
#define FLAG_OBJECT // shows window for object detection
#define FLAG_HAND // shows window for hand detection
//#define FLAG_FACE // shows window for face detection
#define FLAG_CONTACT //shows contact value
#define FLAG_WRITE //records data
//#define FLAG_HISTOGRAM //shows result as histogram

//#define FREQ


//====================================================================================================================================

static inline vector<double> cvVector2vector(Vec4f A)
{
	vector<double> B(4);
	B[0]=A[0];
	B[1]=A[1];
	B[2]=A[2];
	B[3]=A[3];
	return B;
}

static inline vector<double> cvVector2vector(Vec3f A)
{
	vector<double> B(3);
	B[0]=A[0];
	B[1]=A[1];
	B[2]=A[2];
	return B;
}

//====================================================================================================================================

string scene  = "Kitchen";
string object = "04";

int object_num_arg = 0;
bool hsv_arg = false;
bool face_arg = false;
bool surface_arg = false;
bool rgb_arg = false;
bool depth_arg = false;
bool object_arg = false;
bool hand_arg = false;
bool freq_arg = false;

int freq_rate = 30;

// ============================================================================
// THREAD 1 : KINECT 
// ============================================================================
void* kinectGrab(void* v_kinect)
{
	VideoCapture *kinect = reinterpret_cast<VideoCapture *>(v_kinect);

	struct timeval start_time, end_time;
	int c = 0;

	float ratio[2]; ratio[0] = 0.00005; ratio[1] = 0.5;
	char keypress;
    
	Mat plane_tmp = Mat::zeros(480,640,CV_8UC1);
	Mat plane_tmp2 = Mat::zeros(480,640,CV_8UC1);
	Mat tmp_cloud,tmp_cloud2,img;

	bool flag_tmp = true;

	while(true)
	{

		if(freq_arg)
		{
			if(c%freq_rate==0)
			{
				gettimeofday(&start_time, NULL);
			}
		}

		sem_wait(&lock_t1);
		sem_wait(&lock_t1);
		sem_wait(&lock_t1);
		sem_wait(&lock_t1);
		sem_wait(&lock_t1);
		sem_wait(&mutex1);

		kinect->grab();

		kinect->retrieve(rgb_global1,CV_CAP_OPENNI_BGR_IMAGE);
		rgb_global2 = rgb_global1.clone();
		rgb_global3 = rgb_global1.clone();
		rgb_global4 = rgb_global1.clone();

		kinect->retrieve(depth_global,CV_CAP_OPENNI_DEPTH_MAP);
		kinect->retrieve(cloud_global1,CV_CAP_OPENNI_POINT_CLOUD_MAP);
		cloud_global2 = cloud_global1.clone();
		cloud_global3 = cloud_global1.clone();

		frame_number_global =
			kinect->get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_POS_FRAMES);

		if(depth_arg)
		{
			Mat depth_image = Mat::zeros(480,640,CV_8UC3);
			depthImaging(depth_image,depth_global);
			imshow("depth",depth_image);
			cvWaitKey(1);
		}

		if(rgb_arg)
		{
			imshow("rgb",rgb_global1);
			cvWaitKey(1);
		}

		while(hsv_arg)
		{
			imwrite( "../test.png" , rgb_global1 );
			int hue_range[2], sat_range[2];
			Mat img = imread("../test.png");
			getColorThreshold(img, hue_range, sat_range);
			printf("Final Calibration values:\nhue = %d %d\nsat = %d %d\n",
					hue_range[0],hue_range[1],
					sat_range[0],sat_range[1]);
			imshow("rgb",img);
			cout << "press <s> to exit.\n";
			keypress = waitKey(0);
			if (keypress == 'q' || keypress == 's')
			{
				hsv_arg = false;
				break;
			}
		}
 
		// [SURFACE DETECTION]*************************************************
		tmp_cloud = cloud_global1.clone();
		while(surface_arg)
		{       
			Rect box;
			plane_tmp  = Mat::zeros(480,640,CV_8UC1);
			plane_tmp2 = Mat::zeros(480,640,CV_8UC1);
			Vec4f plane_eq = 
				RANSAC3DPlane(tmp_cloud, plane_tmp, box, 1000, ratio, 0.003);
			imshow("plane", plane_tmp*255);

			vector<double> tmpeq = cvVector2vector(plane_eq);
			vector<double> tmpmp = 
				cvVector2vector(
					tmp_cloud.at<Vec3f>((box.br().y+box.tl().y)/2,
										(box.br().x+box.tl().x)/2));

			vector<double> boxmin; boxmin.resize(3);
			vector<double> boxmax; boxmax.resize(3);
			vector<double> boxtmp;
			bool flagfirst = true;
			for(int y=0;y<480;y++)
			{
				for(int x=0;x<640;x++)
				{
					if (norm(tmp_cloud.at<Vec3f>(y,x))!=0) 
					{
						plane_tmp2.data[(y*640)+x] = 1;  /*
						boxtmp = cvVector2vector(tmp_cloud.at<Vec3f>(y,x));
						if(flagfirst)
						{
							flagfirst = false;
							boxmin=boxmax=tmpmp;
						}
						else
						{
							if (boxtmp[0]>boxmax[0] && boxtmp[0]<1.0 && 
								boxtmp[1]>boxmax[1] && boxtmp[1]<1.0 && 
								boxtmp[2]>boxmax[2]&&boxtmp[2]<2.0)
								boxmax = boxtmp;
							if (boxtmp[0]<boxmin[0] && boxtmp[0]>-1.0 &&
								boxtmp[1]<boxmin[1] &&
								boxtmp[2]<boxmin[2])
								boxmin = boxtmp;
						}*/
					}
				}
			}
			imshow("plane_reduced", plane_tmp2*255);

			boxmax = 
				cvVector2vector(
					tmp_cloud.at<Vec3f>(box.tl().y + ((box.br().y-box.tl().y)*0.2),
										box.tl().x + ((box.br().x-box.tl().x)*0.2)));
			boxmin = 
				cvVector2vector(
					tmp_cloud.at<Vec3f>(box.br().y - ((box.br().y-box.tl().y)*0.2),
										box.br().x - ((box.br().x-box.tl().x)*0.2)));

			printf(" %.4f %.4f %.4f \n %.4f %.4f %.4f \n %.4f %.4f %.4f \n %.4f %.4f %.4f %.4f \n",
					boxmin[0], boxmin[1], boxmin[2],
					tmpmp[0], tmpmp[1], tmpmp[2],
					boxmax[0], boxmax[1], boxmax[2],
					plane_eq[0], plane_eq[1], plane_eq[2], plane_eq[3]);

			if (countNonZero(plane_tmp) < 1) 
				printf("NO PLANE FOUND... \n\n");
			else			
				printf("SAVE NORMAL VECTOR OF PLANE : [Y/N] \n\n");

			keypress = waitKey(0); 
			if (keypress == 'y') 
			{
				tmpeq.insert(tmpeq.end(),tmpmp.begin(),tmpmp.end());
				tmpeq.insert(tmpeq.end(),boxmin.begin(),boxmin.end());
				tmpeq.insert(tmpeq.end(),boxmax.begin(),boxmax.end());
				plane_global.push_back(tmpeq);
				for(int y=0;y<480;y++)
					for(int x=0;x<640;x++)
						tmp_cloud.at<Vec3f>(y,x) *= 
							(int)(plane_tmp.data[(y*640)+x]==0);
			} 
			else if (keypress == 'x') 
			{
				tmpeq.insert(tmpeq.end(),tmpmp.begin(),tmpmp.end());
				plane_global.push_back(tmpeq);
			} 
			else if (keypress == 'q')
			{
				writeSurfaceFile(plane_global);
				surface_arg = false;
				break;
			}
			else if (keypress == 'd')
			{
				for(int y=0;y<480;y++)
					for(int x=0;x<640;x++)
						tmp_cloud.at<Vec3f>(y,x) *= 
							(int)(plane_tmp.data[(y*640)+x]==0);
			}
		}
		
		destroyWindow("plane");
		destroyWindow("plane_reduced");
		// *************************************************[SURFACE DETECTION]

		sem_post(&mutex1);
		sem_post(&lock_t2);
		sem_post(&lock_t3);
		sem_post(&lock_t4);
		sem_post(&lock_t5);
		sem_post(&lock_t6);

		if(freq_arg)
		{
			c++;
			if(c%freq_rate==0)
			{
				c = 0;
				gettimeofday(&end_time, NULL);
				printf("Frequency : %f [Hz]",
						freq_rate /
						((end_time.tv_sec - start_time.tv_sec) + 
						 (end_time.tv_usec- start_time.tv_usec) * 1e-6));
			}
		}
	}
	return 0;
}

// ============================================================================
// THREAD 2 : OBJECT DETECTOR
// ============================================================================
void* objectDetector(void* arg)
{
//	int hue_range_obj[2], sat_range_obj[2];
//	// yellow plyers
//	hue_range_obj[0] = 80; hue_range_obj[1] = 102;
//	sat_range_obj[0] = 135; sat_range_obj[1] = 255;
//	// green cup
//	hue_range_obj[0] = 77; hue_range_obj[1] = 98;
//	sat_range_obj[0] = 76; sat_range_obj[1] = 214;
//red bar
//  hue_range_obj[0] = 116; hue_range_obj[1] = 138;
//  sat_range_obj[0] = 199; sat_range_obj[1] = 255;

	int hs[4]; // hue max/min, sat max/min
	switch(object_num_arg)
	{
		case 0:
			hs[0] = 98; hs[1] = 77; hs[2] = 214; hs[3] = 76; // green cup
			hs[0] = 99; hs[1] = 36; hs[2] = 153; hs[3] = 51; // green cup
			hs[0] = 90; hs[1] = 72; hs[2] = 140; hs[3] = 56; // green cup
			break;
		case 1:
			hs[0] = 107; hs[1] = 90; hs[2] = 204; hs[3] = 140;
			hs[0] = 107; hs[1] = 90; hs[2] = 255; hs[3] = 140;
			//hs[0] = 107; hs[1] = 72; hs[2] = 204; hs[3] = 102; // yellow sponge
			hs[0] = 98; hs[1] = 90; hs[2] = 230; hs[3] = 140; // yellow sponge
			break;
		case 2:
			hs[0] = 125; hs[1] = 116; hs[2] = 255; hs[3] = 179; // red knife
			hs[0] = 125; hs[1] = 98; hs[2] = 255; hs[3] = 166; // red knife
			hs[0] = 134; hs[1] = 113; hs[2] = 255; hs[3] = 143; // red knife
			hs[0] = 125; hs[1] = 116; hs[2] = 237; hs[3] = 184; // red knife
			break;
		case 3:
			hs[0] = 111; hs[1] = 98; hs[2] = 255; hs[3] = 153; // orange
			//hs[0] = 116; hs[1] = 98; hs[2] = 255; hs[3] = 204; // orange
			hs[0] = 107; hs[1] = 100; hs[2] = 255; hs[3] = 204; // orange
			break;
		default:
			hs[0] = 107; hs[1] = 90; hs[2] = 204; hs[3] = 140; 
			break;
	}

	while(true)
	{
		sem_wait(&lock_t2);
		sem_wait(&mutex2);

		segmentHSV(rgb_global2, hs, mask_obj_global, box_obj_global);

		if(object_arg)
		{
			Mat rgb_tmp = Mat::zeros(480,640, CV_8UC3);
			rgb_global1.copyTo(rgb_tmp, mask_obj_global);
			imshow("rgb_o",rgb_tmp);
			cvWaitKey(1);
		}

		sem_post(&mutex2);
		sem_post(&lock_t1);
	}

	return 0;
}

// ============================================================================
// THREAD 3 : HAND DETECTOR
// ============================================================================
void* handDetector(void* arg)
{ 
//	// Crop Threshold
//	int hue_range_hand[2], sat_range_hand[2];
//	hue_range_hand[0] = 102; hue_range_hand[1] = 122;
//	sat_range_hand[0] = 69 ; sat_range_hand[1] = 150;

	int hs[4]; // hue max/min, sat max/min
	//hs[0] = 122; hs[1] = 102; hs[2] = 150; hs[3] = 69;
	//hs[0] = 118; hs[1] = 104; hs[2] = 128; hs[3] = 77;

	hs[0] = 116; hs[1] = 98; hs[2] = 140; hs[3] = 64;
	hs[0] = 125; hs[1] = 98; hs[2] = 140; hs[3] = 77;

	Mat img_no_head = Mat::zeros(480,640,CV_8UC3);

	while(true)
	{
		sem_wait(&lock_t3);
		sem_wait(&mutex3);
/*
		face_global.y     -= face_global.height * 0.1;
		face_global.height = face_global.height * 1.3;
		rgb_global3(face_global) = 0;
*/
      	rgb_global3.clone().rowRange(200,480).copyTo(img_no_head.rowRange(200,480));
		segmentHSV(img_no_head, hs, mask_hand_global, box_hand_global);

		if(hand_arg)
		{
			Mat rgb_tmp = Mat::zeros(480,640, CV_8UC3);
			rgb_global1.copyTo(rgb_tmp, mask_hand_global);
			imshow("rgb_h",rgb_tmp); cvWaitKey(1);
		}

		sem_post(&mutex3);
		sem_post(&lock_t1);
	}

	return 0;
}

// ============================================================================
// THREAD 4 : FACE DETECTOR
// ============================================================================
void* faceDetector(void* arg)
{
	//Load the cascade for face detector
	string face_cascade_name = "../cascade/lbpcascade_frontalface.xml";
	CascadeClassifier face_cascade;
	if(!face_cascade.load(face_cascade_name))
	printf("--(!)Error loading face cascade\n");

	Mat img_depth_def, mask_obj_def, img_sub, cloud_mask, cloud_mask2, img_tmp;

	bool NANflag = true;

	while(true)
	{
		sem_wait(&lock_t4);
		sem_wait(&mutex4);

		// just to flush out some frames
		if (frame_number_global<50 || NANflag)
		{
			if (!(countNonZero(rgb_global4!=img_tmp) == 0))
			{
				img_tmp.release();
				img_tmp = rgb_global4.clone();
				face_global = detectFaceAndEyes(rgb_global4, face_cascade);
				//[OBJECT POINT]***************************************************
				cloud_global2(face_global).copyTo(cloud_mask); // reducing the search area
				pointCloudTrajectory(cloud_mask, single_point_face_global);
				NANflag = 
					(isnan(single_point_face_global[0]) ||
					 isnan(single_point_face_global[1]) ||
					 isnan(single_point_face_global[2]));
				cloud_mask.release(); 
				// **************************************************[OBJECT POINT]
			}
		}
		else if (face_arg)
		{
			if (!(countNonZero(rgb_global4!=img_tmp) == 0))
			{
				img_tmp.release();
				img_tmp = rgb_global4.clone();
				face_global = detectFaceAndEyes(rgb_global4, face_cascade);
				//[OBJECT POINT]***************************************************
				cloud_global2(face_global).copyTo(cloud_mask); // reducing the search area
				pointCloudTrajectory(cloud_mask, single_point_face_global);
				cloud_mask.release(); 
				// **************************************************[OBJECT POINT]
			}
			imshow("face",rgb_global4);
			cvWaitKey(1);
		}

		sem_post(&mutex4);
		sem_post(&lock_t6);
		sem_post(&lock_t1);
	}
	return 0;
}

// ============================================================================
// THREAD 5 : CONTACT DETECTOR
// ============================================================================
void* contactDetector(void* arg)
{
	Mat img_depth_def, mask_obj_def, img_sub, cloud_mask,cloud_mask2;
	double contact_val;
	bool flag_contact_init	= true;
	bool flag_contact_obj	= false;
	int c = 0;
	while(true)
	{
		sem_wait(&lock_t5);
		sem_wait(&mutex5);

		// [DEFAULT SCENE]*****************************************************
		if(flag_contact_init)
		{
			mask_obj_def = mask_obj_global.clone();
			depth_global.copyTo(img_depth_def,mask_obj_def);
			if(box_obj_global.x > 0 && box_obj_global.y > 0) 
				flag_contact_init = false;
		}
		// *****************************************************[DEFAULT SCENE]

		//[OBJECT POINT]*******************************************************
		cloud_global3.copyTo(cloud_mask,mask_obj_global); //taking the obj only
		cloud_mask(box_obj_global).copyTo(cloud_mask2); // reducing the search area
		pointCloudTrajectory(cloud_mask2, single_point_obj_global);
		cloud_mask.release(); 
		cloud_mask2.release();
		// ******************************************************[OBJECT POINT]

		// [OBJECT CONTACT]****************************************************
		if(contactCheck(mask_hand_global, mask_obj_global,
						box_hand_global, box_obj_global))
		{
			if(!flag_contact_obj)
			{
				depth_global.copyTo(img_sub,mask_obj_def);			
				absdiff(img_depth_def,img_sub,img_sub);
				contact_val = sum(img_sub)[0] / sum(mask_obj_def)[0];

				if(contact_val > 0 && contact_val < 500)
				{
					contact_obj 		= true;
					flag_contact_obj 	= true;
				}
				else contact_obj = false;
			}
			else contact_obj = true;     
		}
		else
		{
			flag_contact_init 	= true;
			flag_contact_obj 	= false;
			contact_obj 		= false;
			contact_val			= 0.0;
		}
		// face prevention
		if(box_obj_global.tl().y < 201) 
		{
			flag_contact_init	= false;
			flag_contact_obj	= true;
			contact_obj 		= true;
			contact_val			= -1.0;
		} 
		// ****************************************************[OBJECT CONTACT]

#ifdef FLAG_CONTACT
		c++;
		if(c%freq_rate==0)
		{
			//printf("c %d\n", contact_obj);
			printf("CONTACT : %d , CONTACTVAL : %f\n", contact_obj, contact_val);
		}
#endif

		sem_post(&mutex5);
		sem_post(&lock_t6);
		sem_post(&lock_t1);
	}
	return 0;
}

// ============================================================================
// THREAD 6 : WRITE DATA
// ============================================================================
void* writeData(void* arg)
{

#ifdef FLAG_WRITE

// [RECORD DATA]***************************************************************
	ofstream write_file;
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%y%m%d%H%M%S", &tstruct);
	string file_name = "../recording/";
	file_name += buf;
	file_name += ".txt";
	while(true)
	{
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&mutex6);
		if(frame_number_global>50)
		{
			// write values into data.txt
			ofstream write_file(file_name.c_str(), std::ios::app);
			write_file 	<< frame_number_global			<< ","
						<< contact_obj 					<< ","
						<< single_point_obj_global[0]	<< ","
						<< single_point_obj_global[1]	<< ","
						<< single_point_obj_global[2]	<< ","
						<< single_point_face_global[0]	<< ","
						<< single_point_face_global[1]	<< ","
						<< single_point_face_global[2]
						<< "\n";
		}
		sem_post(&mutex6);
		sem_post(&lock_t1);
	}
// ***************************************************************[RECORD DATA]



#else



// [PREDICTION]****************************************************************
	// [VARIABLES]*************************************************************
	Graph Graph_(scene, object);

	int num_points 			= 0;
	int num_locations		= 0;
	int num_surfaces		= 0;
	int file_num 			= 0;
	int minpts 				= 10;
	double epsilon 			= 0.015; 	// if datasets are merged these 2 values can be increased.

	limit_t LIMIT;
	LIMIT.vel 				= 0.005;
	LIMIT.sur_dis 			= 0.075;
	LIMIT.sur_ang 			= 0.95;

	point_t 				curr_point;
	vector<point_t>			pos_vel_acc_avg(3); // motion

	bool flag_motion      	= false;
	bool flag_predict      	= false;
	bool flag_predict_last 	= false;
	bool learn 				= false;
	bool slide 				= false;
	int loc_last 			= 0;
	int surface_num_tmp 	= 0;
	double pow_dec 			= 1;

	string path;	
	vector<vector<string> > data;

	pred_t prediction;

	label_t LABEL;
	vector<string> label;

	msg_t MSG, MSG_last;

	vector<int> 				loc_last_idxs;
	vector<double> 				t_val;
	vector<vector< point_t > > 	pos_vel_acc_mem; // motion->length
	vector<vector< point_t > > 	pva_avg;
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	readSurfaceFile(Graph_);
	printf("Reading test data......Complete\n");
	// *************************************************************[READ FILE]

	// [LEARNED DATA]**********************************************************
	// [NODES]*****************************************************************
	data.clear();
	path =  "../Scene/" + scene + "/" + object + "/data_mov.txt";
	readFile(path.c_str(), data , ',');
	readMovement (Graph_, data);
	label.clear(); label = Graph_.getMovLabel();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph_.updateMovLabel(label);
		writeLabelFile(Graph_, path, 1);
	}
	data.clear();
	path =  "../Scene/" + scene + "/" + object + "/data_loc.txt";
	readFile(path.c_str(), data , ',');
	readLocation_(Graph_, data);
	label.clear(); label = Graph_.getNodeName();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph_.updateNodeName(label);
		writeLabelFile(Graph_, path, 0);
	}
	printf("Creating nodes for the clusters (action locations)......Complete\n");
	// *****************************************************************[NODES]
	readSectorFile  (Graph_, 0);
	readSectorFile  (Graph_, 1);
	readLocationFile(Graph_, 0);
	readLocationFile(Graph_, 1);
	readLocationFile(Graph_, 2);
	readLocationFile(Graph_, 3);
	readLocationFile(Graph_, 4);
	readCounterFile (Graph_, 0);
	num_locations = Graph_.getNodeList().size();
	num_surfaces  = Graph_.getSurface ().size();
	// **********************************************************[LEARNED DATA]

	// [PREDICTION VARIABLES]**************************************************
	reshapeVector(prediction.pred, 			num_locations);
	reshapeVector(prediction.pred_in,		num_locations);
	reshapeVector(prediction.pred_in_last, 	num_locations);
	reshapeVector(prediction.pred_err,		num_locations);
//	reshapeVector(pos_vel_acc_avg, 			num_points);
	reshapeVector(pos_vel_acc_mem, 			3);
	reshapeVector(loc_last_idxs, 			num_locations);
	reshapeVector(LABEL.loc, 				num_locations);
	reshapeVector(LABEL.sur, 				num_surfaces);
	LABEL.mov   = -1;
	MSG.num_loc = num_locations;
	MSG.num_sur = num_surfaces;
	// **************************************************[PREDICTION VARIABLES]

	printf("\n\n>>>>> SYSTEM START <<<<<\n\n");

	Mat imgHistogram(480,640,CV_8UC3);	

	int step = -1;
	int c = 0;
	bool zeros = false;
	while(true)
	{
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&mutex6);

if(frame_number_global>100)
{

		step++;

		//[PREPROCESS DATA]****************************************************
		curr_point.x = single_point_obj_global[0];
		curr_point.y = single_point_obj_global[1];
		curr_point.z = single_point_obj_global[2];
		curr_point.cluster_id = UNCLASSIFIED;
		preprocessDataLive(curr_point, pos_vel_acc_mem,
						   pos_vel_acc_avg, FILTER_WIN);
		//****************************************************[PREPROCESS DATA]

// ============================================================================
// PREDICTION STARTS
// ============================================================================
		zeros 		= false;
		slide 	  	= false;
		LABEL.mov 	= -1;
		MSG.idx   	= step;
		reshapeVector(LABEL.loc, num_locations);
		reshapeVector(LABEL.sur, num_surfaces);

		// 1. Contact trigger
		// 1.1 Check if the object is within a sphere volume of the location areas
		triggerContact(pos_vel_acc_avg[0], Graph_);

		// 2. Prediction during motion
		if (pos_vel_acc_avg[0].cluster_id < 0)
		{
			pva_avg.push_back(pos_vel_acc_avg);
			flag_motion = true;

			predictionEdge(prediction, Graph_,
					pos_vel_acc_avg[0], pos_vel_acc_avg[1],
					loc_last, loc_last_idxs, LIMIT, LABEL,
					flag_predict, flag_predict_last, pow_dec);

			zeros =
					all_of(
							prediction.pred_err.begin(),
							prediction.pred_err.end(),
							[](double ii) { return ii==0.0; });

			if (!zeros)
			{
				MSG.msg 	= 1;
				MSG.label 	= LABEL;
				MSG.loc_idx = pos_vel_acc_avg[0].cluster_id;
				MSG.pred	= prediction;

				if(c%freq_rate==0)
				outputMsg(MSG, Graph_);

				showPrediction(imgHistogram, MSG.pred.pred_err, label);
				imshow("hist", imgHistogram);
				waitKey(1);
			}
			else
			{
				MSG 		= MSG_last;
				cout << "Outside : ";
				for(int i=0;i<6;i++)
				cout << MSG.label.loc[i];
				cout << endl;
				MSG.msg		= 2;		
				MSG.idx   	= step;		
				if(c%freq_rate==0)
				outputMsg(MSG, Graph_);
				showPrediction(imgHistogram, MSG.label.loc, label);
				imshow("hist", imgHistogram);
				waitKey(1);
			}
		}

		// 3. Prediction within location area
		else
		{
			flag_predict      = false;
			flag_predict_last = false;
			reshapeVector(loc_last_idxs,num_locations);

			predictionNode(
					pva_avg, pos_vel_acc_avg[0], pos_vel_acc_avg[1],
					loc_last, pos_vel_acc_avg[0].cluster_id,
					Graph_, LIMIT, LABEL,
					flag_motion, learn);

			loc_last 	= pos_vel_acc_avg[0].cluster_id;
			flag_motion = false;

			pva_avg.clear();

			MSG.msg 	= 2;
			MSG.label 	= LABEL;
			MSG.loc_idx = pos_vel_acc_avg[0].cluster_id;
			MSG.pred 	= prediction;
			if(c%freq_rate==0)
			outputMsg(MSG, Graph_);

			showPrediction(imgHistogram, MSG.label.loc, label);
			imshow("hist", imgHistogram);
			waitKey(1);
		}

		MSG_last = MSG;

		MSG.msg 	= 3;
		if (pos_vel_acc_avg[0].cluster_id < 0)
		{
			if (!zeros)
			{
				MSG.label 	= LABEL;
				MSG.loc_idx = pos_vel_acc_avg[0].cluster_id;
				MSG.pred 	= prediction;
			}
		}
		else
		{
			MSG.label 	= LABEL;
			MSG.loc_idx = pos_vel_acc_avg[0].cluster_id;
			MSG.pred 	= prediction;
		}
		if(c%freq_rate==0)
		outputMsg(MSG, Graph_);

// ============================================================================
// PREDICTION ENDS
// ============================================================================

		c++;


}
		sem_post(&mutex6);
		sem_post(&lock_t1);
	}
// ****************************************************************[PREDICTION]

#endif

  return 0;
}

// ============================================================================
// >>>>> MAIN <<<<<
// ============================================================================

//struct cmp_str
//{
//   bool operator()(char const *a, char const *b)
//   {
//	  return std::strcmp(a, b) < 0;
//   }
//};

int main(int argc, char *argv[])
{
	argc-=1;

	map<string,int> mapper;
	mapper["green_cup"] 	= 0;
	mapper["yellow_sponge"]	= 1;
	mapper["red_knife"] 	= 2;
	mapper["orange"] 		= 3;

	if(argc%2==1)
	{
		printf("Wrong number of input arguments...\n");
		return 0;
	}

	for(int i=0;i<argc;i++)
	{
		if(i%2==1) { continue; }

		if (!strcmp(argv[i+1],"-object"))
		{
			if (!strcmp(argv[i+2],"true"))
			{
				object_arg = true;
			}
			else
			{
				if (strcmp(argv[i+2],"green_cup") &&
					strcmp(argv[i+2],"yellow_sponge") &&
					strcmp(argv[i+2],"red_knife") &&
					strcmp(argv[i+2],"orange"))
				{
					cerr << "Object name is invalid..." << endl;
					return 0;
				}
				else
				{
					object_num_arg = mapper[string(argv[i+2])];
				}
			}
		}
		else if(!strcmp(argv[i+1],"-hsv") &&
				!strcmp(argv[i+2],"true"))
		{
			hsv_arg = true;
		}
		else if(!strcmp(argv[i+1],"-face") &&
				!strcmp(argv[i+2],"true"))
		{
			face_arg = true;
		}
		else if(!strcmp(argv[i+1],"-surface") &&
				!strcmp(argv[i+2],"true"))
		{
			surface_arg = true;
		}
		else if(!strcmp(argv[i+1],"-rgb") &&
				!strcmp(argv[i+2],"true"))
		{
			rgb_arg = true;
		}
		else if(!strcmp(argv[i+1],"-depth") &&
				!strcmp(argv[i+2],"true"))
		{
			depth_arg = true;
		}
		else if(!strcmp(argv[i+1],"-hand") &&
				!strcmp(argv[i+2],"true"))
		{
			hand_arg = true;
		}
	}

	VideoCapture kinect(CV_CAP_OPENNI2); 
	printf("Starting Kinect ...\n");

  // Run the visualization

	if(depth_arg)
	{
		namedWindow("depth");
		moveWindow("depth",0,0);
	}

	if(rgb_arg)
	{
		namedWindow("rgb");
		moveWindow("rgb",0,550);  
	}

	if(surface_arg)
	{
		namedWindow("plane");
		moveWindow("plane",0,0);
		namedWindow("plane_reduced");
		moveWindow("plane_reduced",0,550);  
	}

	if(hand_arg)
	{
	  namedWindow("rgb_h");
	  moveWindow("rgb_h",0,0);
	}

	if(object_arg)
	{
	  namedWindow("rgb_o");
	  moveWindow("rgb_o",0,550);  
	}

	if(face_arg)
	{
		namedWindow("face");
		moveWindow("face",0,0);
	}


#ifdef FLAG_MARKER
  namedWindow("rgb_m");
#endif

#ifdef FLAG_HISTOGRAM
  namedWindow("hist");
  moveWindow("hist",0,0);
#endif

	pthread_t 	thread_kinectGrab,
				thread_objDetector,
				thread_handDetector,
				thread_faceDetector,
				thread_contactDetector,
				thread_writeData;

	sem_init(&lock_t1, 0, MAX);
	sem_init(&lock_t2, 0, 0);
	sem_init(&lock_t3, 0, 0);
	sem_init(&lock_t4, 0, 0);
	sem_init(&lock_t5, 0, 0);
	sem_init(&lock_t6, 0, 0);
	sem_init(&mutex1, 0, 1);
	sem_init(&mutex2, 0, 1);
	sem_init(&mutex3, 0, 1);
	sem_init(&mutex4, 0, 1);
	sem_init(&mutex5, 0, 1);
	sem_init(&mutex6, 0, 1);
	sem_init(&mutex7, 0, 1);

	pthread_attr_t attr;
	cpu_set_t cpus;
	pthread_attr_init(&attr);

	CPU_ZERO(&cpus);
	CPU_SET(1, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_kinectGrab, &attr, kinectGrab, &kinect);

	CPU_ZERO(&cpus);
	CPU_SET(2, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_objDetector, &attr, objectDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(3, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_handDetector, &attr, handDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(4, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_faceDetector, &attr, faceDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(5, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_contactDetector, &attr, contactDetector, NULL);

	CPU_ZERO(&cpus);
	CPU_SET(6, &cpus);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpus);
	pthread_create(&thread_writeData, &attr, writeData, NULL);

	pthread_join(thread_kinectGrab, NULL);
	pthread_join(thread_objDetector, NULL);
	pthread_join(thread_handDetector, NULL);
	pthread_join(thread_faceDetector, NULL);
	pthread_join(thread_contactDetector, NULL);
	pthread_join(thread_writeData, NULL);

	//printf("MAIN THREAD ON CORE : %d\n",sched_getcpu()); 

	return 0;
}





















