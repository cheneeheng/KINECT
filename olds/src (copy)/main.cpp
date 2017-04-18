/*
 * main.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: papazov
 */


#include "dataDeclaration.h"
#include "algo.h"
#include "util.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"

// image variables
Mat rgb_global1 = Mat::zeros(480,640,CV_8UC3);
Mat rgb_global2 = Mat::zeros(480,640,CV_8UC3);
Mat rgb_global3 = Mat::zeros(480,640,CV_8UC3);
Mat rgb_global4 = Mat::zeros(480,640,CV_8UC3);

Mat depth_global,cloud_global;

Mat mask_hand_global = Mat::zeros(480,640,CV_8UC1);
Mat mask_obj_global  = Mat::zeros(480,640,CV_8UC1);

Rect box_obj_global, box_hand_global;
Rect face_global(0,0,0,0);

bool contact_obj = false;

Vec3f single_point_obj_global;

float frame_number_global = 0.0;

vector<Vec4f> plane_global;

// threads
int MAX = 4;
sem_t mutex1,mutex2,mutex3,mutex4,mutex5,mutex6,mutex7;
sem_t lock_t1,lock_t2,lock_t3,lock_t4,lock_t5,lock_t6;


// option flags
#define FLAG_RGB
#define FLAG_DEPTH
//#define FLAG_MARKER
#define FLAG_PLANE

#define FLAG_OBJECT // shows window for object detection
#define FLAG_HAND // shows window for hand detection
//#define FLAG_FACE // shows window for face detection
//#define FLAG_CONTACT //shows contact value
//#define FLAG_WRITE //records data

//#define FREQ


//====================================================================================================================================

unsigned int window = 5;
vector<string> LABEL_MOV;
vector<string> LABEL_LOC_MOV;
vector<string> LABEL_LOC;

int freq_rate = 30;

// ============================================================================
// THREAD 1 : KINECT 
// ============================================================================
void* kinectGrab(void* v_kinect)
{
	VideoCapture *kinect = reinterpret_cast<VideoCapture *>(v_kinect);

	struct timeval start_time, end_time;
	int c = 0;

	bool flag_plane = false;

	float ratio[2]; ratio[0] = 0.00005; ratio[1] = 0.5;
	char keypress;
    
	Mat plane_tmp = Mat::zeros(480,640,CV_8UC1);
	Mat plane_tmp2 = Mat::zeros(480,640,CV_8UC1);
	Mat tmp_cloud,tmp_cloud2;

	while(true)
	{

#ifdef FREQ
		if(c%freq_rate==0) gettimeofday(&start_time, NULL);
#endif

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
		kinect->retrieve(cloud_global,CV_CAP_OPENNI_POINT_CLOUD_MAP);

		frame_number_global =
			kinect->get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_POS_FRAMES);

#ifdef FLAG_DEPTH
		Mat depth_image = Mat::zeros(480,640,CV_8UC3);
		depthImaging(depth_image,depth_global);
		imshow("depth",depth_image); cvWaitKey(1);
#endif

#ifdef FLAG_RGB
		imshow("rgb",rgb_global1); cvWaitKey(1);
#endif
 
#ifdef FLAG_PLANE
		// [SURFACE DETECTION]*************************************************
		tmp_cloud = cloud_global.clone();
		while(!flag_plane)
		{       
			plane_tmp  = Mat::zeros(480,640,CV_8UC1);
			plane_tmp2 = Mat::zeros(480,640,CV_8UC1);
			Vec4f plane_eq = 
				RANSAC3DPlane(tmp_cloud, plane_tmp, 500, ratio, 0.003);
			imshow("plane", plane_tmp*255);

			for(int y=0;y<480;y++)
				for(int x=0;x<640;x++)
					if (norm(tmp_cloud.at<Vec3f>(y,x))!=0) 
						plane_tmp2.data[(y*640)+x] = 1;  
			imshow("plane_reduced", plane_tmp2*255);

			printf("SAVE NORMAL VECTOR OF PLANE : [Y/N] \n\n");
			keypress = waitKey(0); 
			if (keypress == 'y') 
			{
				plane_global.push_back(plane_eq);

				for(int y=0;y<480;y++)
					for(int x=0;x<640;x++)
						tmp_cloud.at<Vec3f>(y,x) *= 
							(int)(plane_tmp.data[(y*640)+x]==0);  

			} 
			else if (keypress == 'q')
			{
				vector<vector<double> > tmptmp;
				for(int i=0;i<plane_global.size();i++)
					tmptmp.push_back(cvVector2vector(plane_global[i]));
				writeSurfaceFile(tmptmp);
				flag_plane = true;
			}
		}
		// *************************************************[SURFACE DETECTION]
#endif

		sem_post(&mutex1);
		sem_post(&lock_t2);
		sem_post(&lock_t3);
		sem_post(&lock_t5);
		sem_post(&lock_t6);

#ifdef FREQ
		c++;
		if(c%freq_rate==0)
		{
			c = 0;
			gettimeofday(&end_time, NULL);
			cout << freq_rate/ ((end_time.tv_sec - start_time.tv_sec) + 
					(end_time.tv_usec- start_time.tv_usec) * 1e-6) 
				 << " [Hz]"<< endl;
		}
#endif

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

	int hs[4]; // hue max/min, sat max/min
	hs[0] = 98;
	hs[1] = 77;
	hs[2] = 214;
	hs[3] = 76;

	while(true)
	{
		sem_wait(&lock_t2);
		sem_wait(&mutex2);

		segmentHSV(rgb_global2, hs, mask_obj_global, box_obj_global);

#ifdef FLAG_OBJECT
		Mat rgb_tmp = Mat::zeros(480,640, CV_8UC3);
		rgb_global1.copyTo(rgb_tmp, mask_obj_global);
		imshow("rgb_o",rgb_tmp); cvWaitKey(1);
#endif

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
	hs[0] = 122;
	hs[1] = 102;
	hs[2] = 150;
	hs[3] = 69;

	cv::Mat img_no_head = cv::Mat::zeros(480,640,CV_8UC3);

	while(true)
	{
		sem_wait(&lock_t3);
		sem_wait(&mutex3);
/*
		face_global.y     -= face_global.height * 0.1;
		face_global.height = face_global.height * 1.3;
		rgb_global3(face_global) = 0;
*/
      	rgb_global3.clone().rowRange(160,480).copyTo(img_no_head.rowRange(160,480));
		segmentHSV(img_no_head, hs, mask_hand_global, box_hand_global);

#ifdef FLAG_HAND
		Mat rgb_tmp = Mat::zeros(480,640, CV_8UC3);
		rgb_global1.copyTo(rgb_tmp, mask_hand_global);
		imshow("rgb_h",rgb_tmp); cvWaitKey(1);
#endif

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

	Mat img_tmp;
	int c = 0;
	while(true)
	{
		sem_wait(&mutex4);
/*
		bool testing1 = rgb_global4.empty();
		bool testing2 = img_tmp.empty();
		//if(c%freq_rate==0)
		//	cout << testing1 << testing2 ;
		//c++;

		bool eq = countNonZero(rgb_global4!=img_tmp) == 0;
		if (!eq)
		{
			img_tmp.release();
			img_tmp = rgb_global4.clone();
			face_global = detectFaceAndEyes(rgb_global4, face_cascade);
		}

#ifdef FLAG_FACE
		imshow("face",rgb_global4); cvWaitKey(1);
#endif
*/
		sem_post(&mutex4);
	}
	return 0;
}

// ============================================================================
// THREAD 5 : CONTACT DETECTOR
// ============================================================================
void* contactDetector(void* arg)
{
	Mat img_depth_def, mask_obj_def, img_sub, cloud_mask,cloud_mask2;
	float contact_sub;
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
		cloud_global.copyTo(cloud_mask,mask_obj_global); //taking the obj only
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
				contact_sub = sum(img_sub)[0] / sum(mask_obj_def)[0];

				if(contact_sub > 0 && contact_sub < 250)
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
		}
		// face prevention
		if(box_obj_global.y < 161) 
		{
			flag_contact_init	= false;
			flag_contact_obj	= true;
			contact_obj 		= true;
		} 
		// ****************************************************[OBJECT CONTACT]

#ifdef FLAG_CONTACT
		c++;
		if(c%freq_rate==0)
		{
			printf("CONTACT : %d , CONTACTVAL : %f\n", contact_obj, contact_sub);
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
	string file_name = "../data/";
	file_name += buf;
	file_name += ".txt";
	while(true)
	{
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&mutex6);
		if(frame_number_global>100)
		{
			// write values into data.txt
			ofstream write_file(file_name.c_str(), std::ios::app);
			write_file 	<< frame_number_global << ","
						<< contact_obj << ","
						<< single_point_obj_global[0]  << ","
						<< single_point_obj_global[1]  << ","
						<< single_point_obj_global[2]  << ","            
						<< plane_global[0] << ","
						<< plane_global[1] << ","
						<< plane_global[2] << ","
						<< plane_global[3] 
						<< "\n";
		}
		sem_post(&mutex6);
		sem_post(&lock_t1);
	}
// ***************************************************************[RECORD DATA]

#else

// [PREDICTION]****************************************************************
	while(true)
	{
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&mutex6);
		// RESERVED FOR PREDICTION CODE
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
int main(int argc, char *argv[])
{
	VideoCapture kinect(CV_CAP_OPENNI2); 
	printf("Starting Kinect ...\n");

  // Run the visualization
#ifdef FLAG_DEPTH
  namedWindow("depth");
#endif
#ifdef FLAG_RGB
  namedWindow("rgb");
#endif
#ifdef FLAG_HAND
  namedWindow("rgb_h");
  moveWindow("rgb_h",0,0);
#endif
#ifdef FLAG_OBJECT
  namedWindow("rgb_o");
  moveWindow("rgb_o",0,490);  
#endif
#ifdef FLAG_PLANE
  namedWindow("plane");
  namedWindow("plane_reduced");
#endif
#ifdef FLAG_MARKER
  namedWindow("rgb_m");
#endif
#ifdef FLAG_FACE
  namedWindow("face");
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





















