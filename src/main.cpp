/*
 * main.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: papazov
 */


//#include "dataDeclaration.h"
#include "util2.h"

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

Vec4f plane_global;

// threads
int MAX = 4;
sem_t mutex1,mutex2,mutex3,mutex4,mutex5,mutex6,mutex7;
sem_t lock_t1,lock_t2,lock_t3,lock_t4,lock_t5,lock_t6;


// option flags
//#define FLAG_RGB
//#define FLAG_DEPTH
//#define FLAG_FACE
//#define FLAG_MARKER
//#define FLAG_PLANE
#define FLAG_OBJECT
#define FLAG_HAND
#define FLAG_THREAD
//#define FLAG_WRITE

//#define FREQ


//====================================================================================================================================
#define SAVEDDATA
//#define DBSCAN
#include "dataDeclaration.h"
#include "algo.h"
#include "util.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"
unsigned int window = 5;
vector<string> LABEL_MOV;
vector<string> LABEL_LOC_MOV;
vector<string> LABEL_LOC;

//====================================================================================================================================
// [THREAD 1 : KINECT]*********************************************************
void* kinectGrab(void* v_kinect)
{
	VideoCapture *kinect = reinterpret_cast<VideoCapture *>(v_kinect);

	struct timeval start_time, end_time;
	int c = 0;

	bool flag_plane = false;

	float ratio[2]; ratio[0] = 0.005; ratio[1] = 0.5;
	char keypress;
    
	Mat plane_tmp = Mat::zeros(480,640,CV_8UC1);
	Mat tmp_cloud,tmp_cloud2;

	while(true)
	{

#ifdef FREQ
if(c==0) gettimeofday(&start_time, NULL);
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
		tmp_cloud = cloud_global.clone();
		while(!flag_plane)
		{       
			plane_tmp = Mat::zeros(480,640,CV_8UC1);
			plane_global = RANSAC3DPlane(tmp_cloud, plane_tmp, 500, ratio, 0.005);
			imshow("plane",(plane_tmp==0));
			printf("SAVE NORMAL VECTOR OF PLANE : [Y/N] \n\n");
			keypress = waitKey(0); 
			if (keypress == 'y') 
			{
				flag_plane = true; 
				destroyWindow("plane");
			} 
			/*else if(keypress == 'a') TODO need to change the ransac algo to check for connected area
			{
				tmp_cloud2.release();
				tmp_cloud.copyTo(tmp_cloud2,(plane_tmp==0)/255);
				tmp_cloud.release();
				tmp_cloud2.copyTo(tmp_cloud);
			}*/ 
			else waitKey(1);
			waitKey(30);
		}
#endif

		sem_post(&mutex1);
		sem_post(&lock_t2);
		sem_post(&lock_t3);
		sem_post(&lock_t5);
		sem_post(&lock_t6);

#ifdef FREQ
c++;
if(c>=20)
{
	c = 0;
	gettimeofday(&end_time, NULL);
	cout << 20/ ((end_time.tv_sec - start_time.tv_sec) + 
			(end_time.tv_usec- start_time.tv_usec) * 1e-6) 
		 << " [Hz]"<< endl;
}
#endif

	}
	return 0;
}


//====================================================================================================================================
// [THREAD 2 : OBJECT DETECTOR]************************************************
void* objectDetector(void* arg)
{
//	int hue_range_obj[2], sat_range_obj[2];
//	// yellow plyers
//	hue_range_obj[0] = 80; hue_range_obj[1] = 102;
//	sat_range_obj[0] = 135; sat_range_obj[1] = 255;
//	// green cup
//	hue_range_obj[0] = 77; hue_range_obj[1] = 98;
//	sat_range_obj[0] = 76; sat_range_obj[1] = 214;

	int hs[4];
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


//====================================================================================================================================
// [THREAD 3 : HAND DETECTOR]**************************************************
void* handDetector(void* arg)
{ 
//	// Crop Threshold
//	int hue_range_hand[2], sat_range_hand[2];
//	hue_range_hand[0] = 102; hue_range_hand[1] = 122;
//	sat_range_hand[0] = 69 ; sat_range_hand[1] = 150;

	int hs[4];
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

//====================================================================================================================================
// [THREAD 4 : FACE DETECTOR]**************************************************

void* faceDetector(void* arg)
{
	//Load the cascade for face detector
	string face_cascade_name = "../cascade/lbpcascade_frontalface.xml";
	CascadeClassifier face_cascade;
	if(!face_cascade.load(face_cascade_name))
	printf("--(!)Error loading face cascade\n");

	Mat img_tmp;

	while(true)
	{
		sem_wait(&mutex4);
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

		sem_post(&mutex4);
	}
	return 0;
}

//====================================================================================================================================
// [THREAD 5 : CONTACT DETECTOR]***********************************************
void* contactDetector(void* arg)
{

	Mat img_depth_def, mask_obj_def, img_sub;

	Mat cloud_mask,cloud_mask2;

	float contact_sub;

	bool flag = true;
	bool flag_contact_obj = false;

	int c = 0;

	while(true)
	{
		sem_wait(&lock_t5);
		sem_wait(&mutex5);

		//[DEFAULT SCENE]********************************************************
		if(flag)
		{
			mask_obj_def = mask_obj_global.clone();
			depth_global.copyTo(img_depth_def,mask_obj_def);
			if(box_obj_global.x > 0 && box_obj_global.y > 0) flag = false;
		}
		//********************************************************[DEFAULT SCENE]

		//[OBJECT POINT]*********************************************************
		cloud_global.copyTo(cloud_mask,mask_obj_global); //taking the obj only
		cloud_mask(box_obj_global).copyTo(cloud_mask2); // reducing the search area
		pointCloudTrajectory(cloud_mask2, single_point_obj_global);
		cloud_mask.release(); 
		cloud_mask2.release();
		//*********************************************************[OBJECT POINT]

		//[OBJECT CONTACT]*******************************************************
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
					contact_obj = true;
					flag_contact_obj = true;
				}
				else contact_obj = false;
			}
			else contact_obj = true;     
		}
		else
		{
			flag = true;
			flag_contact_obj = false;
			contact_obj = false;
		}

		if(box_obj_global.y < 161) 
		{
			flag = false;
			flag_contact_obj = true;
			contact_obj = true;
		} // face prevention
		//*******************************************************[OBJECT CONTACT]

		c++;
		if(c>=30)
		{
			printf("CONTACT : %d     CONTACTVAL : %f\n", contact_obj, contact_sub);
			c = 0;
		}

		sem_post(&mutex5);
		sem_post(&lock_t6);
		sem_post(&lock_t1);
	}
	return 0;
}

//====================================================================================================================================
// [THREAD 6 : WRITE DATA]*****************************************************
void* writeData(void* arg)
{
	//[VARIABLES]**************************************************************
	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	int num_surfaces 	= 1;
	int num_scene 		= 1;
	int num_object 		= 1;

	int num_points 		= 0;
	int num_locations	= 0;
	int file_num 		= 0;

	int minpts 			= 10;
	double epsilon 		= 0.015; 	// if datasets are merged these 2 values can be increased.

	double vel_limit 	= 0.005; //##### still need to validate

	string scene  = "Kitchen";
	string object = "Cup";

	//	vector<vector<Graph> > Graph; // scene -> object
	//	reshapeVector(Graph, num_scene);
	//	for(int i=0;i<num_scene;i++) reshapeVector(Graph[i], num_object);

	Graph Graph_(scene, object);
	//**************************************************************[VARIABLES]

	//[PREDICTION]*************************************************************
	readSurfaceFile(Graph_);
	readLocation_(Graph_);
	readMovement(Graph_);
	readSectorFile(Graph_, 0);
	readSectorFile(Graph_, 1);
	readSectorFile(Graph_, 2);

	num_locations = Graph_.getNodeList().size();

	bool flag_motion      	= false;
	bool flag_predict      	= false;
	bool flag_predict_last 	= false;
	bool learn 				= false;
	bool slide 				= false;
	int last_location 		= 0;
	int surface_num_tmp 	= 0;
	double pow_dec 			= 1;

	vector<int> prediction;
	vector<double> t_val;
	vector<data_t> tmp_data;
	reshapeVector(prediction, num_locations);
	reshapeVector(t_val,      num_locations);

	vector<double> predict_in;
	vector<double> predict_err;
	vector<double> predict_in_last;
	reshapeVector(predict_in,      num_locations);
	reshapeVector(predict_err,     num_locations);
	reshapeVector(predict_in_last, num_locations);

	prepareSector(Graph_);
	printf("Preparing sectors......Complete\n");

	Graph Graph_mem = Graph_;

	point_t curr_point;
	vector<point_t>	pos_vel_acc_avg(3);	
	vector<vector< point_t > > 	pos_vel_acc_mem; // motion->length
	reshapeVector(pos_vel_acc_mem, 3);

	int i = -1;
	while(true)
	{
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&mutex6);

		i++;

		//[PREPROCESS DATA]****************************************************
		curr_point.x = single_point_obj_global[0];
		curr_point.y = single_point_obj_global[1];
		curr_point.z = single_point_obj_global[2];
		curr_point.cluster_id = UNCLASSIFIED;

		preprocessDataLive(curr_point, pos_vel_acc_mem,
						   pos_vel_acc_avg, window);
		//****************************************************[PREPROCESS DATA]


// ============================================================================
// PREDICTION STARTS
// ============================================================================

// 1. Contact trigger
// 1.1 Check if the object is within a sphere volume of the location areas
triggerContact(pos_vel_acc_avg[0], Graph_);

slide = false;

// 2. Prediction during motion
if (pos_vel_acc_avg[0].cluster_id < 0)
{
	if(VERBOSE == 0 || VERBOSE == 2) printf("Nr:%04d,  ", i);

	flag_motion = true;

	// 2.1. Set flag to allow online learning/updates of the knowledge base
	// learn = true;

	// 2.2. Check if the trajectory is within the range of sector map
	checkSector(prediction, t_val,
				pos_vel_acc_avg[0], Graph_, Graph_mem,
				last_location, learn);

	// 2.3. Check for motion (moving/null)
	checkMotion(pos_vel_acc_avg[0], pos_vel_acc_avg[1],
				Graph_.getMovLabel(), Graph_.getSurface(),
				0.1, 0.97, vel_limit);

	// 2.4. Prediction based on the trajectory error from sector map
	motionPrediction(prediction, t_val,
					 flag_predict, flag_predict_last,
					 predict_in, predict_err, predict_in_last,
					 pow_dec, Graph_);

	cout << endl;
}
// 3. Prediction within location area
else
{
	if(VERBOSE == 1 || VERBOSE == 2) printf("Nr:%04d,  ", i);

	flag_predict      = false;
	flag_predict_last = false;

	// 3.1. Location area prediction based on contact trigger
	locationPrediction(pos_vel_acc_avg[0].cluster_id,
					   pos_vel_acc_avg[0], pos_vel_acc_avg[1],
					   Graph_, 0.1, 0.97, vel_limit);

	// 3.2. Check if it is moved back to the same location or not
	if (last_location == pos_vel_acc_avg[0].cluster_id && flag_motion)
	{
		cout << " (same last location...)";
		// update the sector using values from memory
		for(int ii=0;ii<num_locations;ii++)
		{
			int cc = last_location * num_locations + ii;
			int tmp = Graph_.getEdgeList()[cc].size();
			for(int a=0;a<tmp;a++)
			{
				for(int b=0;b<Graph_.getSectorPara().loc_int*Graph_.getSectorPara().sec_int;b++)
				{
					Graph_.getEdgeList()[cc][a].sector_map[b].max =
							Graph_mem.getEdgeList()[cc][a].sector_map[b].max;
					Graph_.getEdgeList()[cc][a].sector_map[b].min =
							Graph_mem.getEdgeList()[cc][a].sector_map[b].min;
				}
			}
		}
	}

	// 3.3. Update sector map if learn flag is set
	if (last_location != pos_vel_acc_avg[0].cluster_id && flag_motion)
	{
		if (learn)
		{
			// copy only the intended values for sectors
			// updating the values in memory
			int c = last_location * num_locations + pos_vel_acc_avg[0].cluster_id;
			int tmp = Graph_.getEdgeList()[c].size();
			for(int iii=0;iii<tmp;iii++)
			{
				Graph_mem.updateEdgeSector(
						Graph_.getEdgeList()[c][iii].sector_map,
						last_location, pos_vel_acc_avg[0].cluster_id, iii);
			}
			// update the sector using values from memory
			// last location is used because only sector with last location was changed
			for(int ii=0;ii<num_locations;ii++)
			{
				int cc = last_location * num_locations + ii;
				int tmp = Graph_.getEdgeList()[cc].size();
				for(int iii=0;iii<tmp;iii++)
				{
					Graph_.updateEdgeSector(
							Graph_mem.getEdgeList()[cc][iii].sector_map,
							last_location, ii, iii);
				}
			}
		}
		else
		{
			// update the sector using values from memory
			for(int ii=0;ii<num_locations;ii++)
			{
				int cc  = last_location * num_locations + ii;
				int tmp = Graph_.getEdgeList()[cc].size();
				for(int iii=0;iii<tmp;iii++)
				{
					Graph_.updateEdgeSector(
							Graph_mem.getEdgeList()[cc][iii].sector_map,
							last_location, ii, iii);
				}
			}
		}
	}

	last_location = pos_vel_acc_avg[0].cluster_id;
	flag_motion = false;

	if(VERBOSE == 1 || VERBOSE == 2) cout << endl;

}

// ============================================================================
// PREDICTION ENDS
// ============================================================================




		sem_post(&mutex6);
		sem_post(&lock_t1);
	}



























/*
	//remove("../data/traj_data.txt");
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

#ifdef FLAG_WRITE
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
#endif

    sem_post(&mutex6);
    sem_post(&lock_t1);
  }
*/
  return 0;
}


//====================================================================================================================================


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
#endif
#ifdef FLAG_MARKER
  namedWindow("rgb_m");
#endif
#ifdef FLAG_FACE
  namedWindow("face");
#endif

	pthread_t thread_kinectGrab,
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





















