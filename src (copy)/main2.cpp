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

	while(true)
	{
		sem_wait(&lock_t3);
		sem_wait(&mutex3);

		face_global.y     -= face_global.height * 0.1;
		face_global.height = face_global.height * 1.3;
		rgb_global3(face_global) = 0;

		segmentHSV(rgb_global3, hs, mask_hand_global, box_hand_global);

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
			face_global = detectFaceAndEyes(rgb_global4, face_cascade);
			img_tmp.release();
			img_tmp = rgb_global4.clone();
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

				if(contact_sub > 0 && contact_sub < 150)
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

		//if(box_obj_global.y < 241) {contact_obj = true;} // face prevention
		//*******************************************************[OBJECT CONTACT]

		c++;
		if(c>=200)
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

	int num_points 		= 0;
	int num_locations	= 0;
	int file_num 		= 0;

	int num_surfaces 	= 1;
	int num_mov 		= 2;
	int obj 			= 3;
	int num_objs 		= 4;

	double epsilon 		= 0.015; 	// if datasets are merged these 2 values can be increased.
	int minpts 			= 10;

	double vel_limit 	= 0.01; //##### still need to validate

	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;

	DIR *dir;
	struct dirent *dent;
	char *name;

	vector<int> file_eof;

	vector<vector<string> > data_full;
	vector<vector<string> > data_test;

	point_t *points_array;
	vector<point_t> points;
	vector<point_t> points_test;
	vector<point_t> locations;

	vector<double> location_boundary;

	vector<double> 			surface_const(4);
	vector<vector<double> > surface(num_surfaces);
	for(int i=0;i<num_surfaces;i++) surface[i] = surface_const;

	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<vector<point_t> > 	pos_vel_acc_avg;

	Graph Graph[2];

	vector<vector<data_t> > node_data;
	vector<vector<double> > node_data_tmp;
	//**************************************************************[VARIABLES]

	//[READ FILE]**************************************************************
	string dir_name = "../data/";
	dir = opendir(dir_name.c_str());
	if(dir!=NULL)
	{
		while((dent=readdir(dir))!=NULL)
		{
			if (strcmp(dent->d_name,".")==0 ||
				strcmp(dent->d_name,"..")==0 ||
				(*dent->d_name) == '.' )
				continue;

			string fn = string(dent->d_name);
			size_t found_extension = fn.find(".txt");
			if(found_extension==std::string::npos)
				continue;

			name = new char[256];
			sprintf(name, "%s%s",dir_name.c_str(),dent->d_name);
			readFile(name, data_full , ',');
			file_eof.push_back(data_full.size());
			delete name;

			//table
			vector<string> last_line = data_full[data_full.size()-1];
			surface[0][0] = atof(last_line[5].c_str());
			surface[0][1] = atof(last_line[6].c_str());
			surface[0][2] = atof(last_line[7].c_str());
			surface[0][3] = atof(last_line[8].c_str());
		}
	}
	closedir(dir);
	//**************************************************************[READ FILE]
	printf("Reading training data......Complete\n");


	//[PARSE DATA]*************************************************************
	num_points = data_full.size();
	points.clear();
	points.resize(num_points);
	parseData2Point(data_full, points);
	//*************************************************************[PARSE DATA]
	printf("Parsing training data......Complete\n");

	// LEARNING
	//[PREPROCESS DATA]********************************************************
	vector<vector<double> > 		  	pos_vel_acc_mem1(3);
	vector<vector< vector<double> > > 	pos_vel_acc_mem;
	for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);

	pos_vel_acc_avg.clear();
	pos_vel_acc_avg.resize(num_points);
	for(int i=0;i<num_points;i++)
	{
		if(i == file_eof[file_num])
		{
			file_num++;
			pos_vel_acc_mem.clear();
			for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
		}
		if(i==0)
			pos_vel_acc_avg[i] = pos_vel_acc_avg1;
		else
			pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1];
		preprocessDataLive(points[i], pos_vel_acc_mem,
						   pos_vel_acc_avg[i], window);
	}
	//********************************************************[PREPROCESS DATA]
	printf("Preprocessing data......Complete\n");

	//[LABELLING MOVEMENT]*****************************************************
	labelingMovement(LABEL_MOV, num_mov, obj, num_objs);
	//*****************************************************[LABELLING MOVEMENT]

	//[LABELLING LOCATION]*****************************************************
	labelingLocation(points, locations, location_boundary, LABEL_LOC,
					 obj, epsilon, minpts);
	num_locations = locations.size();
	//*****************************************************[LABELLING LOCATION]
	printf("Labeling of clusters (action locations)......Complete\n");



//[GRAPH]**********************************************************************
	//[ADD NODES]**************************************************************
	// Collecting data based on locations/nodes
	node_data.clear();
	node_data.resize(num_locations);
	data_t motion_data;
	for(int i=0;i<num_points;i++)
		if (points[i].cluster_id >= 0)
		{
			motion_data.pos = pos_vel_acc_avg[i][0];
			motion_data.vel = pos_vel_acc_avg[i][1];
			motion_data.acc = pos_vel_acc_avg[i][2];
			node_data[points[i].cluster_id].push_back(motion_data);
		}

	// Adding nodes to the graph : nodes corresponds to the locations
	for(int i=0;i<num_locations;i++)
	{
		vector<double> tmp_vec(3);
		int surface_num  =    -1;
		double dist_tmp  = 	 0.0;
		double dist_tmp2 =  10.0; //### need improvement
		double tmp_spd 	 =   0.0;
		double tmp_dir 	 =   0.0;
		for(int ii=0;ii<node_data[i].size();ii++)
		{
			for(int iii=0;iii<num_surfaces;iii++)
			{
				dist_tmp = surfaceDistance(node_data[i][ii].pos,surface[iii]);
				if (dist_tmp < 0.1 && !max_(dist_tmp,dist_tmp2))
					surface_num = iii;
				dist_tmp2 = dist_tmp;

				tmp_vec[0] = surface[iii][0];
				tmp_vec[1] = surface[iii][1];
				tmp_vec[2] = surface[iii][2];
			}
			tmp_spd += l2Norm(node_data[i][ii].vel);
			tmp_dir += l2Norm(crossProduct(tmp_vec,point2vector(node_data[i][ii].vel)))/
					   (l2Norm(tmp_vec) * l2Norm(point2vector(node_data[i][ii].vel)));
		}

		if (surface_num >= 0 &&
			tmp_spd/node_data[i].size() > vel_limit &&
			tmp_dir/node_data[i].size() > 0.96) // sind(75)
			Graph[0].addNode(LABEL_LOC[i],1,surface_num,location_boundary[i],node_data[i]);
		else
			Graph[0].addNode(LABEL_LOC[i],0,surface_num,location_boundary[i],node_data[i]);
	}
	//**************************************************************[ADD NODES]
	//[VISUALIZE NODES]********************************************************
	bool pos_flag = true;
	bool vel_flag = false;
	bool acc_flag = false;

	node_data_tmp = Graph[0].getNodeDataLabel(pos_flag,vel_flag,acc_flag);

	vector<point_t> pos_node(node_data_tmp.size());
	vector<point_t> vel_node(node_data_tmp.size());
	vector<point_t> acc_node(node_data_tmp.size());
	for(int i=0; i<node_data_tmp.size();i++)
	{
		vector<double> tmp_data = node_data_tmp[i];
		int c = 0;
		if(pos_flag)
		{
			pos_node[i].x = tmp_data[c]; ++c;
			pos_node[i].y = tmp_data[c]; ++c;
			pos_node[i].z = tmp_data[c]; ++c;
			pos_node[i].cluster_id = tmp_data[c]; ++c;
		}
		if(vel_flag)
		{
			vel_node[i].x = tmp_data[c]; ++c;
			vel_node[i].y = tmp_data[c]; ++c;
			vel_node[i].z = tmp_data[c]; ++c;
			vel_node[i].cluster_id = tmp_data[c]; ++c;
		}
		if(acc_flag)
		{
			acc_node[i].x = tmp_data[c]; ++c;
			acc_node[i].y = tmp_data[c]; ++c;
			acc_node[i].z = tmp_data[c]; ++c;
			acc_node[i].cluster_id = tmp_data[c]; ++c;
		}
	}
	//********************************************************[VISUALIZE NODES]
	printf("Creating nodes for clusters (action locations)......Complete\n");

	//[EDGES]******************************************************************
	// Initialize sector

	vector<vector<vector<sector_t> > > 	sector (Sqr(num_locations));
           vector<vector<sector_t> > 	sector1(num_location_intervals);
	              vector<sector_t> 		sector2(num_sector_intervals);

	vector<vector<vector<double> > > sector_constraint (Sqr(num_locations));
	       vector<vector<double> > 	 sector_constraint1(num_location_intervals);
	              vector<double>  	 sector_constraint2(num_sector_intervals);

	for(int i=0;i<Sqr(num_locations);i++)
	{
		sector[i] 				= sector1;
		sector_constraint[i] 	= sector_constraint1;
		for(int ii=0;ii<num_location_intervals;ii++)
		{
			sector[i][ii] 				= sector2;
			sector_constraint[i][ii] 	= sector_constraint2;
			for(int iii=0;iii<num_sector_intervals;iii++)
			{
				sector[i][ii][iii].max = 0;
				sector[i][ii][iii].min = INFINITY;
			}
		}
	}

	vector<point_t> norm_location_dir   (Sqr(num_locations));
	vector<point_t> norm_location_normal(Sqr(num_locations));
	vector<double>  distance_location   (Sqr(num_locations));

	for(int i=0;i<Sqr(num_locations);i++)
	{
		norm_location_dir   [i].cluster_id = UNCLASSIFIED;
		norm_location_normal[i].cluster_id = UNCLASSIFIED;
	}

	// prepare the vectors from locations
	prepareSector(norm_location_dir, norm_location_normal,
				  distance_location, locations);

	// THE use of gaussian kernel helps to smoothen can create a tube like structure.
	// However it is still possible to have like bumps because the sampling is just not enough.
	int kernel_size = 5;
	vector<vector<double> > kernel(kernel_size);
	for(int i=0;i<kernel_size;i++) kernel[i].resize(kernel_size);
	gaussKernel(kernel, kernel_size, kernel_size, 1.0);

	generateSector(sector, points, locations,
				   norm_location_dir, norm_location_normal, distance_location,
			       num_location_intervals, num_sector_intervals,
			       file_eof, kernel);

	checkSectorConstraint(sector, sector_constraint, num_locations,
						  num_location_intervals, num_sector_intervals);

//	showConnection(sector, sector_constraint,
//				   norm_location_dir, norm_location_normal, distance_location,
//				   locations,
//				   num_location_intervals, num_sector_intervals, color_code);
	//******************************************************************[EDGES]
	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");

//**********************************************************************[GRAPH]
printf("Creating a graph to represent the clusters (action locations)......Complete\n");










	//[PREPROCESS DATA]********************************************************
	pos_vel_acc_mem.clear();
	for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
	pos_vel_acc_avg.clear();
	pos_vel_acc_avg.resize(1);
	pos_vel_acc_avg[0] = pos_vel_acc_avg1;
	//********************************************************[PREPROCESS DATA]

	//[GRAPH TEMPORARY]********************************************************
	//[ADD NODES]**************************************************************
	// Initialize sector backup
	vector<vector<vector<sector_t> > > 	sector_mem(Sqr(num_locations));

	for(int i=0;i<Sqr(num_locations);i++)
	{
		sector_mem[i] = sector1;
		for(int ii=0;ii<num_location_intervals;ii++)
		{
			sector_mem[i][ii] = sector2;
			for(int iii=0;iii<num_sector_intervals;iii++)
			{
				sector_mem[i][ii][iii].max = sector[i][ii][iii].max;
				sector_mem[i][ii][iii].min = sector[i][ii][iii].min;
			}
		}
	}

	bool flag_predict = false;
	double prediction1[num_locations];
	double prediction2[num_locations];
	vector<double> prediction1_(num_locations);

	double prediction_curr[num_locations];
	int last_prediction_flag = -1;
	double cc = 1;

	vector<double> prediction(num_locations);
	vector<double> t_val(num_locations);
	vector<data_t> tmp_data;
	bool learn = false;
	bool slide = false;
	int last_location = 0;
	int surface_num_tmp = 0;

	point_t curr_point;
	int counter = 0;

	while(true)
	{
		sem_wait(&lock_t6);
		sem_wait(&lock_t6);
		sem_wait(&mutex6);

		//[PREPROCESS DATA]********************************************************
		curr_point.x = single_point_obj_global[0];
		curr_point.y = single_point_obj_global[1];
		curr_point.z = single_point_obj_global[2];
		curr_point.cluster_id = UNCLASSIFIED;

		preprocessDataLive(curr_point, pos_vel_acc_mem,
						   pos_vel_acc_avg[0], window);
		//********************************************************[PREPROCESS DATA]

counter++;
if(counter == 20)
{
		counter = 0;

		slide = false;

		decideBoundary(pos_vel_acc_avg[0][0], locations, location_boundary);

		cout << "000" << " " << pos_vel_acc_avg[0][0].cluster_id << " ";

		if (pos_vel_acc_avg[0][0].cluster_id < 0)
		{
//			learn = true;

			checkSector(prediction, t_val, sector,
						pos_vel_acc_avg[0][0],
						locations,
						norm_location_dir, norm_location_normal,
						distance_location,
						num_location_intervals, num_sector_intervals,
						last_location, learn);

			if (l2Norm(pos_vel_acc_avg[0][1])> vel_limit)
			{
				for(int ii=0;ii<num_surfaces;ii++)
				{
					if (!slide)
					{
						slide =	checkMoveSlide(pos_vel_acc_avg[0][0], pos_vel_acc_avg[0][1], surface[ii], 0.1, 0.97); //####need to add
						surface_num_tmp = ii;
					}
				}

				if (slide)
					cout << " LABEL: "<< LABEL_MOV[1] << " surface " << surface_num_tmp;
				else
					cout << " LABEL: "<< LABEL_MOV[0];
			}
			else
				cout << " LABEL: "<< "NULL";



			for(int ii=0;ii<num_locations;ii++)
			{
				if ((int)prediction[ii]==1)
				{
					prediction1[ii]  = 1.0;
					prediction2[ii]  = 0.0;
				}
				else if ((int)prediction[ii]>1)
				{
					prediction1[ii]  = 0.0;
					prediction2[ii]  = t_val[ii];
				}
				else
				{
					prediction1[ii]  = 0.0;
					prediction2[ii]  = 0.0;
				}
			}

			double tmp_t = 0.0;
			for(int ii=0;ii<num_locations;ii++)
				tmp_t += prediction1[ii];
			for(int ii=0;ii<num_locations;ii++)
				if (tmp_t>0)
					prediction1[ii] /= tmp_t;

			if (tmp_t>0) flag_predict = false;

			double tmp_t2 = 0.0;
			for(int ii=0;ii<num_locations;ii++)
				tmp_t2 += prediction2[ii];
			for(int ii=0;ii<num_locations;ii++)
				if (prediction2[ii] != 0)
					if (prediction2[ii] != tmp_t2)
						prediction2[ii] = 1.0 - (prediction2[ii]/tmp_t2);
					else
						prediction2[ii] = 1.0;

			double tmp_t3 = 0.0;
			for(int ii=0;ii<num_locations;ii++)
				tmp_t3 += prediction2[ii];
			for(int ii=0;ii<num_locations;ii++)
				if (tmp_t3>0)
					prediction2[ii] /= tmp_t3;

			if (flag_predict)
			{
				for(int ii=0;ii<num_locations;ii++)
				{
					if (prediction1_[ii]>0 && prediction[ii]>0)
						prediction2[ii] = prediction2[ii] * (1-pow(0.5,cc)) + pow(0.5,cc);
					else if (prediction2[ii]!=1.0)
						prediction2[ii] = prediction2[ii] * (1-pow(0.5,cc));
				}
				cc += 0.01;
			}
			else
			{
				for(int ii=0;ii<num_locations;ii++)
					prediction1_[ii] = prediction1[ii];
				cc = 1;
			}

			if (tmp_t>0) flag_predict = true;

			for(int ii=0;ii<num_locations;ii++)
				//printf(" %d %.4f ", (int)prediction[ii], prediction2[ii]);
				printf(" %.4f ", prediction2[ii]);
			for(int ii=0;ii<num_locations;ii++)
				if((int)prediction[ii]==1)
					printf(" %s %.4f ", LABEL_LOC[ii+1].c_str(), prediction1[ii]);

		}
		else
		{
			flag_predict = false;

			if (LABEL_LOC[pos_vel_acc_avg[0][0].cluster_id+1].empty())
			{
				cout << " LABEL: "<< "Empty location Label.";
				if (l2Norm(pos_vel_acc_avg[0][1])> vel_limit)
				{
					for(int ii=0;ii<num_surfaces;ii++)
					{
						if (!slide)
						{
							slide =	checkMoveSlide(pos_vel_acc_avg[0][0], pos_vel_acc_avg[0][1], surface[ii], 0.1, 0.96); //####needto add
							surface_num_tmp = ii;
						}
					}

					if (slide)
						cout << " LABEL: "<< LABEL_MOV[1] << " surface " << surface_num_tmp;
					else
						cout << " LABEL: "<< LABEL_MOV[0];
				}
				else
					cout << " LABEL: "<< "NULL";
			}
			else
				cout << " LABEL: "<< LABEL_LOC[pos_vel_acc_avg[0][0].cluster_id+1];

			// if it is moved back to the same location
			if (last_location != pos_vel_acc_avg[0][0].cluster_id)
			{
				if (learn)
				{
					// copy only the intended values for sectors
					// updating the values in memory
					int c = last_location * num_locations + pos_vel_acc_avg[0][0].cluster_id;
					for(int ii=0;ii<num_location_intervals;ii++)
					{
						for(int iii=0;iii<num_sector_intervals;iii++)
						{
							sector_mem[c][ii][iii].max = sector[c][ii][iii].max;
							sector_mem[c][ii][iii].min = sector[c][ii][iii].min;
						}
					}
					// update the sector using values from memory
					for(int ii=0;ii<num_locations;ii++)
					{
						int cc = last_location * num_locations + ii;
						for(int iii=0;iii<num_location_intervals;iii++)
						{
							for(int iiii=0;iiii<num_sector_intervals;iiii++)
							{
								sector[cc][iii][iiii].max = sector_mem[cc][iii][iiii].max;
								sector[cc][iii][iiii].min = sector_mem[cc][iii][iiii].min;
							}
						}
					}
					learn = false;
				}
				else
				{
					// update the sector using values from memory
					for(int ii=0;ii<num_locations;ii++)
					{
						int cc = last_location * num_locations + ii;
						for(int iii=0;iii<num_location_intervals;iii++)
						{
							for(int iiii=0;iiii<num_sector_intervals;iiii++)
							{
								sector[cc][iii][iiii].max = sector_mem[cc][iii][iiii].max;
								sector[cc][iii][iiii].min = sector_mem[cc][iii][iiii].min;
							}
						}
					}
				}
			}
			else
			{
				if (learn)
					cout << " (same last location...)";
			}

			last_prediction_flag = -1;
			last_location = pos_vel_acc_avg[0][0].cluster_id;
		}
		cout << endl;
	//********************************************************[GRAPH TEMPORARY]

}
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





















