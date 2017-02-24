//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

//#define LEARN

#include "dataDeclaration.h"
#include "algo.h"
#include "util.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"


//=============================================================================
// Global
//=============================================================================
unsigned int window = 5;

vector<string> LABEL_MOV;
vector<string> LABEL_LOC;


//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{

#ifdef LEARN

	// [VARIABLES]*************************************************************
	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);

	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;

	int num_surfaces 	= 1;
	int num_scene 		= 1;
	int num_object 		= 1;

	int num_points 		= 0;
	int num_locations	= 0;
	int num_mov 		= 2;
	int file_num 		= 0;

	int minpts 			= 10;
	double epsilon 		= 0.015; 	// if datasets are merged these 2 values can be increased.

	string scene  = "Kitchen";
	string object = "Cup";

	vector<int> 				file_eof;
	vector<point_t> 			points;
	vector<point_t> 			locations;
	vector<vector<string> > 	data_full;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion

	vector<double> 				surface_const(4);
	vector<vector<double> > 	surface(num_surfaces);
	for(int i=0;i<num_surfaces;i++) surface[i] = surface_const;

//	vector<vector<Graph> > Graph; // scene -> object
//	reshapeVector(Graph, num_scene);
//	for(int i=0;i<num_scene;i++) reshapeVector(Graph[i], num_object);

	Graph Graph(scene, object);

	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	DIR *dir;
	struct dirent *dent;
	string name;
	string dir_name;

	dir_name = "../../KINECT/data/";
	dir		 = opendir(dir_name.c_str());

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

			name = dir_name + dent->d_name;
			readFile(name.c_str(), data_full , ',');
			file_eof.push_back(data_full.size());

			//table
			vector<string> last_line = data_full[data_full.size()-1];
			surface[0][0] = atof(last_line[5].c_str());
			surface[0][1] = atof(last_line[6].c_str());
			surface[0][2] = atof(last_line[7].c_str());
			surface[0][3] = atof(last_line[8].c_str());
		}
	}
	closedir(dir);
	Graph.addSurface(surface);
	writeSurfaceFile(Graph);
	printf("Reading training data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_full.size();
	reshapeVector(points, num_points);
	parseData2Point(data_full, points);
	printf("Parsing training data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [PREPROCESS DATA]*******************************************************
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<point_t > 		  	pos_vel_acc_mem1(3);
	vector<vector< point_t > > 	pos_vel_acc_mem; // motion->length
	reshapeVector(pos_vel_acc_avg, num_points);
	reshapeVector(pos_vel_acc_mem, 3);

	for(int i=0;i<num_points;i++)
	{
		if (i == file_eof[file_num])
		{
			file_num++;
			reshapeVector(pos_vel_acc_mem, 3);
		}
		if (i == 0)
			pos_vel_acc_avg[i] = pos_vel_acc_avg1;
		else
			pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1];
		preprocessDataLive(points[i], pos_vel_acc_mem, pos_vel_acc_avg[i],
						   window);
	}
	printf("Preprocessing data......Complete\n");
	//********************************************************[PREPROCESS DATA]

	//[LABELLING MOVEMENT]*****************************************************
	labelMovement(Graph);
	printf("Labeling of movements......Complete\n");
	//*****************************************************[LABELLING MOVEMENT]


// [GRAPH]*********************************************************************

	// [ADD NODES]*************************************************************
	labelLocation_(Graph, pos_vel_acc_avg, epsilon, minpts);
	printf("Creating nodes for clusters (action locations)......Complete\n");
	// *************************************************************[ADD NODES]

	//[EDGES]******************************************************************
	Graph.initEdge(num_location_intervals, num_sector_intervals);
	labelSector(Graph, pos_vel_acc_avg,	0.05, 5, 5, file_eof, color_code);
	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");
	//******************************************************************[EDGES]

//**********************************************************************[GRAPH]
printf("Creating a graph to represent the clusters (action locations)......Complete\n");



#else

// [TESTING]*******************************************************************



	// [VARIABLES]*************************************************************
	int num_scene 		= 1;
	int num_object 		= 1;

	string scene  = "Kitchen";
	string object = "Cup";

	//	vector<vector<Graph> > Graph; // scene -> object
	//	reshapeVector(Graph, num_scene);
	//	for(int i=0;i<num_scene;i++) reshapeVector(Graph[i], num_object);

	Graph Graph_(scene, object);

	int num_points 		= 0;
	int num_locations	= 0;
	int num_surface		= 0;
	int file_num 		= 0;

	int minpts 			= 10;
	double epsilon 		= 0.015; 	// if datasets are merged these 2 values can be increased.
	double vel_limit 	= 0.005; //##### still need to validate

	vector<int> 				file_eof;
	vector<point_t> 			points_test;
	vector<vector<string> > 	data_test;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion

	vector<unsigned char*> color_code(12);
	for(int j=0;j<12;j++) color_code[j] = Calloc(unsigned char,3);
	colorCode(color_code);
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	DIR *dir;
	struct dirent *dent;
	string name;
	string dir_name;

	dir_name = "../../KINECT/data/test/";
	dir		 = opendir(dir_name.c_str());

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

			name = dir_name + dent->d_name;
			readFile(name.c_str(), data_test , ',');
			file_eof.push_back(data_test.size());

//			//table
//			vector<string> last_line = data_full[data_full.size()-1];
//			surface[0][0] = atof(last_line[5].c_str());
//			surface[0][1] = atof(last_line[6].c_str());
//			surface[0][2] = atof(last_line[7].c_str());
//			surface[0][3] = atof(last_line[8].c_str());
		}
	}
	closedir(dir);
	readSurfaceFile(Graph_);
	printf("Reading test data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_test.size();
	reshapeVector(points_test, num_points);
	parseData2Point(data_test, points_test);
	printf("Parsing test data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [LEARNED DATA]**********************************************************
	readLocation_ (Graph_   );
	readMovement  (Graph_   );
	readSectorFile(Graph_, 0);
	readSectorFile(Graph_, 1);
	readSectorFile(Graph_, 2);
	num_locations = Graph_.getNodeList().size();
	num_surface	= Graph_.getSurface().size();
	// **********************************************************[LEARNED DATA]

	// [PREDICTION VARIABLES]**************************************************
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
	vector<double> predict_in;
	vector<double> predict_err;
	vector<double> predict_in_last;
	reshapeVector(prediction, 		num_locations);
	reshapeVector(t_val,      		num_locations);
	reshapeVector(predict_in,		num_locations);
	reshapeVector(predict_err,		num_locations);
	reshapeVector(predict_in_last,	num_locations);

	prepareSector(Graph_);
	printf("Preparing sectors......Complete\n");

	Graph Graph_mem = Graph_;

	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<point_t > 		  	pos_vel_acc_mem1(3);
	vector<vector< point_t > > 	pos_vel_acc_mem; // motion->length
	reshapeVector(pos_vel_acc_avg, num_points);
	reshapeVector(pos_vel_acc_mem, 3);

	label_t LABEL;
	LABEL.mov = -1;
	reshapeVector(LABEL.loc, 		num_locations);
	reshapeVector(LABEL.surface, 	num_surface);

	// JUST FOR VISUALIZING
	vector<point_t> p_data; vector<string> label_data;
	for(int i=0;i<num_locations;i++)
		label_data.push_back(Graph_.getNode(i).name);
	// **************************************************[PREDICTION VARIABLES]

	printf("\n\n>>>>> SYSTEM START <<<<<\n\n");

	for(int i=0;i<num_points;i++)
	{
		//[PREPROCESS DATA]****************************************************
		if(i == file_eof[file_num])
		{
			file_num++;
			pos_vel_acc_mem.clear();
			for(int i=0;i<3;i++) pos_vel_acc_mem.push_back(pos_vel_acc_mem1);
		}

		if (i>0) 	pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1];
		else 		pos_vel_acc_avg[i] = pos_vel_acc_avg1;

		preprocessDataLive(points_test[i], pos_vel_acc_mem, pos_vel_acc_avg[i],
						   window);
		p_data.push_back(pos_vel_acc_avg[i][0]);
		//****************************************************[PREPROCESS DATA]

// ============================================================================
// PREDICTION STARTS
// ============================================================================
slide 	  = false;
LABEL.mov = -1;
reshapeVector(LABEL.loc, 		num_locations);
reshapeVector(LABEL.surface, 	num_surface);

// 1. Contact trigger
// 1.1 Check if the object is within a sphere volume of the location areas
triggerContact(pos_vel_acc_avg[i][0], Graph_);

// 2. Prediction during motion
if (pos_vel_acc_avg[i][0].cluster_id < 0)
{
	flag_motion = true;

	// 2.1. Set flag to allow online learning/updates of the knowledge base
	// learn = true;

	// 2.2. Check if the trajectory is within the range of sector map
	checkSector(prediction, t_val,
				pos_vel_acc_avg[i][0], Graph_, Graph_mem,
				last_location, learn);

	// 2.3. Check for motion (moving/null)
	checkMotion(pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1],
				Graph_.getMovLabel(), Graph_.getSurface(),
				0.1, 0.97, vel_limit, LABEL);

	// 2.4. Prediction based on the trajectory error from sector map
	motionPrediction(prediction, t_val,
					 flag_predict, flag_predict_last,
					 predict_in, predict_err, predict_in_last,
					 pow_dec, Graph_);

	if (VERBOSE == 0 || VERBOSE == 1)
	{
		printf("Nr:%04d,  ", i);

		if (LABEL.mov < 0)
		{
			printf("LABEL: NULL\n");
		}
		else if (LABEL.mov == 1)
		{
			for(int ii=0;ii<num_surface;ii++)
			{
				if (LABEL.surface[ii] > 0)
				{
					printf("LABEL: %s on surface %d  ",
							Graph_.getMovLabel()[LABEL.mov].c_str(), ii);
					break;
				}
			}
		}
		else
		{
			printf("LABEL: %s  ", Graph_.getMovLabel()[LABEL.mov].c_str());
		}

		for(int ii=0;ii<num_locations;ii++)
		{
			printf(" %.4f ", predict_err[ii]);
		}

		for(int ii=0;ii<num_locations;ii++)
		{
			if (prediction[ii] == WITHIN_RANGE)
			{
				printf(" %s %.4f ", Graph_.getNode(ii).name.c_str(), predict_in[ii]);
			}
		}

		printf("\n");
	}

}
// 3. Prediction within location area
else
{
	flag_predict      = false;
	flag_predict_last = false;

	// 3.1. Location area prediction based on contact trigger
	locationPrediction(pos_vel_acc_avg[i][0].cluster_id,
					   pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1],
					   Graph_, 0.1, 0.97, vel_limit, LABEL);

	// 3.2. Check if it is moved back to the same location or not
	if (last_location == pos_vel_acc_avg[i][0].cluster_id && flag_motion)
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
	if (last_location != pos_vel_acc_avg[i][0].cluster_id && flag_motion)
	{
		if (learn)
		{
			// copy only the intended values for sectors
			// updating the values in memory
			int c = last_location * num_locations + pos_vel_acc_avg[i][0].cluster_id;
			int tmp = Graph_.getEdgeList()[c].size();
			for(int iii=0;iii<tmp;iii++)
			{
				Graph_mem.updateEdgeSector(
						Graph_.getEdgeList()[c][iii].sector_map,
						last_location, pos_vel_acc_avg[i][0].cluster_id, iii);
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

	last_location = pos_vel_acc_avg[i][0].cluster_id;
	flag_motion = false;

	if (VERBOSE == 0 || VERBOSE == 2)
	{
		printf("Nr:%04d,  ", i);

		for(int ii=0;ii<num_locations;ii++)
		{
			if (LABEL.loc[ii] > 0)
			{
				printf("LABEL: %s  ", Graph_.getNode(ii).name.c_str());
				break;
			}
			else if (LABEL.loc[ii] < 0)
			{
				printf("LABEL: Empty location Label.  ");
				if (LABEL.mov < 0)
				{
					printf("LABEL: NULL\n");
				}
				else if (LABEL.mov < 0)
				{
					for(int ii=0;ii<num_surface;ii++)
					{
						if (LABEL.surface[ii] > 0)
						{
							printf("LABEL: %s on surface %d  ",
									Graph_.getMovLabel()[LABEL.mov].c_str(), ii);
							break;
						}
					}
				}
				else
				{
					printf("LABEL: %s  ", Graph_.getMovLabel()[LABEL.mov].c_str());
				}
				break;
			}
		}

		printf("\n");
	}
}

// LABEL ONLY
if (VERBOSE == 3)
{
	if (pos_vel_acc_avg[i][0].cluster_id < 0)
	{
		printf("Nr:%04d,  ", i);

		if (LABEL.mov < 0)
		{
			printf("LABEL: NULL\n");
		}
		else if (LABEL.mov == 1)
		{
			for(int ii=0;ii<num_surface;ii++)
			{
				if (LABEL.surface[ii] > 0)
				{
					printf("LABEL: %s on surface %d  ",
							Graph_.getMovLabel()[LABEL.mov].c_str(), ii);
					break;
				}
			}
		}
		else
		{
			printf("LABEL: %s  ", Graph_.getMovLabel()[LABEL.mov].c_str());
		}

		if (*max_element(predict_in.begin(), predict_in.end()) > 0)
		{
			unsigned int tmptmp = distance(predict_in.begin(), max_element(predict_in.begin(), predict_in.end()));
			printf("%s  %.4f  ", Graph_.getNode(tmptmp).name.c_str(), *max_element(predict_in.begin(), predict_in.end()));
		}
		else
		{
			unsigned int tmptmp = distance(predict_err.begin(), max_element(predict_err.begin(), predict_err.end()));
			printf("%s  %.4f  ", Graph_.getNode(tmptmp).name.c_str(), *max_element(predict_err.begin(), predict_err.end()));
		}

		printf("\n");

	}
	else
	{
		printf("Nr:%04d,  ", i);

		for(int ii=0;ii<num_locations;ii++)
		{
			if (LABEL.loc[ii] > 0)
			{
				printf("LABEL: %s  ", Graph_.getNode(ii).name.c_str());
				break;
			}
			else if (LABEL.loc[ii] < 0)
			{
				printf("LABEL: Empty location Label.  ");
				if (LABEL.mov < 0)
				{
					printf("LABEL: NULL\n");
				}
				else if (LABEL.mov < 0)
				{
					for(int ii=0;ii<num_surface;ii++)
					{
						if (LABEL.surface[ii] > 0)
						{
							printf("LABEL: %s on surface %d  ",
									Graph_.getMovLabel()[LABEL.mov].c_str(), ii);
							break;
						}
					}
				}
				else
				{
					printf("LABEL: %s  ", Graph_.getMovLabel()[LABEL.mov].c_str());
				}
				break;
			}
		}

		printf("\n");

	}

}

// ============================================================================
// PREDICTION ENDS
// ============================================================================

	}

	showConnection(p_data,label_data,Graph_,color_code,true);

#endif

//		vector<double> x,y1,y2,y0;
//		for(int i=0;i<num_points;i++)
//		{
//			x.push_back(i);
//			y0.push_back(l2Norm(pos_vel_acc_avg[i][0]));
//			y1.push_back(l2Norm(pos_vel_acc_avg[i][1]));
//			y2.push_back(l2Norm(pos_vel_acc_avg[i][2]));
//		}
//
//		plotData(x, y0);
//		plotData(x, y1);
//		plotData(x, y2);

	cout << "END" << endl;

	return 0;
}
