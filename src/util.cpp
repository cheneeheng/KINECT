/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

//#define DBSCAN

#include "util.h"

#ifdef PC
	string SCENE_ = "../Scene/";
#else
	string SCENE_ = "Scene/";
#endif

// ============================================================================
// Modules
// ============================================================================

int learnLocationArea(
	string dirname_,
	string scene,
	string object)
{
	// [PARAMETERS]************************************************************
	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;
	// if more data points are available these 2 values can be increased.
	int minpts 					= 20;
	double epsilon 				= 0.015;
	// ************************************************************[PARAMETERS]

	// [VARIABLES]*************************************************************
	bool replace 				= false;
	string path					;
	int num_points 				= 0;
	int num_locations			= 0;
	int file_num 				= 0;
	vector<string> 				label;
	vector<int> 				file_eof;
	vector<point_t> 			points;
	vector<vector<string> > 	data_full;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<vector<point_t> > 	pos_vel_acc_mem; // motion->length
	vector<vector<unsigned char> > color_code; colorCode(color_code);
	Graph Graph(scene, object);
	printf("Initialization......Complete\n");
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	struct dirent **namelist;
	string name;
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dirname_ + namelist[i]->d_name;
		readFile(name.c_str(), data_full , ',');
		file_eof.push_back(data_full.size());
	}
	readSurfaceFile(Graph);
	printf("Reading training data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_full.size();
	reshapeVector(points, num_points);
	parseData2Point(data_full, points);
	printf("Parsing training data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [PREPROCESS DATA]*******************************************************
	reshapeVector(pos_vel_acc_avg, num_points);
	reshapeVector(pos_vel_acc_mem, 3);
	for(int i=0;i<num_points;i++)
	{
		if (i == file_eof[file_num])
		{
			file_num++;
			reshapeVector(pos_vel_acc_mem, 3);
		}
		if (i == 0)	{ pos_vel_acc_avg[i] = pos_vel_acc_avg1; }
		else		{ pos_vel_acc_avg[i] = pos_vel_acc_avg[i-1]; }
		preprocessDataLive(
				points[i], pos_vel_acc_mem, pos_vel_acc_avg[i], FILTER_WIN);
	}
	printf("Pre-processing data......Complete\n");
	// *******************************************************[PREPROCESS DATA]

	directoryCheck(SCENE_ + scene + "/" + object);

	// [LABELLING MOVEMENT]****************************************************
	data_full.clear();
	path =  SCENE_ + scene + "/" + object + "/data_mov.txt";
	readFile(path.c_str(), data_full , ',');
	labelMovement(Graph, data_full);
	writeLabelFile(Graph, path, 1);
	label.clear(); label = Graph.getMovLabel();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph.updateMovLabel(label);
		writeLabelFile(Graph, path, 1);
	}
	printf("Labeling of movements......Complete\n");
	// ****************************************************[LABELLING MOVEMENT]

	// [LABELLING LOCATION]****************************************************
	data_full.clear();
	path =  SCENE_ + scene + "/" + object + "/data_loc.txt";
	readFile(path.c_str(), data_full , ',');
	labelLocation_(Graph, pos_vel_acc_avg, data_full, epsilon, minpts);
	writeLabelFile(Graph, path, 0);
	label.clear(); label = Graph.getNodeName();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph.updateNodeName(label);
		writeLabelFile(Graph, path, 0);
	}
	printf("Labeling of location areas......Complete\n");
	// ****************************************************[LABELLING LOCATION]
	return 1;
}

int learnSector(
	string dirname_,
	string scene,
	string object)
{
	// [PARAMETERS]************************************************************
	int num_location_intervals	= 20;
	int num_sector_intervals 	= 36;
	// if more data points are available these 2 values can be increased.
	int minpts 					= 20;
	double epsilon 				= 0.015;
	double max_range 			= 0.05;
	// ************************************************************[PARAMETERS]

	// [VARIABLES]*************************************************************
	string path;
	int num_points 				= 0;
	int num_locations			= 0;
	int file_num 				= 0;
	vector<string> 				label;
	vector<int> 				file_eof;
	vector<point_t> 			points;
	vector<vector<string> > 	data_full;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<vector<point_t> > 	pos_vel_acc_mem; // motion->length
	vector<vector<unsigned char> > color_code; colorCode(color_code);
	Graph Graph(scene, object);
	printf("Initialization......Complete\n");
	// *************************************************************[VARIABLES]

	// [LEARNED DATA]**********************************************************
	directoryCheck(SCENE_ + scene + "/" + object);
	string path_tmp_src, path_tmp_dest;
	path_tmp_src 	= SCENE_ + scene + string("/ALL/data_loc.txt");
	path_tmp_dest 	= SCENE_ + scene + "/" + object + "/data_loc.txt";
	copyFile(path_tmp_src, path_tmp_dest);
	path_tmp_src 	= SCENE_ + scene + string("/ALL/data_mov.txt");
	path_tmp_dest 	= SCENE_ + scene + "/" + object + "/data_mov.txt";
	copyFile(path_tmp_src, path_tmp_dest);
	printf("Copying learned data......Complete\n");
	// **********************************************************[LEARNED DATA]

	// [READ FILE]*************************************************************
	struct dirent **namelist;
	string name;
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dirname_ + namelist[i]->d_name;
		readFile(name.c_str(), data_full , ',');
		file_eof.push_back(data_full.size());
		vector<string> last_line = data_full[data_full.size()-1];
	}
	readSurfaceFile(Graph);
	printf("Reading training data......Complete\n");
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	num_points = data_full.size();
	reshapeVector(points, num_points);
	parseData2Point(data_full, points);
	printf("Parsing training data......Complete\n");
	// ************************************************************[PARSE DATA]

	// [PREPROCESS DATA]*******************************************************
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
						   FILTER_WIN);
	}
	printf("Pre-processing data......Complete\n");
	// *******************************************************[PREPROCESS DATA]

	// [NODES]*****************************************************************
	data_full.clear();
	path =  SCENE_ + scene + "/" + object + "/data_mov.txt";
	readFile(path.c_str(), data_full , ',');
	readMovement (Graph, data_full);
	label.clear(); label = Graph.getMovLabel();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph.updateMovLabel(label);
		writeLabelFile(Graph, path, 1);
	}
	data_full.clear();
	path =  SCENE_ + scene + "/" + object + "/data_loc.txt";
	readFile(path.c_str(), data_full , ',');
	readLocation_(Graph, data_full);
	labelLocation_(Graph, pos_vel_acc_avg, data_full, epsilon, minpts);
	label.clear(); label = Graph.getNodeName();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph.updateNodeName(label);
		writeLabelFile(Graph, path, 0);
	}
	printf("Creating nodes for the clusters (action locations)......Complete\n");
	// *****************************************************************[NODES]

	//[EDGES]******************************************************************
	Graph.initEdge(num_location_intervals, num_sector_intervals);
	labelSector(Graph, pos_vel_acc_avg,	max_range, file_eof, color_code);
	directoryCheck(SCENE_ + scene + "/" + object);
	path = 	SCENE_ + scene + "/" + object + "/loc_data_beg.txt";
	writeLearnedDataFile(Graph, path, 0);
	path = 	SCENE_ + scene + "/" + object + "/loc_data_mid.txt";
	writeLearnedDataFile(Graph, path, 1);
	path = 	SCENE_ + scene + "/" + object + "/loc_data_end.txt";
	writeLearnedDataFile(Graph, path, 2);
	path = 	SCENE_ + scene + "/" + object + "/loc_data_tangent.txt";
	writeLearnedDataFile(Graph, path, 3);
	path = 	SCENE_ + scene + "/" + object + "/loc_data_normal.txt";
	writeLearnedDataFile(Graph, path, 4);
	path = 	SCENE_ + scene + "/" + object + "/counter.txt";
	writeLearnedDataFile(Graph, path, 5);
	path = 	SCENE_ + scene + "/" + object + "/sec_data_max.txt";
	writeLearnedDataFile(Graph, path, 6);
	path = 	SCENE_ + scene + "/" + object + "/sec_data_const.txt";
	writeLearnedDataFile(Graph, path, 7);
	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");
	// *****************************************************************[EDGES]
	return 1;
}

int testing(
	string dirname_,
	string scene,
	string object)
{
	// [VARIABLES]*************************************************************
	vector<vector<unsigned char> > color_code; colorCode(color_code);

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

	vector<int> 				file_eof;
	vector<point_t> 			points_test;
	vector<vector<string> > 	data_test;
	vector<vector<point_t> > 	pos_vel_acc_avg; // length->motion

	bool replace 			= false;

	bool flag_motion      	= false;
	bool flag_predict      	= false;
	bool flag_predict_last 	= false;
	bool learn 				= false;
	bool slide 				= false;
	int loc_last 			= 0;
	int surface_num_tmp 	= 0;
	double pow_dec 			= 1;

	string path;

	pred_t prediction;

	label_t LABEL;

	msg_t MSG;

	vector<int> 				loc_last_idxs;
	vector<double> 				t_val;
	vector<point_t> 			pos_vel_acc_avg1(3);
	vector<point_t > 		  	pos_vel_acc_mem1(3);
	vector<vector< point_t > > 	pos_vel_acc_mem; // motion->length
	vector<vector< point_t > > 	pva_avg;
	vector<string> 				label;

	// JUST FOR VISUALIZING
	vector<point_t> p_data; vector<string> label_data;
	// *************************************************************[VARIABLES]

	// [READ FILE]*************************************************************
	struct dirent **namelist;
	string name;
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dirname_ + namelist[i]->d_name;
		readFile(name.c_str(), data_test , ',');
		file_eof.push_back(data_test.size());
	}
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
	// [NODES]*****************************************************************
	data_test.clear();
	path =  SCENE_ + scene + "/" + object + "/data_mov.txt";
	readFile(path.c_str(), data_test , ',');
	readMovement (Graph_, data_test);
	label.clear(); label = Graph_.getMovLabel();
	if (replaceLabel(label))
	{
		remove(path.c_str());
		Graph_.updateMovLabel(label);
		writeLabelFile(Graph_, path, 1);
	}
	data_test.clear();
	path =  SCENE_ + scene + "/" + object + "/data_loc.txt";
	readFile(path.c_str(), data_test , ',');
	readLocation_(Graph_, data_test);
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

	for(int i=0;i<num_locations;i++)
		label_data.push_back(Graph_.getNode(i).name);
	// **********************************************************[LEARNED DATA]

	// [PREDICTION VARIABLES]**************************************************
	reshapeVector(prediction.pred, 			num_locations);
	reshapeVector(prediction.pred_in,		num_locations);
	reshapeVector(prediction.pred_in_last, 	num_locations);
	reshapeVector(prediction.pred_err,		num_locations);
	reshapeVector(pos_vel_acc_avg, 			num_points);
	reshapeVector(pos_vel_acc_mem, 			3);
	reshapeVector(loc_last_idxs, 			num_locations);
	reshapeVector(LABEL.loc, 				num_locations);
	reshapeVector(LABEL.sur, 				num_surfaces);
	LABEL.mov   = -1;
	MSG.num_loc = num_locations;
	MSG.num_sur = num_surfaces;
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
						   FILTER_WIN);
		//****************************************************[PREPROCESS DATA]

// ============================================================================
// PREDICTION STARTS
// ============================================================================

		slide 	  = false;
		LABEL.mov = -1;
		MSG.idx   =  i;
		reshapeVector(LABEL.loc, num_locations);
		reshapeVector(LABEL.sur, num_surfaces);

		// 1. Contact trigger
		// 1.1 Check if the object is within a sphere volume of the location areas
		triggerContact(pos_vel_acc_avg[i][0], Graph_);
		p_data.push_back(pos_vel_acc_avg[i][0]);

		// 2. Prediction during motion
		if (pos_vel_acc_avg[i][0].cluster_id < 0)
		{
			pva_avg.push_back(pos_vel_acc_avg[i]);
			flag_motion = true;

			predictionEdge(
					prediction, Graph_,
					pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1],
					loc_last, loc_last_idxs, LIMIT, LABEL,
					flag_predict, flag_predict_last, pow_dec);

			bool zeros =
					all_of(
							prediction.pred_err.begin(),
							prediction.pred_err.end(),
							[](double ii) { return ii==0.0; });

			if (!zeros)
			{
				MSG.msg 	= 1;
				MSG.label 	= LABEL;
				MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
				MSG.pred 	= prediction;
			}
			outputMsg(MSG, Graph_);
		}

		// 3. Prediction within location area
		else
		{
			flag_predict      = false;
			flag_predict_last = false;
			reshapeVector(loc_last_idxs,num_locations);

			predictionNode(
					pva_avg, pos_vel_acc_avg[i][0], pos_vel_acc_avg[i][1],
					loc_last, pos_vel_acc_avg[i][0].cluster_id,
					Graph_, LIMIT, LABEL,
					flag_motion, learn);

			loc_last 	= pos_vel_acc_avg[i][0].cluster_id;
			flag_motion = false;

			pva_avg.clear();

			MSG.msg 	= 2;
			MSG.label 	= LABEL;
			MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
			MSG.pred 	= prediction;
			outputMsg(MSG, Graph_);
		}

		bool zeros =
				all_of(
						prediction.pred_err.begin(),
						prediction.pred_err.end(),
						[](double ii) { return ii==0.0; });

		if (pos_vel_acc_avg[i][0].cluster_id < 0)
		{
			if (!zeros)
			{
				MSG.msg 	= 3;
				MSG.label 	= LABEL;
				MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
				MSG.pred 	= prediction;
			}
		}
		else
		{
			MSG.msg 	= 3;
			MSG.label 	= LABEL;
			MSG.loc_idx = pos_vel_acc_avg[i][0].cluster_id;
			MSG.pred 	= prediction;
		}
		outputMsg(MSG, Graph_);


// ============================================================================
// PREDICTION ENDS
// ============================================================================

	} // points

	showConnection(p_data,label_data,Graph_,color_code,true);
	return 1;
}

// ============================================================================
// Data
// ============================================================================

void parseData2Point(
	vector<vector<string> > data_full,
	vector<point_t> &points)
{
	for(int i=0;i<points.size();i++)
	{
		points[i].x = atof(data_full[i][2].c_str());
		points[i].y = atof(data_full[i][3].c_str());
		points[i].z = atof(data_full[i][4].c_str());
		points[i].cluster_id = UNCLASSIFIED;
	}
}

void preprocessDataLive(
	point_t pos_,
	vector< vector< point_t > > &pos_vel_acc_mem_, // motion -> length(empty at beginning)
	vector<point_t> &pos_vel_acc_avg_, //motion
	unsigned int window_)
{
	point_t vel = minusPoint(pos_, pos_vel_acc_avg_[0]);
	point_t acc = minusPoint(vel , pos_vel_acc_avg_[1]);
	vector<point_t> tmp(3);
	tmp[0] = pos_; tmp[1] = vel; tmp[2] = acc;
	for(int i=0;i<3;i++)
	{
		if(pos_vel_acc_mem_[i].size() == window_)
		{
			pos_vel_acc_avg_[i] =
					movingAverage(tmp[i], pos_vel_acc_mem_[i]);
		}
		else if (pos_vel_acc_mem_[i].size()>0)
		{
			pos_vel_acc_avg_[i] =
					averagePointIncrement(tmp[i], pos_vel_acc_mem_[i]);
		}
		else
		{
			pos_vel_acc_mem_[i].push_back(tmp[i]);
			pos_vel_acc_avg_[i] = tmp[i];
			for(int ii=i+1;ii<3;ii++)
			{
				pos_vel_acc_avg_[ii].cluster_id = UNCLASSIFIED;
				pos_vel_acc_avg_[ii].x =
				pos_vel_acc_avg_[ii].y =
				pos_vel_acc_avg_[ii].z = 0;
			}
			break;
		}
		pos_vel_acc_avg_[i].cluster_id = UNCLASSIFIED;
	}
}

// ============================================================================
// EXTRAS
// ============================================================================

void outputMsg(
	msg_t MSG_,
	Graph Graph_)
{
	switch(MSG_.msg)
	{
		case 1 :
			// 1. message for prediction during motion.
			if (VERBOSE == 0 || VERBOSE == 1)
			{
				printf("Nr:%04d,  ", MSG_.idx);
				printf("LABEL : ");
				if (MSG_.label.mov < 0)
				{
					printf("NULL ");
				}
				else if (MSG_.label.mov == 1)
				{
					for(int ii=0;ii<MSG_.num_sur;ii++)
					{
						if (MSG_.label.sur[ii] > 0)
						{
							printf("%s on surface %d  ",
									Graph_.getMovLabel()[MSG_.label.mov].c_str(),
									ii);
							break;
						}
					}
				}
				else
				{
					printf("%s  ",
							Graph_.getMovLabel()[MSG_.label.mov].c_str());
				}
				for(int ii=0;ii<MSG_.num_loc;ii++)
				{
					printf(" %.4f ", MSG_.pred.pred_err[ii]);
				}
				for(int ii=0;ii<MSG_.num_loc;ii++)
				{
					if (MSG_.pred.pred[ii] == WITHIN_RANGE)
					{
						printf(" %s %.4f ",
								Graph_.getNode(ii).name.c_str(),
								MSG_.pred.pred_in[ii]);
					}
				}
				printf("\n");
			}
			break;

		case 2 :
			// 2. message for prediction for location areas.
			if (VERBOSE == 0 || VERBOSE == 2)
			{
				printf("Nr:%04d,  ", MSG_.idx);
				printf("LABEL : ");
				for(int ii=0;ii<MSG_.num_loc;ii++)
				{
					if (MSG_.label.loc[ii] > 0)
					{
						if (MSG_.label.mov < 0)
						{
							printf("NULL ");
						}
						else if (MSG_.label.mov == 1)
						{
							for(int ii=0;ii<MSG_.num_sur;ii++)
							{
								if (MSG_.label.sur[ii] > 0)
								{
									printf("%s on surface %d ",
											Graph_.getMovLabel()
												[MSG_.label.mov].c_str(),
											ii);
									break;
								}
							}
						}
						else
						{
							printf("%s ",
									Graph_.getMovLabel()
										[MSG_.label.mov].c_str());
						}

						printf("%s ",
								Graph_.getNode(ii).name.c_str());
						break;
					}
					else if (MSG_.label.loc[ii] < 0)
					{
						printf("Empty location Label.  ");
						if (MSG_.label.mov < 0)
						{
							printf("NULL ");
						}
						else if (MSG_.label.mov == 1)
						{
							for(int ii=0;ii<MSG_.num_sur;ii++)
							{
								if (MSG_.label.sur[ii] > 0)
								{
									printf("%s on surface %d ",
											Graph_.getMovLabel()
												[MSG_.label.mov].c_str(),
											ii);
									break;
								}
							}
						}
						else
						{
							printf("%s ",
									Graph_.getMovLabel()
										[MSG_.label.mov].c_str());
						}
						break;
					}
				}
				printf("\n");
			}
			break;

		case 3:
			// 3. LABEL ONLY MESSSAGE
			if (VERBOSE == 3)
			{
				if (MSG_.loc_idx < 0)
				{
					printf("Nr:%04d,  ", MSG_.idx);
					printf("LABEL : ");
					if (MSG_.label.mov < 0)
					{
						printf("NULL ");
					}
					else if (MSG_.label.mov == 1)
					{
						for(int ii=0;ii<MSG_.num_sur;ii++)
						{
							if (MSG_.label.sur[ii] > 0)
							{
								printf("%s on surface %d ",
										Graph_.getMovLabel()
											[MSG_.label.mov].c_str(),
										ii);
								break;
							}
						}
					}
					else
					{
						printf("%s ",
								Graph_.getMovLabel()[MSG_.label.mov].c_str());
					}
					if (*max_element(
							MSG_.pred.pred_in.begin(),
							MSG_.pred.pred_in.end()) > 0)
					{
						unsigned int tmptmp =
								distance(
										MSG_.pred.pred_in.begin(),
										max_element(
												MSG_.pred.pred_in.begin(),
												MSG_.pred.pred_in.end()));
						printf("%s  %.4f  ",
								Graph_.getNode(tmptmp).name.c_str(),
								*max_element(
										MSG_.pred.pred_in.begin(),
										MSG_.pred.pred_in.end()));
					}
					else
					{
						unsigned int tmptmp =
								distance(
										MSG_.pred.pred_err.begin(),
										max_element(
												MSG_.pred.pred_err.begin(),
												MSG_.pred.pred_err.end()));
						printf("%s  %.4f  ",
								Graph_.getNode(tmptmp).name.c_str(),
								*max_element(
										MSG_.pred.pred_err.begin(),
										MSG_.pred.pred_err.end()));
					}
					printf("\n");
				}
				else
				{
					printf("Nr:%04d,  ", MSG_.idx);
					printf("LABEL : ");
					for(int ii=0;ii<MSG_.num_loc;ii++)
					{
						if (MSG_.label.loc[ii] > 0)
						{
							if (MSG_.label.mov < 0)
							{
								printf("NULL ");
							}
							else if (MSG_.label.mov == 1)
							{
								for(int ii=0;ii<MSG_.num_sur;ii++)
								{
									if (MSG_.label.sur[ii] > 0)
									{
										printf("%s on surface %d ",
												Graph_.getMovLabel()
													[MSG_.label.mov].c_str(),
												ii);
										break;
									}
								}
							}
							else
							{
								printf("%s ",
										Graph_.getMovLabel()
											[MSG_.label.mov].c_str());
							}

							printf("%s ",
									Graph_.getNode(ii).name.c_str());
							break;
						}
						else if (MSG_.label.loc[ii] < 0)
						{
							printf("Empty location Label.  ");
							if (MSG_.label.mov < 0)
							{
								printf("NULL ");
							}
							else if (MSG_.label.mov < 0)
							{
								for(int ii=0;ii<MSG_.num_sur;ii++)
								{
									if (MSG_.label.sur[ii] > 0)
									{
										printf("%s on surface %d ",
												Graph_.getMovLabel()
													[MSG_.label.mov].c_str(),
												ii);
										break;
									}
								}
							}
							else
							{
								printf("%s ",
										Graph_.getMovLabel()
											[MSG_.label.mov].c_str());
							}
							break;
						}
					}
					printf("\n");
				}
			}
			break;
	}
}





//---------------------------------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------------------------------
// General Functions

void writePointFile(
	point_t *p,
	unsigned int num_points)
{
	remove("data.txt");
	for(unsigned int i=0;i<num_points;i++)
	{
		// write values into data.txt
		std::ofstream write_file("data.txt", std::ios::app);
		write_file << p[i].x << ","
				   << p[i].y << ","
				   << p[i].z << ","
				   << p[i].cluster_id
				   << "\n";
	}
}

bool checkSurfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_,
	double surface_limit_,
	double surface_range_limit_)
{
	if(surfaceDistance(pos_, surface_) < surface_limit_) // less than 10 cm off the surface
		if(surfaceRange(pos_, pos_surface_, surface_) < surface_range_limit_)
			return true;
		else
			return false;
	else
		return false;
}

int checkMoveSlide(
	point_t pos_,
	point_t vel_,
	vector<double> surface_,
	double surface_limit_,
	double angle_limit_)
{
	cout << surfaceDistance(pos_, surface_) << "  ";
	cout << surfaceAngle(vel_, surface_) << endl;
	if(surfaceDistance(pos_, surface_) < surface_limit_) // less than 10 cm off the surface
		if(surfaceAngle(vel_, surface_) > angle_limit_)
			return 1;
		else
			return 0;
	else
		return 0;
}

double checkMoveSlideOutside(
		point_t pos_,
		point_t vel_,
		double **surface_,
		unsigned int num_surfaces_)
{
	vector<double> A(3);
	double dir_tmp = INFINITY, dist_tmp = INFINITY;
	for(int i=0;i<num_surfaces_;i++)
	{
		A[0] = surface_[i][0];
		A[1] = surface_[i][1];
		A[2] = surface_[i][2];
		dir_tmp  = l2Norm(crossProduct(A,point2vector(vel_)));
		dist_tmp = surface_[i][0]*pos_.x +
						  surface_[i][1]*pos_.y +
						  surface_[i][2]*pos_.z -
						  surface_[i][3];
//		cout << dir_tmp << " " << dist_tmp << " ";
	}
	return 0.0;
}


/*
// UNUSED---------------------------------------------------------------------------------------------------------------------
void writeSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > sector_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << num_locations_ 		  << "\n";
		write_file << num_location_intervals_ << "\n";
		write_file << num_sector_intervals_   << "\n";
		for(int i=0;i<sector_.size();i++)
		{
			for(int ii=0;ii<sector_[i].size();ii++)
			{
				for(int iii=0;iii<sector_[i][ii].size();iii++)
				{
					write_file << sector_[i][ii][iii];
					if (i<sector_[i][ii].size()-1)
						write_file << ",";
				}
				write_file << "\n";
			}
		}
	}
}

void readSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > &sector_)
{
	vector<vector<string> > data;
	readFile(path_.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Sector data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][0].c_str());
		num_location_intervals 	= atoi(data[1][0].c_str());
		num_sector_intervals 	= atoi(data[2][0].c_str());

	    reshapeVector(sector_,Sqr(num_locations));
	    vector<vector<double> > sector1(num_location_intervals);
		       vector<double> 	sector2(num_sector_intervals);

		for(int i=0;i<Sqr(num_locations);i++)
		{
			sector_[i] = sector1;
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				sector_[i][ii] = sector2;
			}
		}

		for(int i=0;i<num_locations;i++)
		{
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				int tmp = i*num_location_intervals+ii+3;
				for(int iii=0;iii<num_sector_intervals;iii++)
				{
					sector_[i][ii][iii] = atof(data[tmp][iii].c_str());
				}
			}
		}
	}
	// Should we add the option to edit the data?
}

void writeLocLabelFile(
	Graph Graph_,
	string path_)
{
	if (!ifstream(path_))
	{
		vector<node_tt> node_tmp = Graph_.getNodeList();
		ofstream write_file(path_, ios::app);
		for(int i=0;i<node_tmp.size();i++)
		{
			write_file << node_tmp[i].name       << ","
					   << node_tmp[i].location.x << ","
					   << node_tmp[i].location.y << ","
					   << node_tmp[i].location.z << ","
					   << node_tmp[i].boundary   << ","
					   << node_tmp[i].surface	 << ","
					   << node_tmp[i].surface_boundary;
			write_file << "\n";
		}
	}
}

void writeMovLabelFile(
	string path_,
	vector<string> label_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		for(int i=0;i<label_.size();i++)
		{
			write_file << label_[i];
			if (i<label_.size()-1)
				write_file << ",";
		}
		write_file << "\n";
	}
}

void writeCounterFile(
	Graph Graph_,
	string path_,
	int type_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	sector_para_t para = Graph_.getSectorPara();

	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << "Locations," 			<< num_locations 	<< "\n";
		write_file << "Location Intervals," << para.loc_int 	<< "\n";
		write_file << "Sector Intervals," 	<< para.sec_int 	<< "\n";
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				write_file << "Edge,"    << i
						   << ",Number," << ii << "\n";
				write_file << Graph_.getCounter(i,ii)
						   << "\n";
			}
		}
	}
}

void writeSectorFile(
	Graph Graph_,
	string path_,
	int type_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	sector_para_t para = Graph_.getSectorPara();

	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << "Locations," 			<< num_locations 	<< "\n";
		write_file << "Location Intervals," << para.loc_int 	<< "\n";
		write_file << "Sector Intervals," 	<< para.sec_int 	<< "\n";
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				vector<double> sector       = edges[i][ii].sector_map;
				vector<double> sector_const = edges[i][ii].sector_const;
				write_file << "Edge,"    << i
						   << ",Number," << ii << "\n";
				for(int iii=0;iii<sector.size();iii++)
				{
					switch(type_)
					{
						case 0:
							write_file << sector[iii];
							break;
						case 1:
							write_file << sector_const[iii];
							break;
					}
					if (iii<sector.size()-1)
						write_file << ",";
					else
						write_file << "\n";
				}
			}
		}
	}
}

*/




