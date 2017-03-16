/*
 * prediction.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#include "prediction.h"

// ============================================================================
// Prediction
// ============================================================================

void triggerContact(
	point_t &point_,
	Graph Graph_)
{
	vector<vector<double> > surfaces = Graph_.getSurface();
	vector<node_tt> nodes 	= Graph_.getNodeList();
	int num_locations 		= nodes.size();
	vector<point_t> locations(num_locations);
	vector<double>  locations_boundary(num_locations);
	vector<int>  	surfaces_num(num_locations);
	vector<double>  surfaces_boundary(num_locations);
	for(int i=0;i<num_locations;i++)
	{
		locations[i]         	= nodes[i].location;
		locations_boundary[i]	= nodes[i].boundary;
		surfaces_num[i]			= nodes[i].surface;
		surfaces_boundary[i]	= nodes[i].surface_boundary;
	}
	decideBoundary(
			point_, point_, locations, locations_boundary,
			surfaces, surfaces_num, surfaces_boundary);
}

void checkMotion(
	point_t pos_,
	point_t vel_,
	vector<vector<double> > surface_,
	limit_t limit,
	label_t &LABEL_)
{
	int sur_idx = -1;
	double dis_tmp;
	double dis_tmp1;
	double ang_tmp;
	vector<double> dis_tmp2(surface_.size());
	vector<double> ang_tmp2(surface_.size());

	if (l2Norm(vel_) < limit.vel)
	{
		LABEL_.mov = -1;
	}
	else
	{
		for(int ii=0;ii<surface_.size();ii++)
		{
			point_t s_tmp;
			s_tmp.x = surface_[ii][4];
			s_tmp.y = surface_[ii][5];
			s_tmp.z = surface_[ii][6];
			dis_tmp = 0.25*l2Norm(minusPoint(pos_,s_tmp)) +
						0.75*abs(surfaceDistance(pos_, surface_[ii]));
			ang_tmp = surfaceAngle(vel_, surface_[ii]);
//			cout << ii << " " << ang_tmp << " " << dis_tmp << " " << l2Norm(minusPoint(pos_,s_tmp)) << " " << abs(surfaceDistance(pos_, surface_[ii])) << endl;
			if (dis_tmp < limit.sur_dis &&
				ang_tmp > limit.sur_ang)
			{
				dis_tmp2[ii] = dis_tmp;
				ang_tmp2[ii] = ang_tmp;
			}
			else
			{
				dis_tmp2[ii] = INFINITY;
				ang_tmp2[ii] = INFINITY;
			}
		}

		if (*min_element(dis_tmp2.begin(), dis_tmp2.end())!=INFINITY)
		{
			sur_idx = distance(dis_tmp2.begin(), min_element(dis_tmp2.begin(), dis_tmp2.end()));
			LABEL_.mov = 1;
			LABEL_.sur[sur_idx] = 1;
		}
		else
		{
			LABEL_.mov = 0;
		}
	}
}

void checkSector(
	pred_t &prediction_,
	vector<double> &t_val_,
	vector<int> &loc_last_idxs_,
	point_t point_,
	Graph &Graph_,
	int label1_)
{
	int sec_int 				= Graph_.getSectorPara().sec_int;
	int loc_int 				= Graph_.getSectorPara().loc_int;

	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();

	bool flag=false;

	for(int i=0;i<num_locations;i++)
	{
		prediction_.pred[i] = EXCEED_RANGE;
		t_val_[i]	   		= 0;

		if(label1_ == i) continue;

		int edge_xy = label1_*num_locations+i;
		int loc_idx = -1;
		int sec_idx = -1;
		int ind_ls 	=  0;
		vector<double> sector_map 	= Graph_.getEdgeList()[edge_xy][0].sector_map;
		vector<double> sector_const	= Graph_.getEdgeList()[edge_xy][0].sector_const;
		vector<point_t> tan 		= Graph_.getEdgeList()[edge_xy][0].tan;
		vector<point_t> nor 		= Graph_.getEdgeList()[edge_xy][0].nor;
		vector<point_t> loc_beg		= Graph_.getEdgeList()[edge_xy][0].loc_start;
		vector<point_t> loc_mid		= Graph_.getEdgeList()[edge_xy][0].loc_mid;
		vector<point_t> loc_end		= Graph_.getEdgeList()[edge_xy][0].loc_end;
		vector<double>  sector_map_mem = sector_map;
		point_t delta_t;
		double tmp_dis = 0.0;

		// [SECTOR MAP]********************************************************
		tmp_dis =
		determineLocationInterval(
				loc_idx, loc_last_idxs_[i], loc_int, point_,
				loc_beg, loc_mid, loc_end, tan, true);

		determineSectorInterval(
				sec_idx, loc_idx, sec_int, delta_t, point_,
				loc_mid, tan, nor);

		if (tmp_dis > 0.075) //##NEED TO VERIFY
		{
			prediction_.pred[i] = OUT_OF_BOUND;
			t_val_[i]      		= 
					l2Norm(minusPoint(point_,Graph_.getNode(i).location))-
					invPdfExp(0.01, 0.0, Graph_.getNode(i).boundary);
		}
		else
		{
			ind_ls  = loc_idx*sec_int + sec_idx;
			if (loc_idx < loc_int && sec_idx < sec_int)
			{
				if (l2Norm(delta_t) <= sector_map[ind_ls])
				{
					prediction_.pred[i] = WITHIN_RANGE;
					t_val_[i]      		= l2Norm(delta_t);
				}
				else
				{
					prediction_.pred[i] = OUT_OF_RANGE;
					t_val_[i] 			= l2Norm(delta_t) - sector_map[ind_ls];
				}
			}
		}
		// ********************************************************[SECTOR MAP]
	}
}

void motionPrediction(
	pred_t &prediction_,
	vector<double> &t_val_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	double &pow_dec_,
	Graph Graph_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();

	reshapeVector(prediction_.pred_in,num_locations);
	for(int ii=0;ii<num_locations;ii++)
	{
		if (prediction_.pred[ii] == WITHIN_RANGE)
		{
			flag_predict_      		 = false;
			flag_predict_last_ 		 = true;
			prediction_.pred_in[ii]	 = 1.0;
			prediction_.pred_err[ii] = 0.999999;
		}
		else if (prediction_.pred[ii] == OUT_OF_RANGE)
		{
			prediction_.pred_in[ii]  = 0.0;
			prediction_.pred_err[ii] = pdfExp(0.01, 0.0, t_val_[ii]); // NEED TO CHANGE ######
			// with 0.01 variance, a difference of 10cm will have 0.60653065971 prob.
		}
		else if (prediction_.pred[ii] == OUT_OF_BOUND)
		{
			prediction_.pred_in[ii]  = 0.0;
			prediction_.pred_err[ii] = pdfExp(0.005, 0.0, t_val_[ii]); // NEED TO CHANGE ######
			// with 0.01 variance, a difference of 10cm will have 0.60653065971 prob.
		}
		else
		{
			prediction_.pred_in[ii]  = 0.0;
			prediction_.pred_err[ii] = 0.0;
		}
	}

	normalizeData(prediction_.pred_in);
	//normalizeData(prediction_.pred_err);

	if (flag_predict_)
	{
		for(int ii=0;ii<num_locations;ii++)
		{
			if (prediction_.pred_in_last[ii] > 0 &&
				prediction_.pred[ii] > WITHIN_RANGE)
			{
				prediction_.pred_err[ii] =
						prediction_.pred_err[ii] * (1-pow(0.5,pow_dec_)) + pow(0.5,pow_dec_);
			}
			else if (prediction_.pred_err[ii] != 1.0) // prevent only 1 valid location prediction case
			{
				prediction_.pred_err[ii] =
						prediction_.pred_err[ii] * (1-pow(0.5,pow_dec_));
			}
		}
		pow_dec_ += 0.5;
	}
	else
	{
		for(int ii=0;ii<num_locations;ii++)
			prediction_.pred_in_last[ii] = prediction_.pred_in[ii];
		pow_dec_ = 1;
	}

	flag_predict_ = flag_predict_last_;

}

void locationPrediction(
	Graph Graph_,
	point_t pos_,
	point_t vel_,
	int label2_,
	limit_t limit_,
	label_t &label_)
{
	// check if label is empty
	if (!strcmp(Graph_.getNode(label2_).name.c_str(),""))
		label_.loc[label2_] = -1;
	else
		label_.loc[label2_] =  1;

	checkMotion(pos_, vel_, Graph_.getSurface(), limit_, label_);
}

void predictionNode(
	vector<vector<point_t> > pva_avg,
	point_t pos_,
	point_t vel_,
	int label1_,
	int label2_,
	Graph &Graph_,
	limit_t limit_,
	label_t &label_,
	bool flag_motion_,
	bool learn_)
{
	// 3.1. Location area prediction based on contact trigger
	locationPrediction(Graph_, pos_, vel_, label2_, limit_, label_);

	// 3.2. Check if it is moved back to the same location or not
	if (label1_ == label2_ && flag_motion_)
	{
		cout << " (same last location...)";
	}

	// 3.3. Update sector map if learn flag is set
	if (label1_ != label2_ && flag_motion_ && learn_)
	{
		vector<point_t> points_est, coeffs;
		int num_loc 		= Graph_.getNodeList().size();
		float max_range 	= 0.05;
		fitSectorCurve(
				Graph_, pva_avg, points_est, coeffs,
				label1_*num_loc+label2_,
				0, pva_avg.size(), label1_, label2_);
		checkSectorCurve(
				Graph_, points_est,
				label1_*num_loc+label2_,
				label1_, label2_);
		adjustSectorCurve(
				Graph_, points_est, coeffs,
				label1_*num_loc+label2_,
				0, pva_avg.size(), label1_, label2_);
		checkSectorCurveConstraint(
				Graph_, max_range,
				label1_*num_loc+label2_,
				label1_, label2_);
		Graph_.incrementCounter(
				label1_*num_loc+label2_, 0);
	}
}

void predictionEdge(
	pred_t &prediction_,
	Graph &Graph_,
	point_t pos_,
	point_t vel_,
	int label1_,
	vector<int> &last_loc_,
	limit_t limit_,
	label_t &label_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	double &pow_dec_)
{
	vector<double> t_val(Graph_.getNodeList().size());

	// 2.1. Set flag to allow online learning/updates of the knowledge base
	// learn = true;

	// 2.2. Check if the trajectory is within the range of sector map
	checkSector(prediction_, t_val, last_loc_, pos_, Graph_, label1_);

//	for(int i=0;i<t_val.size();i++)
//	cout << t_val[i] << endl;

	// 2.3. Check for motion (moving/null)
	checkMotion(pos_, vel_, Graph_.getSurface(), limit_ , label_);

	// 2.4. Prediction based on the trajectory error from sector map
	motionPrediction(
			prediction_, t_val, flag_predict_, flag_predict_last_,
			pow_dec_, Graph_);
}



