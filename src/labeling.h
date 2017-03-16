/*
 * labelling.h
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#ifndef LABELING_H_
#define LABELING_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "dbscan.h"

// ============================================================================
// Mov, Loc Labels
// ============================================================================

void decideBoundary(
	point_t 				&point1_,
	point_t 				&point2_,
	vector<point_t> 		locations_,
	vector<double> 			locations_boundary_,
	vector<vector<double> > surfaces_,
	vector<int>     		surfaces_num_,
	vector<double> 			surfaces_boundary_);

void contactBoundary(
	vector<point_t> &p,
	vector<point_t> locations,
	vector<double> 	&location_boundary,
	bool 			learn);

void labelMovement(
	Graph &Graph_,
	vector<vector<string> > data_);

void labelLocation(
	vector<vector<string> > data_,
	vector<point_t> 		&points_,
	vector<point_t> 		&locations_,
	vector<double>  		&locations_boundary_,
	vector<string>  		&label_,
	vector<vector<double> > surfaces_,
	vector<int>     		&surfaces_num_,
	vector<double>     		&surfaces_boundary_,
	double 					epsilon_,
	int    					minpts_);

void labelLocation_(
	Graph &Graph_,
	vector<vector<point_t> > &pos_vel_acc_,
	vector<vector<string> > data_,
	double epsilon_,
	int minpts_);

void labelSector(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	double max_range_,
	vector<int> file_eof_,
	vector<vector<unsigned char> > color_code_);

void readingLocation(
	vector<point_t> &locations,
	vector<double> &location_boundary,
	vector<string> &LABEL_LOC,
	int obj);

bool replaceLabel(
	vector<string> &label_);

// ============================================================================
// Sector Curve
// ============================================================================

bool checkDirection(
	vector<double> A,
	vector<double> B);

double determineLocationInterval(
	int &loc_idx_,
	int &loc_last_idx_,
	int loc_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_,
	bool prediction=false);

void determineLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	int loc_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_,
	bool prediction=false);

void determineSectorInterval(
	int &sec_idx_,
	int loc_idx_,
	int sec_int_,
	point_t &delta_t_,
	point_t point_,
	vector<point_t> mid_,
	vector<point_t> tangent_,
	vector<point_t> normal_);

void determineSectorMap(
	vector<double> &sector_map,
	int &ind_loc_last_,
	int loc_int_,
	int sec_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_,
	vector<point_t> normal_,
	bool adjust=false);

void fitSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg_,
	vector<point_t> &points_est,
	vector<point_t> &coeffs_,
	int edge_xy_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_);

void checkSectorCurve(
	Graph &Graph_,
	vector<point_t> &points_est,
	int edge_xy_,
	int label1_,
	int label2_);

void checkSectorCurveConstraint(
	Graph &Graph_,
	double max_range_,
	int edge_xy_,
	int label1_,
	int label2_);

void adjustSectorCurve(
	Graph &Graph_,
	vector<point_t> &points_est,
	vector<point_t> coeffs_,
	int edge_xy_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_);

void updateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg_,
	int edge_xy_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_,
	double max_range_);

void generateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	double max_range_,
	vector<int> file_eof_);

void fillLocationData(
	Graph &Graph_);



#endif /* LABELING_H_ */
