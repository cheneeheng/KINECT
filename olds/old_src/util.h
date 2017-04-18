/*
 * util.h
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"

// ============================================================================
// dbscan
// ============================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_t *p);

void combineNearCluster(
	vector<point_t> &points,
	vector<point_t> &locations);

void decideBoundary(
	point_t &p,
	vector<point_t> location,
	vector<double> location_boundary);

void contactBoundary(
	vector<point_t> &p,
	vector<point_t> locations,
	vector<double> &location_boundary,
	bool learn);

// ============================================================================
// B-spline
// ============================================================================

void curveFit(
	vector<point_t> points_,
	vector<point_t> &curves_);

void polyCurveFit(
	vector<double> points_,
	vector<double> &coeff_,
	vector<double> &cov_);

void polyCurveFitPoint(
	vector<point_t> points_,
	vector<point_t> &points_est_,
	vector<point_t> &coeffs_,
	vector<point_t> &covs_,
	bool est_);

void polyCurveFitEst(
	vector<double> &points_,
	int num_points_,
	vector<double> coeffs_,
	vector<double> covs_);

void polyCurveLength(
	double &length_,
	double a_,
	double b_,
	vector<point_t> coeffs_);

// ============================================================================
// Data
// ============================================================================

void parseData2Point(
	vector<vector<string> > data_full,
	vector<point_t> &points);

void preprocessDataLive(
	point_t pos_,
	vector< vector< point_t > > &pos_vel_acc_mem_, // motion -> length(empty at beginning)
	vector<point_t> &pos_vel_acc_avg_, //motion
	unsigned int window_);

// ============================================================================
// Files
// ============================================================================

void writeSurfaceFile(
	Graph Graph_);

void writeMovLabelFile(
	string path_,
	vector<string> label_);

void writeLocLabelFile(
	string path_,
	vector<string> label_,
	vector<point_t> locations_,
	vector<double> boundary_,
	vector<int> surface_num_container_);

void writeLocationFile(
	Graph Graph_,
	string path_,
	int type_);

void writeSectorFile(
	Graph Graph_,
	string path_,
	int type_);

void writeSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > sector_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_);

void writeCounterFile(
	Graph Graph_,
	string path_,
	int type_);

int fileSelect(
	const struct dirent *entry);

void readFile(
	const char *name,
	vector<vector<string> > &data_full,
	char delimiter);

void readSurfaceFile(
	Graph &Graph_);

void readLocation(
	string path_,
	vector<string> &LABEL_LOC_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_,
	vector<int> &surface_num_);

void readLocation_(
	Graph &Graph_);

void readMovement(
	Graph &Graph_);

void readSectorFile(
	Graph &Graph_,
	int maxminconst_);

void readLocationFile(
	Graph &Graph_,
	int type_);

void readSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > &sector_);

void readCounterFile(
	Graph &Graph_,
	int type_);

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
	vector<point_t> tangent_);

void determineLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	int loc_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_);

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
	int label2_);

void generateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	vector<int> file_eof_);

void fillLocationData(
	Graph &Graph_);

// ============================================================================
// Labels
// ============================================================================

void labelMovement(
	Graph Graph_);

void labelLocation(
	string path_,
	vector<point_t> &points_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_,
	vector<string> &label_,
	vector<int> &surface_num_,
	double epsilon_,
	int minpts_);

void labelLocation_(
	Graph &Graph_,
	vector<vector<point_t> > &pos_vel_acc_avg_,
	double epsilon_,
	int minpts_);

void labelSector(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	double max_range_,
	int kernel_size_x_,
	int kernel_size_y_,
	vector<int> file_eof_,
	vector<unsigned char*> color_code_);

void readingLocation(
	vector<point_t> &locations,
	vector<double> &location_boundary,
	vector<string> &LABEL_LOC,
	int obj);

// ============================================================================
// Prediction
// ============================================================================

void triggerContact(
	point_t &p_,
	Graph Graph_);

void checkMotion(
	point_t pos_,
	point_t vel_,
	vector<vector<double> > surface_,
	limit_t limit,
	label_t &LABEL_);

void checkSector(
	pred_t &prediction_,
	vector<double> &t_val_,
	vector<int> &last_loc_,
	point_t pos_,
	Graph &Graph_,
	int label1_);

void motionPrediction(
	vector<int> &prediction_,
	vector<double> &t_val_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	vector<double> &predict_in_,
	vector<double> &predict_err_,
	vector<double> &predict_in_last_,
	double &pow_dec_,
	Graph Graph_);

void locationPrediction(
	int location_num_,
	point_t pos_,
	point_t vel_,
	Graph Graph_,
	double surface_limit_,
	double angle_limit_,
	double vel_limit_,
	label_t &LABEL_);


void predictionEdge(
	pred_t &prediction_,
	Graph &Graph_,
	point_t pos_,
	point_t vel_,
	int label1_,
	vector<int> &last_loc_,
	limit_t limit,
	label_t &label_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	double &pow_dec_);

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
	bool learn_);

// ============================================================================
// EXTRAS
// ============================================================================

void outputMsg(
	msg_t MSG_,
	Graph Graph_);



















void writePointFile(
	point_t *p,
	unsigned int num_points);

int checkMoveSlide(
	point_t pos_,
	point_t vel_,
	vector<double> surface_,
	double surface_limit_,
	double angle_limit_);

double checkMoveSlideOutside(
	point_t pos_,
	point_t vel_,
	double **surface_,
	unsigned int num_surfaces_);

bool checkSurfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_,
	double surface_limit_,
	double surface_range_limit_);

double surfaceDistance(
	point_t pos_,
	vector<double> surface_);

double surfaceAngle(
	point_t vel_,
	vector<double> surface_);

double surfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_);








#endif /* UTIL_H_ */