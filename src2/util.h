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

void writePointFile(
	point_t *p,
	unsigned int num_points);

bool checkMoveSlide(
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

void writeSectorFile(
	Graph Graph_,
	string path_,
	int maxminconst_);

void writeSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > sector_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_);

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

void readSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > &sector_);

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
// Sector
// ============================================================================

void generateSector(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	vector<int>file_eof_,
	vector<vector<double> > kernel_);

void prepareSector(
	Graph &Graph_);

void updateSector(
	vector<sector_t> &sector_,
	point_t pos_,
	vector<point_t> locations_,
	sector_para_t sector_para_,
	int tmp_id1_,
	int tmp_id2_,
	vector<vector<double> > kernel_);

void checkSectorConstraint(
	Graph &Graph_,
	double max_range_,
	int kernel_size_x_,
	int kernel_size_y_);

// ============================================================================
// Prediction
// ============================================================================

void triggerContact(
	point_t &p_,
	Graph Graph_);

void checkMotion(
	point_t pos_,
	point_t vel_,
	vector<string> label_,
	vector<vector<double> > surface_,
	double surface_limit_,
	double angle_limit_,
	double vel_limit_,
	label_t &LABEL_);

void checkSector(
	vector<int> &prediction_,
	vector<double> &t_val_,
	point_t pos_,
	Graph &Graph_,
	Graph Graph_mem_,
	int tmp_id_,
	bool learn_);

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





















#endif /* UTIL_H_ */
