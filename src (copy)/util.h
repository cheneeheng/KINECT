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

// data file ==================================================================
void readFile(
	char *name,
	vector<vector<string> > &data_full,
	char delimiter);

void rewriteFileLoc(
	char *name_,
	int line_,
	vector<string> new_,
	char delimiter_);

void rewriteFileObj(
	char *name_,
	int line_,
	vector<string> new_,
	char delimiter_);

void writeLocLabelFile(
	vector<string> label_,
	unsigned int obj_,
	vector<point_t> locations_);

void writeObjLabelFile(
	vector<string> label_,
	unsigned int obj_);

void parseData2Point(
	vector<vector<string> > data_full,
	vector<point_t> &points);

void preprocessData(
	point_t *p1,
	point_t **pos_vel_acc,
	unsigned int num_points,
	unsigned int *file_eof,
	unsigned int window);

void preprocessDataLive(
	point_t pos,
	vector< vector< vector<double> > > &pos_vel_acc_mem, // motion,xyz,length
	vector<point_t> &pos_vel_acc_avg,
	unsigned int window);

// dbscan =====================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_t *p);

void combineNearCluster(
	vector<point_t> &points,
	vector<point_t> &locations);

void contactBoundary(
	vector<point_t> &p,
	vector<point_t> locations,
	vector<double> &location_boundary,
	bool learn);

void decideBoundary(
	point_t &p,
	vector<point_t> location,
	vector<double> location_boundary);

// SVM ========================================================================

void classifierFeature(
	data_t motionData,
	point_t location,
	double &location_distance,
	double &location_angle);

void classifierSVM(
	vector<data_t> motionData,
	int *label,
	unsigned int num_points,
	point_t *location,
	unsigned int num_locations,
	bool train=true);

// Sector =====================================================================

void prepareSector(
	vector<point_t> &tmp_dir,
	vector<point_t> &tmp_dir_normal,
	vector<double> &tmp_norm,
	vector<point_t> location);

void updateSector(
	vector<vector<vector<sector_t> > > 	&sector_,
	point_t pos_,
	vector<point_t> locations_,
	vector<point_t> tmp_dir_,
	vector<point_t> tmp_dir_normal_,
	vector<double> tmp_norm_,
	int num_location_intervals_,
	int num_sector_intervals_,
	int tmp_id1_,
	int tmp_id2_,
	vector<vector<double> > kernel_);

void checkSector(
	vector<double> &prediction,
	vector<double> &t_val,
	vector<vector<vector<sector_t> > > 	&sector,
	point_t pos_,
	vector<point_t> location,
	vector<point_t> tmp_dir,
	vector<point_t> tmp_dir_normal,
	vector<double> tmp_norm,
	int num_location_intervals,
	int num_sector_intervals,
	int tmp_id,
	bool learn=false);

void generateSector(
	vector<vector<vector<sector_t> > > 	&sector_,
	vector<point_t> pos_,
	vector<point_t> locations_,
	vector<point_t> tmp_dir_,
	vector<point_t> tmp_dir_normal_,
	vector<double> tmp_norm_,
	int num_location_intervals_,
	int num_sector_intervals_,
	vector<int>file_eof_,
	vector<vector<double> > kernel_);

void checkSectorConstraint(
	vector<vector<vector<sector_t> > > 	sector,
	vector<vector<vector<double> > > &sector_constraint,
	int num_locations,
	int num_location_intervals,
	int num_sector_intervals);







void labelingMovement(
	vector<string> &LABEL_MOV,
	int num_mov,
	int obj,
	int num_objs);

void labelingLocation(
	vector<point_t> &points,
	vector<point_t> &locations,
	vector<double> &location_boundary,
	vector<string> &LABEL_LOC,
	int obj,
	double epsilon,
	int minpts);


#endif /* UTIL_H_ */
