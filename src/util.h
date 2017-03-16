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
#include "util.h"
#include "dbscan.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "readWriteFile.h"
#include "labeling.h"
#include "prediction.h"

// ============================================================================
// Modules
// ============================================================================

int learnLocationArea(
	string dirname_,
	string scene,
	string object);

int learnSector(
	string dirname_,
	string scene,
	string object);

int testing(
	string dirname_,
	string scene,
	string object);

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









#endif /* UTIL_H_ */
