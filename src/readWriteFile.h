/*
 * readWriteFile.h
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#ifndef READWRITEFILE_H_
#define READWRITEFILE_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"

void writeSurfaceFile(
	Graph Graph_);

void writeLabelFile(
	Graph Graph_,
	string path_,
	int movloc_);

void writeLearnedDataFile(
	Graph Graph_,
	string path_,
	int type_);

void writeMovLabelFile(
	string path_,
	vector<string> label_);

void writeLocLabelFile(
	Graph Graph_,
	string path_);

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
	vector<vector<string> > data_,
	vector<string> &LABEL_LOC_,
	vector<point_t> &locations_,
	vector<double> &location_boundary_,
	vector<int> &surface_num_);

void readLocation_(
	Graph &Graph_,
	vector<vector<string> > data_);

void readMovement(
	Graph &Graph_,
	vector<vector<string> > data_);

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

bool copyFile(
	string SRC,
	string DEST);

bool directoryCheck(
	string path_ );


#endif /* READWRITEFILE_H_ */
