/*
 * vtkExtra.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef VTKEXTRA_H_
#define VTKEXTRA_H_

#include "dataDeclaration.h"
#include "Graph.h"
#include "algo.h"

// For backward compatibility with new VTK generic data arrays
#define InsertNextTypedTuple InsertNextTupleValue

//=============================================================================

void colorCode(
	vector<unsigned char*> &container_);

void showData(
	vector<point_t> points_,
	vector<string> &labels_,
	vector<unsigned char*> color_,
	bool cluster_,
	bool labeling_);

void showConnectionOnly(
	Graph Graph_,
	vector<unsigned char*> color_);

void showConnection(
	vector<point_t> points_,
	vector<string> &labels_,
	Graph Graph_,
	vector<unsigned char*> color_,
	bool show_points);

void plotData(
	vector<double> x_,
	vector<double> y_);

vtkSmartPointer<vtkPolyDataMapper> dataPoints(
	vector<point_t> points_,
	vector<unsigned char*> color_,
	bool cluster_);

#endif /* VTKEXTRA_H_ */
