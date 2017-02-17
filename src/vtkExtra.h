/*
 * vtkExtra.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef VTKEXTRA_H_
#define VTKEXTRA_H_

#include "dataDeclaration.h"

//=============================================================================

void colorCode(vector<unsigned char*> &container);

void showData(
	vector<point_t> p,
	vector<string> &label,
	vector<unsigned char*> color_,
	bool cluster,
	bool labeling);

void showConnection(
	vector<vector<vector<sector_t> > > 	sector,
	vector<vector<vector<double> > > sector_constraint,
	vector<point_t> loc_loc_vec,
	vector<point_t> loc_loc_normal,
	vector<double>  loc_loc_norm,
	vector<point_t> location,
	int num_location_intervals,
	int num_sector_intervals,
	vector<unsigned char*> color_);


#endif /* VTKEXTRA_H_ */
