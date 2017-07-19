/*
 * DataFilter.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 *      Detail: Moving average filter is applied on the data.
 */

#ifndef DATAFILTER_H_
#define DATAFILTER_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "algo.h"

#include <Eigen/Eigen>

class DataFilter
{
public:

	/**
	 * Constructor for DataFilter class.
	 */
	DataFilter();

	/**
	 * Destructor for DataFilter class.
	 */
	virtual ~DataFilter();

	/**
	 * Resets the filter.
	 */
	void ResetFilter();

	/**
	 * Filters the motion data.
	 *
	 * @param pos_ Trajectory point.
	 * @param pos_vel_acc_avg_ Motion data.
	 * @param window_ Filter window.
	 */
	int PreprocessDataLive(
			const Eigen::Vector4d &pos_,
			std::vector<Eigen::Vector4d> &pos_vel_acc_avg_, //motion
			const int &window_);

	/**
	 * Filters the contact.
	 *
	 * @param contact_ Contact flag.
	 * @param contact_out_ Filtered contact flag.
	 * @param window_ Filter window.
	 */
	int PreprocessContactLive(
			const int &contact_,
			int &contact_out_,
			const int &window_);

private:

	/**
	 * Memory of motion data and contact flag used for averaging.
	 */
	std::vector<std::vector<Eigen::Vector4d> > pos_vel_acc_mem;
	std::vector<float> contact_mem;
};

#endif /* DATAFILTER_H_ */
