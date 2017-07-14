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
	DataFilter();
	virtual ~DataFilter();

	void ResetFilter();
	int PreprocessDataLive(
			const Eigen::Vector4d &pos_,
			std::vector<Eigen::Vector4d> &pos_vel_acc_avg_, //motion
			const int &window_);
	int PreprocessContactLive(
			const int &contact_,
			int &contact_out_,
			const int &window_);

private:
	std::vector<std::vector<Eigen::Vector4d> > 	pos_vel_acc_mem;
	std::vector<float> contact_mem;
};

#endif /* DATAFILTER_H_ */
