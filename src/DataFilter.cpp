/*
 * DataFilter.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 *      Detail: Moving average filter is applied on the data.
 */

#include "DataFilter.h"

DataFilter::DataFilter()
{
}

DataFilter::~DataFilter()
{
}

void DataFilter::ResetFilter()
{
	reshapeVector(pos_vel_acc_mem, 3); //pva
	contact_mem.clear();
}

int DataFilter::PreprocessDataLive(
		const Eigen::Vector4d &pos_,
		std::vector<Eigen::Vector4d> &pva_avg_, //motion
		const int &window_)
{
	auto vel = pos_ - pva_avg_[0];
	auto acc = vel - pva_avg_[1];
	std::vector<Eigen::Vector4d> tmp
	{ pos_, vel, acc };

	for (int i = 0; i < 3; i++)
	{
		// 1. Memory is same length as window.
		if (pos_vel_acc_mem[i].size() == window_)
		{
			// To prevent outliers.
			if (V4d3d(vel).norm() < 0.3)
			{
				pva_avg_[i] = movingAverage(tmp[i], pos_vel_acc_mem[i]);
			}
			else
			{
				if (i == 0)
				{
					auto new_pva_avg_from_last = pva_avg_[0] + pva_avg_[1];
					pva_avg_[i] = movingAverage(new_pva_avg_from_last,
							pos_vel_acc_mem[i]);
				}
			}
		}
		// 2. Memory is not empty but less than window.
		else if (pos_vel_acc_mem[i].size() > 0)
		{
			pva_avg_[i] = movingAverageIncrement(tmp[i], pos_vel_acc_mem[i]);
		}
		// 3. Memory is empty.
		else
		{
			pos_vel_acc_mem[i].push_back(tmp[i]);
			pva_avg_[i] = tmp[i];
			pva_avg_[i][3] = -1;
			for (int ii = i + 1; ii < 3; ii++)
			{
				pva_avg_[ii][0] = pva_avg_[ii][1] = pva_avg_[ii][2] = 0.0;
				pva_avg_[ii][3] = -1;
			}
			break;
		}
		pva_avg_[i][3] = -1;
	}
	return EXIT_SUCCESS;
}

int DataFilter::PreprocessContactLive(
		const int &contact_,
		int &contact_out_,
		const int &window_)
{
	// 1. Memory is same length as window.
	if (contact_mem.size() == window_)
	{
		contact_out_ = round(movingAverage((float) contact_, contact_mem));
	}
	// 2. Memory is not empty but less than window.
	else if (contact_mem.size() > 0)
	{
		contact_out_ = round(
				movingAverageIncrement((float) contact_, contact_mem));
	}
	// 3. Memory is empty.
	else
	{
		contact_mem.push_back((float) contact_out_);
	}
	return EXIT_SUCCESS;
}
