/*
 * Deployment.h
 *
 *  Created on: May 8, 2017
 *      Author: chen
 *      Detail: This class can be deployed for online evaluation.
 */

#ifndef DEPLOYMENT_H_
#define DEPLOYMENT_H_

#include "ActionPrediction.h"
#include "DataFilter.h"

#define FILTER_WIN 5

class Deployment: public ActionPrediction, public DataFilter
{
public:

	Deployment();
	virtual ~Deployment();

	int Deploy(
			Eigen::Vector4d point_,
			int contact_);
};

#endif /* DEPLOYMENT_H_ */
