/*
 * ObjectPrediction.h
 *
 *  Created on: May 22, 2017
 *      Author: chen
 */

#ifndef OBJECTPREDICTION_H_
#define OBJECTPREDICTION_H_

#include "COS.h"

class ObjectPrediction
{
public:
	ObjectPrediction();
	virtual ~ObjectPrediction();

	/*
	 * P(O_t|O_t-1)
	 */
	virtual std::vector<double> GetOSTransition(
			const COS &OS_,
			const std::string &obj_)
	{
		return OS_.TransitionOS()[obj_][OS_.OSLabel()];
	}

	/*
	 * P(LA_t|O_t)
	 */
	virtual std::vector<double> GetObjectLATransition(
			const COS &OS_,
			const std::string &obj_,
			const int &os_)
	{
		return OS_.TransitionOSLA()[obj_][os_];
	}

	/*
	 * P(O_t|LA_t)
	 */
	virtual std::vector<double> GetLAObjectTransition(
			const COS &OS_,
			const std::string &obj_,
			const int &la_)
	{
		return OS_.TransitionLAOS()[obj_][la_];
	}

	/*
	 * Updates the current object state.
	 */
	virtual int UpdateOS(
			COS &OS_,
			const std::string &obj_,
			const std::string &la_)
	{
		OS_.OSLabel(OS_.LAOSMap()[obj_][la_]);
		return EXIT_SUCCESS;
	}

};

#endif /* OBJECTPREDICTION_H_ */
