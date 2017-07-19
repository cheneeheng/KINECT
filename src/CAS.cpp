/*******************************************************************************
 * CAS.cpp
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: ActionState container.
 ******************************************************************************/

#include "CAS.h"

CAS::CAS()
		: grasp(GRAB::RELEASE),
				label1(-1),
				label2(-1),
				obj(-1),
				vel(0.0),
				pct_err(-1),
				surface_flag(0),
				surface_name(""),
				surface_dist(0.0)
{
}

CAS::~CAS()
{
}
