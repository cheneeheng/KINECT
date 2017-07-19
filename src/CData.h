/*******************************************************************************
 * CData.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: Container for input data.
 ******************************************************************************/

#ifndef CONTAINERDATA_H_
#define CONTAINERDATA_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <memory>
#include "CGraph.h"
#include "CKB.h"
#include "CAS.h"
#include "COS.h"

#include <Eigen/Eigen>

class CData
{

public:

	/**
	 * Constructor of class CData.
	 */
	CData();

	/**
	 * Constructor of class CData.
	 * @param object_ Object name for CGraph.
	 * @param loc_int_ Trajectory interval parameter.
	 * @param sec_int_ Sector interval parameter.
	 */
	CData(
			const std::string &object_,
			const int &loc_int_,
			const int &sec_int_);

	/**
	 * Destructor of class CData.
	 */
	virtual ~CData()
	{
	}
	;

	/* Graph class */
	std::shared_ptr<CGraph> G;

	/* Knowledge-base class */
	std::shared_ptr<CKB> KB;

	/* Action State class */
	std::shared_ptr<CAS> AS;

	/* Object State class */
	std::shared_ptr<COS> OS;

	/* Dictionary of messages to be parsed */
	std::shared_ptr<std::vector<std::string> > msg;

	/* Position-Velocity-Acceleration */
	std::shared_ptr<std::vector<Eigen::Vector4d> > pva;

	/* Contact */
	std::shared_ptr<int> contact;

};

#endif /* CONTAINERDATA_H_ */
