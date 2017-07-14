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
	CData(
			const std::string &object_,
			const int &loc_int_,
			const int &sec_int_,
			std::shared_ptr<CKB> KB_,
			std::shared_ptr<COS> OS_);

	virtual ~CData();

	/* Graph */
	std::shared_ptr<CGraph> G;

	/* Knowledge-base */
	std::shared_ptr<CKB> KB;

	/* Action State */
	std::shared_ptr<CAS> AS;

	/* Object State */
	std::shared_ptr<COS> OS;

	/* position-velocity-acceleration */
	std::shared_ptr<std::vector<Eigen::Vector4d> > pva;

	/* Contact */
	std::shared_ptr<int> contact;
};

#endif /* CONTAINERDATA_H_ */
