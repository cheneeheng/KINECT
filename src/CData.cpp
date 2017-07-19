/*
 * CData.cpp
 *
 *  Created on: May 20, 2017
 *      Author: chen
 */

#include "CData.h"

CData::CData()
		: G(new CGraph),
				KB(new CKB),
				AS(new CAS),
				OS(new COS),
				msg(new std::vector<std::string>),
				contact(new int),
				pva(new std::vector<Eigen::Vector4d>)
{
	pva->clear();
	pva->resize(3);
}

CData::CData(
		const std::string &object_,
		const int &loc_int_,
		const int &sec_int_)
		: G(new CGraph(object_, loc_int_, sec_int_)),
				KB(new CKB),
				AS(new CAS),
				OS(new COS),
				msg(new std::vector<std::string>),
				contact(new int),
				pva(new std::vector<Eigen::Vector4d>)
{
	pva->clear();
	pva->resize(3);
}
