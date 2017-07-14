/*
 * ActionParser.h
 *
 *  Created on: May 21, 2017
 *      Author: chen
 *      Detail: Parse the action state using a decision tree.
 */

#ifndef ACTIONPARSER_H_
#define ACTIONPARSER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string>
#include <vector>
#include "CGraph.h"
#include "CKB.h"
#include "CAS.h"

class ActionParser
{
public:
	ActionParser(
			const std::string &obj_,
			std::shared_ptr<CKB> KB_,
			std::shared_ptr<std::vector<std::string> > msg_,
			const std::string &path_);
	virtual ~ActionParser();

	virtual int ReadMsg(
			std::string path_);
	virtual void SetMsg(
			std::shared_ptr<std::vector<std::string> > msg_)
	{
		message = msg_;
	}

	virtual void Init(
			int delay_);
	virtual std::string Decode(
			std::string obj_,
			std::string loc_,
			std::string msg_);

	virtual void Delay_(
			CAS *s_);
	virtual void Parse(
			CAS *s_);
	virtual void DT0();
	virtual void DT1();
	virtual void DT2_1();
	virtual void DT2_2();
	virtual void DT3_1();
	virtual void DT3_2();
	virtual void DT4_1(
			double x_);
	virtual void DT4_2(
			std::vector<double> x_);
	virtual void Display(
			const std::string &filename_);

private:
	std::shared_ptr<CKB> KB;
	std::shared_ptr<std::vector<std::string> > message;
	/*
	 std::map<std::string,std::pair<int,int> > ac;
	 std::vector<std::string> al;
	 std::map<std::string,std::map<std::string,std::string> > ol;
	 */

	std::vector<int> message_num;

	std::string obj;
	std::string loc;
	int delay_factor;
	std::vector<CAS> state_mem;
	std::string output;
	std::string output_mem;
	std::string label;
	std::string label_mem;
	std::string repeat;
	bool grasp_flag;
	bool start_loc;

	std::string o;
	std::string l;
	std::string a;

	std::string ob_ac;

	std::string msg_path;

	double addFunction(
			double x,
			double y)
	{
		return fabs(x) + fabs(y);
	}

};

#endif /* ACTIONPARSER_H_ */
