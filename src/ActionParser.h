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
#include "CData.h"

/**
 * Parses action state into human understandable sentences.
 */
class ActionParser
{
public:

	/**
	 * Constructor for ActionParser class.
	 *
	 * @param obj_ Object name.
	 * @param cdata_ Data container.
	 */
	ActionParser(
			const std::string &obj_,
			std::shared_ptr<CData> cdata_,
			const std::string &path_);

	/**
	 * Destructor for ActionParser class.
	 */
	virtual ~ActionParser();

	/**
	 * Reads a list or predefined sentence dictionary.
	 *
	 * @param path_ Dictionary file name.
	 */
	virtual int ReadMsg(
			std::string path_);

	/**
	 * Overrides the sentence dictionary.
	 *
	 * @param msg_ List of sentences.
	 */
	virtual void SetMsg(
			std::shared_ptr<std::vector<std::string> > msg_)
	{
		message = msg_;
	}

	/**
	 * Initialization.
	 *
	 * @param delay_ Delay factor.
	 */
	virtual void Init(
			int delay_);

	/**
	 * Decodes the sentences using relevant informations.
	 * Parser uses generic phrases during parsing.
	 *
	 * @param obj_ Object name.
	 * @param loc_ LA name.
	 * @param msg_ Sentence.
	 */
	virtual std::string Decode(
			std::string obj_,
			std::string loc_,
			std::string msg_);

	/**
	 * Delay in action parsing. Acts as a filter.
	 *
	 * @param s_ Action state.
	 */
	virtual void Delay_(
			CAS *s_);

	/**
	 * Parses action state.
	 *
	 * @param s_ Action state.
	 */
	virtual void Parse(
			CAS *s_);

	/**
	 * Decision tree.
	 *
	 * 0. Grasp
	 * - no: end
	 * - yes: 1.
	 *
	 * 1. Position (pct_err)
	 * - -1: 2.1. (in LA)
	 * - 0: end
	 * - xx: 2.2. (in SM)
	 *
	 * 2.1. LA
	 * - start?
	 * -- yes: end
	 * -- no: surface?
	 * --- yes: 3.1
	 * --- no: end
	 * 2.2. SM
	 * - yes: 3.2
	 * - no: end
	 *
	 * 3.1. surface constraint
	 * - vel>0 ?
	 * -- yes: end
	 * -- no: end
	 * 3.2 trajectory number
	 * - just 1: 4.1.
	 * - multiple: 4.2.
	 *
	 * 4.1. 1 trajectory
	 * - pct_err > x% ?
	 * --yes: end
	 * --no: end
	 * 4.2. multiple trajectory
	 * - ratio of max > x% ?
	 * -- yes: end
	 * -- no: ratio of next max - end
	 */
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

	/**
	 * Displays the parsed sentence.
	 *
	 * @param filename_ Filename to save the parsed message.
	 */
	virtual void Display(
			const std::string &filename_,
			const int &timestamp_,
			std::string &display_);

private:

	// Knowledge-base
	std::shared_ptr<CKB> KB;

	// List of messages
	std::shared_ptr<std::vector<std::string> > message;

	// Message pair. To check if the same message was parsed twice.
	std::pair<int, int> message_num;

	// Object name
	std::string obj;

	// Location name
	std::string loc;

	// Delay factor
	int delay_factor;

	// Action state memory for averaging
	std::vector<CAS> state_mem;

	// Output to display
	std::string output;

	// Output memory for averaging
	std::string output_mem;

	// Action label
	std::string label;

	// Action label memory to check if it has changed
	std::string label_mem;

	// Grasp state 1/0
	bool grasp_flag;

	// Object, Label, Action for decoding.
	std::string o, l, a;

	// Object action pair
	std::string ob_ac;

	// Path to save the parsed sentences
	std::string msg_path;

	/**
	 * Internal add function for accumulate.
	 */
	double addFunction(
			double x,
			double y)
	{
		return fabs(x) + fabs(y);
	}

};

#endif /* ACTIONPARSER_H_ */
