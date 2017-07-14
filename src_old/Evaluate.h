/*******************************************************************************
 * Evaluate.h
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 *      Detail: Evaluate and update the action state.
 ******************************************************************************/

#ifndef EVALUATE_H_
#define EVALUATE_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "CGraph.h"
#include "CAS.h"

class Evaluate
{
	public:
		Evaluate();
		virtual ~Evaluate();

		/*
		 * Update the action state at the node.
		 */
		virtual int UpdateStateNode(
				const CGraph &G_,
				CAS &AS_);

		/*
		 * Update the action state at the edge.
		 */
		virtual int UpdateStateEdge(
				const CGraph &G_,
				CAS &AS_);

	protected:
		int label1_eval;
		double vel_eval;
		double surface_dist_eval;
		std::vector<double> win_eval;
		std::vector<double> pct_eval;
		std::vector<double> pct_eval_mem;
		std::vector<std::string> al_eval;
		std::map<std::string, std::pair<int, int> > ac_eval;
};

#endif /* EVALUATE_H_ */
