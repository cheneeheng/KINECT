/*******************************************************************************
 * Evaluate.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: chen
 *      Detail: Evaluate and update the action state.
 ******************************************************************************/

#include "Evaluate.h"

Evaluate::Evaluate()
		: label1_eval(-1),
				vel_eval(0.0),
				surface_dist_eval(0.0)
{
}

Evaluate::~Evaluate()
{
}

int Evaluate::UpdateStateNode(
		const CGraph &G_,
		CAS &AS_)
{
	AS_.Velocity(vel_eval);
	AS_.SurfaceDistance(surface_dist_eval);
	return EXIT_SUCCESS;
}

int Evaluate::UpdateStateEdge(
		const CGraph &G_,
		CAS &AS_)
{
	auto minmax = std::minmax_element(pct_eval.begin(), pct_eval.end());

	if (*minmax.second <= 0)
	{
		AS_.Probability(0.0);
		AS_.SurfaceFlag(0);
		AS_.SurfaceName("");
	}
	else
	{
		AS_.Probability(*minmax.second);
		auto max_idx = minmax.second - pct_eval.begin();

		// Saving the LA as integers instead of string.
		int c = 0, cc = 0;
		for (auto i : al_eval)
		{
			if (G_.GetNode(label1_eval).name == i)
			{
				AS_.Label1(cc);
				c++;
			}
			if (G_.GetNode(max_idx).name == i)
			{
				AS_.Label2(cc);
				c++;
			}
			if (c == 2)
				break;
			cc++;
		}

		AS_.SurfaceFlag(G_.GetNode(max_idx).surface_flag);
		AS_.SurfaceName(G_.GetNode(max_idx).name);
	}

	AS_.Velocity(vel_eval);

	int c = 0;
	auto g = AS_.Goal();
	auto w = AS_.Window();
	for (auto i : G_.GetNodeList())
	{
		g[i.name] = pct_eval[c];
		w[i.name] = win_eval[c];
		c++;
	}
	AS_.Goal(g);
	AS_.Window(w);
	return EXIT_SUCCESS;
}
