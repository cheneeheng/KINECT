/*
 * NBayesClassifier.cpp
 *
 *  Created on: Jul 14, 2017
 *      Author: chen
 */

#include "NBayesClassifier.h"

NBayesClassifier::NBayesClassifier()
		: OP(new ObjectPrediction),
				P1_container_all(new std::vector<double>)
{

}

NBayesClassifier::~NBayesClassifier()
{
}

void NBayesClassifier::Prior(
		std::shared_ptr<CData> cdata_,
		const int &label1_)
{
	P1_container_all->clear();

	double sum_tmp = 0.0;
	std::vector<double> P1_container;

	auto g = cdata_->G;
	auto os = cdata_->OS;
	auto kb = cdata_->KB;
	auto ac = kb->AC()["GEOMETRIC"];

	for (int ii = 0; ii < os->OSLabelList().size(); ii++)
	{
		P1_container.clear();
		sum_tmp = 0.0;

		double obj_trans = OP->GetOSTransition(*(os), g->GetObject())[ii];

		for (int i = 0; i < g->GetNumberOfNodes(); i++)
		{
			int la1, la0, c = 0;

			for (int l = ac.first; l < ac.second + 1; l++)
			{
				if (kb->AL()[l] == g->GetNode(label1_).name)
				{
					la0 = l;
					c++;
				}
				if (kb->AL()[l] == g->GetNode(i).name)
				{
					la1 = l;
					c++;
				}
				if (c > 1)
				{
					break;
				}
			}

			auto P_OS_LA = OP->GetLAObjectTransition(*(os), g->GetObject(),
					la1 - ac.first)[ii];
			auto P_LA_LA = kb->TransitionLA()[g->GetObject()][la0][la1];

			P1_container.push_back(P_OS_LA * P_LA_LA);
			sum_tmp += P1_container.back();
			P1_container.back() *= obj_trans;
		}

		if (!P1_container_all->empty())
		{
			if (sum_tmp > 0.0)
			{
				for (int i = 0; i < P1_container.size(); i++)
				{
					P1_container[i] /= sum_tmp;
					(*P1_container_all)[i] += P1_container[i];
				}
			}
		}
		else
		{
			if (sum_tmp > 0.0)
			{
				for (auto &i : P1_container)
				{
					i /= sum_tmp;
				}
			}
			*P1_container_all = P1_container;
		}
	}
}

void NBayesClassifier::Classify(
		std::vector<double> &prediction_)
{
	double sum_tmp = 0.0;
	for (int i = 0; i < P1_container_all->size(); i++)
	{
		prediction_[i] *= (*P1_container_all)[i];
		sum_tmp += prediction_[i];
	}

	for (auto &prediction_i : prediction_)
	{
		prediction_i /= sum_tmp;
	}
}
