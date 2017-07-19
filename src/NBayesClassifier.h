/*
 * NBayesClassifier.h
 *
 *  Created on: Jul 14, 2017
 *      Author: chen
 */

#ifndef NBAYESCLASSIFIER_H_
#define NBAYESCLASSIFIER_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <memory>
#include "CData.h"
#include "ObjectPrediction.h"

class NBayesClassifier
{
public:
	/**
	 * Constructor for NBayesClassifier class.
	 */
	NBayesClassifier();

	/**
	 * Destructor for Evaluate class.
	 */
	virtual ~NBayesClassifier();

	/*
	 * Calculates the prior of the naive bayes classifier.
	 *
	 * @param cdata_ Data container.
	 * @param label1_ Starting LA/node
	 */
	void Prior(
			std::shared_ptr<CData> cdata_,
			const int &label1_);

	/*
	 * Calculates the full naive bayes classifier.
	 *
	 * @param prediction_ Likelihood
	 */
	void Classify(
			std::vector<double> &prediction_);

private:
	std::shared_ptr<ObjectPrediction> OP;
	std::shared_ptr<std::vector<double> > P1_container_all;

};

#endif /* NBAYESCLASSIFIER_H_ */
