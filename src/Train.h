/*
 * Train.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 *      Detail: Builds the LA and SM.
 */

#ifndef TRAIN_H_
#define TRAIN_H_

#include "CData.h"
#include "TrainLA.h"
#include "TrainSM.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "DataParser.h"
#include "DataFilter.h"

#include <Eigen/Eigen>

/**
 * Evaluates and builds the SM and LA.
 */
class Train: public DataParser, public TrainLA, public TrainSM
{
public:

	/**
	 * Constructor for class Train.
	 *
	 * @param loc_int_ Number of location interval.
	 * @param sec_int_ Number of sector interval.
	 * @param f_win_ Filter window.
	 */
	Train(
			const int &loc_int_,
			const int &sec_int_,
			const int &f_win_);

	/**
	 * Destructor for class Train.
	 */
	virtual ~Train();

	/**
	 * Adjusts the a known LA.
	 *
	 * @param LA_name_ Name of the LA to be changed.
	 * @param G_ Scene graph.
	 * @param LA_new Data for the new LA. 3D point + radius.
	 */
	int LAAdjust(
			const std::string &LA_name_,
			std::shared_ptr<CGraph> G_,
			Eigen::Vector4d LA_new);

	/*
	 * Main learning function. Builds the SM and LA.
	 *
	 * @param G_ Graph of the scene.
	 * @param KB_ List of knowledge-base.
	 * @param filename_ Name of dataset file.
	 * @param new_label_ Flag. TRUE = new label is present.
	 * @param face_ Flag. TRUE = LA face is modified.
	 */
	int Learning(
			std::shared_ptr<CGraph> G_,
			std::shared_ptr<CKB> KB_,
			const std::string &filename_,
			bool new_label_,
			bool face_);

private:
	std::shared_ptr<DataFilter> DF;
	std::shared_ptr<ReadFile> RF;
	std::shared_ptr<WriteFile> WF;

	int loc_int;
	int sec_int;
	int f_win;
};

#endif /* TRAIN_H_ */
