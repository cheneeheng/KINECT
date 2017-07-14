/*
 * Train.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
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

class Train :  public DataParser, public TrainLA, public TrainSM
{
public:
	Train(
			const int &loc_int_,
			const int &sec_int_,
			const int &f_win_);
	virtual ~Train();

	/*
	 * Main learning function.
	 *
	 * Input
	 * - new_label_ : 	to check if a new label is seen in the data,
	 * 					needed for evaluation only
	 * - face_ 		:	a flag to use the moving LA transformation.
	 */
	int Learning(
			std::shared_ptr<CGraph> G_,
			std::shared_ptr<CKB> KB_,
			const std::string &filename_,
			const std::string &path_LA_,
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
