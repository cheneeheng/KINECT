/*
 * Test.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 *      Detail: Online evaluation.
 */

#ifndef TEST_H_
#define TEST_H_

#include "CData.h"
#include "VTKExtra.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "DataParser.h"
#include "DataFilter.h"
#include "ActionParser.h"
#include "ActionPrediction.h"

/**
 * Carries out online action prediction.
 */
class Test: public DataParser
{
public:

	/**
	 * Constructor for Test class.
	 * @param obj_ Object name.
	 * @param loc_int_ Number of location interval.
	 * @param sec_int_ Number of sector interval.
	 * @param f_win Filter window.
	 * @param cdata_ Data container.
	 * @param path_ Path to save the parsed action.
	 * @param object_state_ Flag to use Bayes classifier.
	 */
	Test(
			const std::string &obj_,
			const int &loc_int_,
			const int &sec_int_,
			const int &f_win_,
			std::shared_ptr<CData> cdata_,
			const std::string &path_,
			bool object_state_);

	/**
	 * Destructor for Test class.
	 */
	virtual ~Test();

	/**
	 * Writes and saves the window/radius of SM.
	 *
	 * @param path_ Path to the dst file.
	 */
	virtual int WriteWindow(
			const std::string &path_);

	/**
	 * Applies gaussian filter to the SM.
	 *
	 * @param num_x_ Dim 1 of the kernel.
	 * @param num_y_ Dim 2 of the kernel.
	 */
	virtual int ApplyGauss(
			const int &num_x_,
			const int &num_y_);

	/**
	 * Initializes the class.
	 */
	virtual int TestInit();

	/**
	 * Adjusts the a known LA.
	 *
	 * @param LA_name_ Name of the LA to be changed.
	 * @param LA_new Data for the new LA. 3D point + radius.
	 */
	virtual int LAAdjust(
			const std::string &LA_name_,
			Eigen::Vector4d LA_new);

	/**
	 * Writes the output of the evaluation to a file.
	 *
	 * @param filename_ Name of the file to be written.
	 * @param resultdir_ Name of the directory to be written.
	 * @param labels_predict_ Predicted labels.
	 * @param data_writeout_ List of data values to be written.
	 * @param goals_ List of probabilities for possible goals.
	 * @param windows_ List of variance for each SM involved.
	 * @param nolabel_ Flag. TRUE = dataset is labeled.
	 */
	virtual int WriteResult(
			const std::string &filename_,
			const std::string &resultdir_,
			bool nolabel_);

	/**
	 * Filters the data using the class DataFilter.
	 *
	 * @param pva_in_ Position p .
	 * @param pva_out_ Motion vector [p v a]' .
	 * @param contact_in_ Raw data.
	 * @param contact_out_ Filtered data.
	 */
	virtual int FilterData(
			const Eigen::Vector4d &pva_in_,
			std::vector<Eigen::Vector4d> &pva_out_,
			const int &contact_in_,
			int &contact_out_);

	/**
	 * Carries out prediction using the class ActionPrediction.
	 */
	virtual int Predict();

	/**
	 * Carries out output data parsing using the class ActionParser.
	 *
	 * @param filename_ Name of file to write the parsed messages.
	 * @param timestamp_ Timestamp.
	 * @param display_ Displayed output message.
	 */
	virtual int Parser(
			const std::string &filename_,
			const int &timestamp_,
			std::string &display_);


	virtual int GetData(const int &counter);

	/**
	 * Main evaluation function.
	 *
	 * @param filename_ Name of the file to be written.
	 * @param resultdir_ Name of the directory to be written.
	 * @param face_ Flag. TRUE = LA face will be modified.
	 */
	virtual int Testing(
			const std::string &filename_,
			const std::string &resultdir_,
			bool face_);

private:
	std::shared_ptr<DataFilter> DF;
	std::shared_ptr<ActionPrediction> APred;
	std::shared_ptr<ActionParser> AParse;
	std::shared_ptr<ReadFile> RF;
	std::shared_ptr<WriteFile> WF;
	int loc_int;
	int sec_int;
	int f_win;

	std::vector<std::vector<double> > data_writeout;
	std::vector<std::string> labels_predict;
	std::vector<std::vector<Eigen::Vector4d> >	pvas;
	std::vector<std::map<std::string,double> > goals;
	std::vector<std::map<std::string,double> > windows;

	std::shared_ptr<CData> cdata;
};

#endif /* TEST_H_ */
