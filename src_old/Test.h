/*
 * Test.h
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#ifndef TEST_H_
#define TEST_H_

#include "VTKExtra.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "DataParser.h"
#include "DataFilter.h"
#include "ActionParser.h"
#include "ActionPrediction.h"

class Test : public DataParser
{
public:
	Test(
			const std::string &obj_,
			const int &loc_int_,
			const int &sec_int_,
			const int &f_win_,
			std::shared_ptr<CKB> KB_,
			std::shared_ptr<COS> OS_,
			std::shared_ptr<std::vector<std::string> > msg_,
			const std::string &path_,
			bool object_state_);
	virtual ~Test();

	virtual int ReadKB(const std::string &path_);
	virtual int ReadLA(const std::string &path_);
	virtual int ReadGraph(const std::string &path_);

	virtual int SetMessage(std::shared_ptr<std::vector<std::string> > msg_);
	virtual int SetKB(CKB *kb_);
	virtual int SetOS(COS *os_);

	virtual int WriteWindow(const std::string &path_);

	virtual int ApplyGauss(
			const int &num_x_,
			const int &num_y_);

	virtual int TestInit();
	virtual int TestFaceAdjust(Eigen::Vector4d face_parser);

	virtual int WriteResult(
			const std::string &filename_,
			const std::string &resultdir_,
			bool nolabel_);

	virtual int FilterData(
			const Eigen::Vector4d &pva_in_,
			const int &contact_in_);

	virtual int Predict();

	virtual int Parser(
			const std::string &filename_,
			std::string &display_);

	virtual int GetData(const int &counter);

	virtual int Testing(
			const std::string &filename_,
			const std::string &resultdir_,
			std::string &phrase_,
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
};

#endif /* TEST_H_ */
