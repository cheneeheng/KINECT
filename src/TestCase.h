/*
 * TestCase.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 *      Detail: Test cases.
 */

#ifndef TESTCASE_H_
#define TESTCASE_H_

//#include "algo.h"
#include "core.h"
#include "Test.h"
#include "print.h"
#include "Train.h"

#ifdef DATA1
#define FILTER_WIN 5
#elif DATA2
#define FILTER_WIN 9
#else
#define FILTER_WIN 5
#endif

#define	LOC_INT 100
#define	SEC_INT 36

/**
 * Test Cases
 */
class TestCase
{
public:

	/**
	 * Constructor for TestCase class.
	 */
	TestCase();

	/**
	 * Destructor for TestCase class.
	 */
	virtual ~TestCase();

	/**
	 * Chooses which test case to evaluate.
	 *
	 * @param x_ Test case number.
	 */
	void Choose(
			int x_);

	/**
	 * Builds the SM and LA using all datasets.
	 *
	 * @param idx_ Index number of the folder to evaluate.
	 * @param object_ Name of the object.
	 * @param face_ Flag. TRUE = LA face will be modified.
	 */
	int TrnInd(
			std::vector<int> idx_,
			std::string object_,
			bool face_);

	/**
	 * Builds the SM and LA using LOOCV.
	 *
	 * @param idx_ Index number of the folder to evaluate.
	 * @param object_ Name of the object.
	 * @param face_ Flag. TRUE = LA face will be modified.
	 */
	int Trn(
			std::vector<int> idx_,
			std::string object_,
			bool face_);

	/**
	 * Online evaluation.
	 *
	 * @param idx_ Index number of the folder to evaluate.
	 * @param object_ Name of the object.
	 * @param gauss_ Flag. TRUE = SM will be smoothed with gaussian filter.
	 * @param face_ Flag. TRUE = LA face will be modified.
	 * @param os_ Flag. TRUE = OS will be included.
	 */
	int Tst(
			std::vector<int> idx_,
			std::string object_,
			bool gauss_,
			bool face_,
			bool os_);

	/**
	 * Labels the dataset. Compares the with a set off reference points.
	 *
	 * @param idx_ Index number of the folder to evaluate.
	 */
	int Lbl(
			std::vector<int> idx_);

	/**
	 * Reads the data needed for online evaluation.
	 *
	 * @param cdata_ Data container.
	 * @param object_ Name of the object.
	 * @param os_ Flag. TRUE = OS will be included.
	 */
	int ReadFileCData(
			std::shared_ptr<CData> cdata_,
			const std::string &object_,
			bool os_);

	/**
	 * Reads filename and action label list.
	 *
	 * @param idx_ Index number of the folder to evaluate.
	 */
	int ReadFileExt(
			const std::vector<int> &idx_);

private:

	std::string PARENT, EVAL, EVAL2, RESULT, RESULT2, KB_DIR, DATA_DIR;
	std::map<int, std::string> dict;

	int sub_num;

	std::shared_ptr<std::map<int, std::map<int, std::pair<int, std::string> > > > file_list; // subject, file number, action, filename
	std::shared_ptr<std::map<int, std::vector<std::string> > > label_list;
	std::shared_ptr<ReadFile> RF;
	std::shared_ptr<WriteFile> WF;

};

#endif /* TESTCASE_H_ */
