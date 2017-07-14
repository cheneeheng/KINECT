/*
 * TestCase.h
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#ifndef TESTCASE_H_
#define TESTCASE_H_

//#include "algo.h"
#include "core.h"
#include "Test.h"
#include "print.h"
#include "Train.h"

#define FILTER_WIN 9
#define	LOC_INT 100
#define	SEC_INT 36

class TestCase
{
	public:
		TestCase();
		virtual ~TestCase();

		void Choose(int x_);
		int TrnInd(
				std::vector<int> idx_,
				std::string object_,
				bool face_);
		int Trn(
				std::vector<int> idx_,
				std::string object_,
				bool face_);
		int Tst(
				std::vector<int> idx_,
				std::string object_,
				bool gauss_,
				bool face_,
				bool os_);
		int Lbl(
				std::vector<int> idx_);
		int ReadFileExt(
				const std::vector<int> &idx_,
				const std::string &object_,
				bool os_);

	private:
		std::string PARENT, EVAL, EVAL2, RESULT, RESULT2, KB_DIR, DATA_DIR;
		std::map<int,std::string> dict;

		int sub_num;

		std::shared_ptr<std::map<int,std::map<int,std::pair<int,std::string> > > >file_list; // subject, file number, action, filename
		std::shared_ptr<std::map<int,std::vector<std::string> > > label_list;
		std::shared_ptr<std::vector<std::string> > message;
		std::shared_ptr<CKB> KB;
		std::shared_ptr<COS> OS;
		//std::shared_ptr<CGraph> G;
		std::shared_ptr<ReadFile> RF;
		std::shared_ptr<WriteFile> WF;

};

#endif /* TESTCASE_H_ */
