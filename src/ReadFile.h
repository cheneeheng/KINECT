/*
 * ReadFile.h
 *
 *  Created on: Apr 16, 2017
 *      Author: chen
 *      Detail: Read data from file.
 */

#ifndef READFILE_H_
#define READFILE_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <dirent.h>
#include <sys/stat.h>
#include "CGraph.h"
#include "CKB.h"
#include "COS.h"
#include "print.h"
#include "core.h"

#include <Eigen/Eigen>

class ReadFile
{
public:

	/**
	 * Constructor for ReadFile class.
	 */
	ReadFile();

	/**
	 * Destructor for ReadFile class.
	 */
	virtual ~ReadFile();

	/**
	 * clear all class variables.
	 */
	void ClearRF();

	/**
	 * read each word of each line.
	 *
	 * @param path_ Path + Filename.
	 * @param delimiter Delimiter in char.
	 */
	int ReadWord(
			const std::string &path_,
			const char &delimiter);

	/**
	 * read each line.
	 *
	 * @param path_ Path + Filename.
	 */
	int ReadLine(
			const std::string &path_);

	/**
	 * read file names.
	 *
	 * @param dir_name_ Path.
	 * @param idx_ Index to choose which subject to take.
	 * @param file_list_ List of file names.
	 * @param sub_num Number of subjects.
	 */
	int ReadFileName(
			const std::string &dir_name_,
			const std::vector<int> &idx_,
			std::map<int, std::map<int, std::pair<int, std::string> > > *file_list_,
			int &sub_num_);

	/**
	 * read file names with ref. position for LA for data set labeling.
	 *
	 * @param dir_name_ Path.
	 * @param label_list_ Label list.
	 */
	int ReadRefLabelFileName(
			std::string dir_name_,
			std::map<std::string, std::string> &label_list_);

	/**
	 * read file with action/LA sequence.
	 *
	 * @param path_ Path + Filename.
	 * @param label_ Label sequence.
	 */
	int ReadLabelSeq(
			const std::string &path_,
			std::shared_ptr<std::map<int, std::vector<std::string> > > label_);

	/**
	 * read files that define the planar surfaces (raw data).
	 *
	 * @param path_ Path + Filename.
	 * @param rotation_ Rotation matrix to align with up vector.
	 * @param planeeq_ Equation of plane of surface.
	 * @param boxmin_ Minimum point of OBB.
	 * @param boxmid_ Midpoint of plane.
	 * @param boxmax_ Maximum point of OBB.
	 */
	int ReadSurfaceFile(
			const std::string &path_,
			std::vector<Eigen::Matrix3d> *rotation_,
			std::vector<Eigen::Vector4d> *planeeq_,
			std::vector<Eigen::Vector3d> *boxmin_,
			std::vector<Eigen::Vector3d> *boxmid_,
			std::vector<Eigen::Vector3d> *boxmax_);

	/**
	 * read files that contain the KB data.
	 *
	 * @param path_ Path + Filename.
	 * @param kb_ Knowledge-base.
	 */
	int ReadFileKB(
			const std::string &path_,
			std::shared_ptr<CKB> kb_);

	int ReadFileKBSurface(
			const std::string &path_,
			std::shared_ptr<CKB> kb_);

	int ReadFileKBSurfaceLimit(
			const std::string &path_,
			std::shared_ptr<CKB> kb_);

	int ReadFileKBActionLabel(
			const std::string &path_,
			std::shared_ptr<CKB> kb_);

	int ReadFileKBActionObjectLabel(
			const std::string &path_,
			std::shared_ptr<CKB> kb_);

	int ReadFileKBActionTransition(
			const std::string &path_,
			std::shared_ptr<CKB> kb_);

	/**
	 * read LAs.
	 *
	 * @param path_ Path + Filename.
	 * @param al_ Action labels.
	 * @param Graph_ Graph of scene.
	 */
	int ReadFileLA(
			const std::string &path_,
			const std::vector<std::string> &al_,
			std::shared_ptr<CGraph> Graph_);

	/**
	 * read SMs.
	 *
	 * @param path_ Path + Filename.
	 * @param Graph_ Graph of scene.
	 */
	int ReadFileGraph(
			const std::string &path_,
			std::shared_ptr<CGraph> Graph_);

	/**
	 * read files that contain the OS data.
	 *
	 * @param path_ Path + Filename.
	 * @param os_ Object state.
	 */
	int ReadFileOS(
			const std::string &path_,
			std::shared_ptr<COS> os_);

	int ReadFileOSObjectLabel(
			const std::string &path_,
			std::shared_ptr<COS> os_);

	int ReadFileOSTransition(
			const std::string &path_,
			std::shared_ptr<COS> os_);

	int ReadFileOSActionObjectLabelState(
			const std::string &path_,
			std::shared_ptr<COS> os_);

	int ReadFileOSActionObjectState(
			const std::string &path_,
			std::shared_ptr<COS> os_);

	int ReadFileOSObjectActionState(
			const std::string &path_,
			std::shared_ptr<COS> os_);

	/**
	 * read the message list for parsing.
	 *
	 * @param path_ Path + Filename.
	 * @param message_ Predefined sentences.
	 */
	int ReadMsg(
			const std::string &path_,
			std::shared_ptr<std::vector<std::string> > message_);

	/**
	 * read the TUM kitchen data set
	 */
	int ReadDataset(
			const std::string &path_,
			std::vector<Eigen::Vector3d> &t_,
			std::vector<std::vector<Eigen::Matrix3d> > &R_,
			std::map<std::string, Eigen::Vector3d> &offset_list_);

	std::vector<std::string> GetDataLineRF()
	{
		return *data_line_rf;
	}
	std::vector<std::vector<std::string> > GetDataWordRF()
	{
		return *data_word_rf;
	}

private:
	struct dirent **list0, **list1, **list2;
	std::vector<std::string> *data_line_rf;
	std::vector<std::vector<std::string> > *data_word_rf;
};

#endif /* READFILE_H_ */
