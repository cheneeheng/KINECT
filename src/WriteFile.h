/*
 * WriteFile.h
 *
 *  Created on: Apr 18, 2017
 *      Author: chen
 *      Detail: Writing data to file.
 */

#ifndef WRITEFILE_H_
#define WRITEFILE_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include "CGraph.h"
#include "CKB.h"
#include "algo.h"

#include <Eigen/Eigen>

/**
 * Writes data to file.
 */
class WriteFile
{
public:

	/**
	 * Constructor for class WriteFile.
	 */
	WriteFile();

	/**
	 * Destructor for class WriteFile.
	 */
	virtual ~WriteFile();

	/**
	 * Rewrites the labels in the original dataset to fill up holes or gaps
	 * in the labeling.
	 *
	 * @param path_ Path to file.
	 * @param data_ List of data from original file.
	 * @param points_ List of points.
	 * @param contact_ List of contact.
	 * @param face_ Updates the label for face.
	 * @param label_ref_write_ Points in label reference
	 * @param label_ref_name_ Name of label reference
	 * @param label_list_ List of label for actions taken
	 */
	void RewriteDataFile(
			std::string path_,
			std::vector<std::vector<std::string> > data_,
			std::vector<Eigen::Vector4d> points_,
			std::vector<int> contact_,
			Eigen::Vector4d face_,
			std::vector<Eigen::Vector3d> label_ref_write_,
			std::vector<std::string> label_ref_name_,
			std::vector<std::string> label_list_);
	void RewriteDataFileFilter(
			int curr_,
			int mem_,
			int mem2_,
			int mem3_,
			std::string &mem_s_,
			std::string &mem_s2_,
			std::string &mem_s3_,
			std::vector<std::string> &label_);

	void WriteFileLA(
			CGraph *Graph_,
			CKB *kb_,
			std::string path_);

//		void WriteFileLA(
//				std::vector<std::string> line_,
//				std::vector<std::vector<std::string> > data_tmp,
//				std::string path_);

	void WriteFileGraph(
			CGraph *Graph_,
			std::string path_);

	void WriteFileWindow(
			CGraph *Graph_,
			std::string path_);

	void WriteFileSurface(
			std::string path_,
			std::vector<Eigen::Matrix3d> rotation_,
			std::vector<Eigen::Vector4d> planeeq_,
			std::vector<Eigen::Vector3d> boxmin_,
			std::vector<Eigen::Vector3d> boxmid_,
			std::vector<Eigen::Vector3d> boxmax_);

	void WriteOSTransition(
			std::string path_,
			std::map<std::string, std::vector<std::vector<int> > > transition_);

	void WriteFilePrediction(
			CGraph *Graph_,
			CKB *kb_,
			std::string path_,
			std::vector<std::string> labels_,
			std::vector<std::map<std::string, double> > goals_,
			std::vector<std::map<std::string, double> > windows_);

	void WriteFilePrediction(
			CGraph *Graph_,
			CKB *kb_,
			std::string path_,
			std::vector<std::string> labels_,
			std::vector<std::string> labels_predict_,
			std::vector<std::map<std::string, double> > goals_,
			std::vector<std::map<std::string, double> > windows_);

	void WriteFile_(
			std::string path_,
			std::vector<std::vector<double> > data_);

};

#endif /* WRITEFILE_H_ */
