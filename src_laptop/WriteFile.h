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

using namespace std;
using namespace Eigen;

class WriteFile
{
public:
	WriteFile();
	virtual ~WriteFile();

	void RewriteDataFileFilter(
			int curr_,
			int mem_,
			int mem2_,
			int mem3_,
			string &mem_s_,
			string &mem_s2_,
			string &mem_s3_,
			vector<string> &label_);

	void RewriteDataFile(
			string path_,
			vector<vector<string> > data_,
			vector<Vector4d> points_,
			vector<int> contact_,
			Vector4d face_,
			vector<Vector3d> label_ref_write_, // points in label reference
			vector<string> label_ref_name_, // name of label reference
			vector<string> label_list_); // list of label for actions taken

	void WriteFileLA(
			CGraph *Graph_,
			CKB *kb_,
			string path_);

//		void WriteFileLA(
//				vector<string> line_,
//				vector<vector<string> > data_tmp,
//				string path_);

	void WriteFileGraph(
			CGraph *Graph_,
			string path_);

	void WriteFileWindow(
			CGraph *Graph_,
			string path_);

	void WriteFileSurface(
			string path_,
			vector<Matrix3d> rotation_,
			vector<Vector4d> planeeq_,
			vector<Vector3d> boxmin_,
			vector<Vector3d> boxmid_,
			vector<Vector3d> boxmax_);

	void WriteOSTransition(
			string path_,
			map<string, vector<vector<int> > > transition_);

	void WriteFilePrediction(
			CGraph *Graph_,
			CKB *kb_,
			string path_,
			vector<string> labels_,
			vector<map<string, double> > goals_,
			vector<map<string, double> > windows_);

	void WriteFilePrediction(
			CGraph *Graph_,
			CKB *kb_,
			string path_,
			vector<string> labels_,
			vector<string> labels_predict_,
			vector<map<string, double> > goals_,
			vector<map<string, double> > windows_);

	void WriteFile_(
			string path_,
			vector<vector<double> > data_);

};

#endif /* WRITEFILE_H_ */
