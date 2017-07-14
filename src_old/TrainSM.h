/*
 * TrainSM.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef TRAINSM_H_
#define TRAINSM_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <memory>
#include "core.h"
#include "CGraph.h"
#include "CKB.h"
#include "VTKExtra.h"
#include "print.h"

// number of fit coefficients
// nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 (for cubic b-spline)
#define NCOEFFS	15
#define NBREAK 	(NCOEFFS - 2)
#define DEGREE 10 //k+1

class TrainSM
{
public:
	TrainSM(
			const int &loc_int_,
			const int &sec_int_);
	virtual ~TrainSM();

	virtual void ClearSM();

	virtual int FitCurve(
			std::vector<Eigen::Vector4d> &points_est_,
			std::vector<Eigen::Vector3d> &coeffs_);
	virtual int DecideSectorIntervalExt(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			Eigen::Vector3d &delta_t_,
			int &sec_idx_,
			const int &loc_idx_);
	double DecideLocationIntervalExt(
			std::shared_ptr<CGraph::edge_t> edge_,
			const Eigen::Vector4d &point_,
			int &loc_idx_,
			const int &loc_last_idx_,
			const int &loc_offset_,
			bool loc_init_);
	virtual int AdjustSectorMap(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			int &loc_last_idx_,
			int &loc_curr_idx_,
			double &delta_t_mem_,
			bool &loc_init_,
			int loc_offset_);
	virtual int AdjustCurve(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			int &loc_last_idx_,
			bool &loc_init_,
			int loc_offset_);
	virtual int AdjustCurveExt(
			std::shared_ptr<CGraph> G_,
			std::vector<Eigen::Vector3d> coeffs_); // from fit curve
	virtual int FitSectorMap(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			int &loc_last_idx_,
			int loc_offset_,
			bool &loc_init_);
	virtual int FitSectorMapInit(
			std::shared_ptr<CGraph> G_,
			std::vector<Eigen::Vector4d> &points_,
			const int &loc_offset_);
	virtual int FitSectorMapExt(
			std::shared_ptr<CGraph> G_,
			const int &loc_offset_);
	virtual int FindWindowConstraint(
			std::shared_ptr<CGraph> G_) {return EXIT_SUCCESS;}
	virtual int UpdateSectorMap(
			std::shared_ptr<CGraph> G_);
	virtual int BuildSectorMap(
			std::shared_ptr<CGraph> G_,
			std::shared_ptr<CKB> kb_,
			std::shared_ptr<std::vector<std::vector<Eigen::Vector4d> > >pva_avg_,
			std::shared_ptr<std::vector<int> >contact_);

	virtual void SetLabel1SM(const int &x_) {label1_sm = x_;}
	virtual void SetLabel2SM(const int &x_) {label2_sm = x_;}

private:
	// label1_sm	 : start location
	// label2_sm	 : goal location
	// label_idx_sm : index of data point for label1_sm
	int label1_sm, label2_sm, label_idx_sm;
	std::vector<Eigen::Vector4d> pos_sm, pos_ind_sm, vel_sm;

protected:
	int loc_int;
	int sec_int;

};

#endif /* TRAINSM_H_ */
