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

/**
 * Evaluates and builds the SM.
 */
class TrainSM
{
public:

	/**
	 * Constructor for class TrainSM.
	 *
	 * @param loc_int_ Number of location interval.
	 * @param sec_int_ Number of sector interval.
	 */
	TrainSM(
			const int &loc_int_,
			const int &sec_int_);

	/**
	 * Destructor for class TrainSM.
	 */
	virtual ~TrainSM();

	/**
	 * Clears all data container in class TrainSM.
	 */
	virtual void ClearSM();

	/**
	 * Fits a polynomial curve to represent the reference trajectory of a SM.
	 *
	 * @param points_est_ Estimated points from the curve fitting.
	 * @param coeffs_ Coefficients of the polynomial.
	 */
	virtual int FitCurve(
			std::vector<Eigen::Vector4d> &points_est_,
			std::vector<Eigen::Vector3d> &coeffs_);

	/**
	 * Decides which sector interval it is in now.
	 *
	 * @param edge_ The SM to run the evaluation.
	 * @param point_ Trajectory point.
	 * @param delta_t_ Perpendicular distance to the data point from curve.
	 * @param sec_idx_ Sector interval to be determined.
	 * @param loc_idx_ Known location interval.
	 */
	virtual int DecideSectorIntervalExt(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			Eigen::Vector3d &delta_t_,
			int &sec_idx_,
			const int &loc_idx_);

	/**
	 * Decides which location interval it is in now.
	 *
	 * @param edge_ The SM to run the evaluation.
	 * @param point_ Trajectory point.
	 * @param loc_idx_ Location interval to be determined.
	 * @param loc_last_idx_ Last known location interval.
	 * @param loc_offset_ Offset to limit the search range of the SM.
	 * @param loc_init_ Flag. TRUE = beginning of the SM.
	 * @return Dead-zone distance between 2 location intervals due to curvature.
	 */
	double DecideLocationIntervalExt(
			std::shared_ptr<CGraph::edge_t> edge_,
			const Eigen::Vector4d &point_,
			int &loc_idx_,
			const int &loc_last_idx_,
			const int &loc_offset_,
			bool loc_init_);

	/**
	 * Updates the SM for a single point (variance).
	 *
	 * @param edge_ The SM to run the evaluation.
	 * @param point_ Trajectory point.
	 * @param loc_last_idx_ Last known location interval.
	 * @param loc_curr_idx_ Current known location interval.
	 * @param loc_init_ Flag. TRUE = beginning of the SM.
	 * @param loc_offset_ Offset to limit the search range of the SM.
	 */
	virtual int AdjustCurve(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			int &loc_last_idx_,
			bool &loc_init_,
			int loc_offset_);

	/**
	 * Fits the a curve and adjusts the SM according to the curve.
	 *
	 * @param G_ Graph of the scene.
	 * @param coeffs_ Coefficients of the polynomial.
	 */
	virtual int AdjustCurveExt(
			std::shared_ptr<CGraph> G_,
			std::vector<Eigen::Vector3d> coeffs_); // from fit curve

	/**
	 * Updates the SM for the first observation for a single point.
	 *
	 * @param edge_ The SM to run the evaluation.
	 * @param point_ Trajectory point.
	 * @param loc_last_idx_ Last known location interval.
	 * @param loc_curr_idx_ Current known location interval.
	 * @param loc_init_ Flag. TRUE = beginning of the SM.
	 * @param loc_offset_ Offset to limit the search range of the SM.
	 */
	virtual int FitSectorMapInit(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			int &loc_last_idx_,
			int &loc_curr_idx_,
			bool &loc_init_,
			int loc_offset_);

	/**
	 * Updates the SM for the first observation.
	 *
	 * @param G_ Graph of the scene.
	 * @param point_ Trajectory point.
	 * @param loc_offset_ Offset to limit the search range of the SM.
	 */
	virtual int FitSectorMapInitExt(
			std::shared_ptr<CGraph> G_,
			std::vector<Eigen::Vector4d> &points_,
			const int &loc_offset_);

	/**
	 * Updates the SM for each observation for a single point.
	 *
	 * @param edge_ The SM to run the evaluation.
	 * @param point_ Trajectory point.
	 * @param loc_last_idx_ Last known location interval.
	 * @param loc_offset_ Offset to limit the search range of the SM.
	 * @param loc_init_ Flag. TRUE = beginning of the SM.
	 */
	virtual int FitSectorMap(
			std::shared_ptr<CGraph::edge_t> edge_,
			Eigen::Vector4d point_,
			int &loc_last_idx_,
			int loc_offset_,
			bool &loc_init_);

	/**
	 * Updates the SM for each observation.
	 *
	 * @param G_ Graph of the scene.
	 * @param loc_offset_ Offset to limit the search range of the SM.
	 */
	virtual int FitSectorMapExt(
			std::shared_ptr<CGraph> G_,
			const int &loc_offset_);

	/**
	 * Finds the window constraint. UNUSED.
	 */
	virtual int FindWindowConstraint(
			std::shared_ptr<CGraph> G_)
	{
		return EXIT_SUCCESS;
	}

	/**
	 * Main function to build a SM.
	 *
	 * @param G_ Graph of the scene.
	 */
	virtual int UpdateSectorMap(
			std::shared_ptr<CGraph> G_);

	/**
	 * Main function to build SM from overall data.
	 *
	 * @param G_ Graph of the scene.
	 * @param kb_ List of knowledge-base.
	 * @param pva_avg_ Vector of [pos vel acc].
	 * @param contact_ Container of flags. TRUE = contact.
	 */
	virtual int BuildSectorMap(
			std::shared_ptr<CGraph> G_,
			std::shared_ptr<CKB> kb_,
			std::shared_ptr<std::vector<std::vector<Eigen::Vector4d> > > pva_avg_,
			std::shared_ptr<std::vector<int> > contact_);

	virtual void SetLabel1SM(
			const int &x_)
	{
		label1_sm = x_;
	}
	virtual void SetLabel2SM(
			const int &x_)
	{
		label2_sm = x_;
	}

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
