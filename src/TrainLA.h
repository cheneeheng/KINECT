/*
 * TrainLA.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef TRAINLA_H_
#define TRAINLA_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "CGraph.h"
#include "CKB.h"
#include "VTKExtra.h"
#include "print.h"
#include "core.h"
#include "DBSCAN.h"

#define BOUNDARY_VAR 0.1
#define DBSCAN_EPS 0.015
#define DBSCAN_MIN 5

/**
 * Evaluates and builds the LA.
 */
class TrainLA: public DBSCAN
{
public:

	/**
	 * Constructor for class TrainLA.
	 *
	 * @param loc_int_ Number of location interval.
	 * @param sec_int_ Number of sector interval.
	 */
	TrainLA(
			const int &loc_int_,
			const int &sec_int_);

	/**
	 * Destructor for class TrainLA.
	 */
	virtual ~TrainLA();

	/**
	 * Clears all data container in class TrainLA.
	 */
	virtual void ClearLA();

	/**
	 * Clusters the data to find LAs.
	 *
	 * @param centroids_ List of centroids/LA.
	 */
	virtual int ClusteringExt(
			std::shared_ptr<std::vector<Eigen::Vector4d> > centroids_);

	/**
	 * Decides if object is within OBB.
	 *
	 * @param point_ Current trajectory point.
	 * @param box_min_ Minimum point of OBB.
	 * @param box_max_ Maximum point of OBB.
	 */
	virtual int DecideBoundaryCuboidExt(
			Eigen::Vector4d &point_,
			Eigen::Vector3d box_min_,
			Eigen::Vector3d box_max_);

	/**
	 * Learns the boundary of the bounding sphere.
	 *
	 * @param centroids_ List of centroids/LA.
	 */
	virtual int LearnBoundary(
			std::shared_ptr<std::vector<Eigen::Vector4d> > centroids_);

	/**
	 * Checks if the trajectory point is within boundary of sphere and OBB.
	 *
	 * @param centroids_ List of centroids/LA.
	 */
	virtual int ContactBoundary(
			std::shared_ptr<std::vector<Eigen::Vector4d> > centroids_);

	/**
	 * Checks if object is within surface boundary.
	 *
	 * @param centroids_ List of centroids/LA.
	 */
	virtual int SurfaceContactCheck(
			std::shared_ptr<std::vector<Eigen::Vector4d> > centroids_);

	/**
	 * Main function to build LAs.
	 *
	 * @param G_ Graph of the scene.
	 * @param kb_ List of knowledge-base.
	 * @param pos_vel_acc_ Vector of [pos vel acc].
	 * @param contact_flag_ Container of flags. TRUE = contact.
	 * @param flag_ Flag. TRUE = new label is present.
	 */
	virtual int BuildLocationArea(
			std::shared_ptr<CGraph> G_,
			std::shared_ptr<CKB> kb_,
			std::shared_ptr<std::vector<std::vector<Eigen::Vector4d> > > pos_vel_acc_,
			std::shared_ptr<std::vector<int> > contact_flag_,
			bool flag_);

private:
	std::shared_ptr<std::vector<int> > contact_flag; // contact 1/0
	std::shared_ptr<std::vector<int> > locations_flag; // whether location has change in contact 1/0
	std::vector<int> loc_idx_zero; // loc_idx_zero unused at all

	std::shared_ptr<std::vector<Eigen::Vector4d> > points_avg;
	std::shared_ptr<std::vector<Eigen::Vector4d> > locations; // centroids

	std::vector<std::string> goal_action;

	std::vector<Eigen::Vector3d> surfaces_mid;
	std::vector<Eigen::Vector3d> surfaces_min;
	std::vector<Eigen::Vector3d> surfaces_max;
	std::vector<Eigen::Vector4d> surfaces_eq; // equation of plane
	std::vector<double> surfaces_limit; // surface distance limit
	std::vector<int> surfaces_flag;  // flag if surface is detected
	std::vector<Eigen::Matrix3d> surfaces_rot;  // surface rotation

protected:
	int loc_int;
	int sec_int;

};

#endif /* TRAINLA_H_ */
