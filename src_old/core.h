/*
 * core.h
 *
 *  Created on: Mar 26, 2017
 *      Author: chen
 */

#ifndef CORE_H_
#define CORE_H_

#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>
#include "algo.h"

#define BOUNDARY_VAR 0.1

bool directionCheck(
	Eigen::Vector3d A,
	Eigen::Vector3d B);

int checkBoundarySphere(
	Eigen::Vector4d &point_,
	std::vector<Eigen::Vector4d> centroids_);

int checkBoundaryCuboid(
	Eigen::Vector4d &point_,
	Eigen::Vector3d box_min_,
	Eigen::Vector3d box_max_);

int decideBoundarySphere(
	Eigen::Vector4d 		 &point_,
	std::vector<Eigen::Vector4d> centroids_);

int decideBoundaryCuboid(
	Eigen::Vector4d &point_,
	Eigen::Vector3d box_min_,
	Eigen::Vector3d box_max_);

int decideBoundaryClosest(
	Eigen::Vector4d 		&point_,
	std::vector<Eigen::Vector4d> centroids_);

double checkSurfaceDistance(
	Eigen::Vector4d centroids_,
	Eigen::Vector4d surface_eq_);

bool decideSurface(
	Eigen::Vector4d centroids_,
	Eigen::Vector4d surface_eq_,
	double limit_);

double dLI(
		int &loc_idx_,
		const Eigen::Vector4d &point_,
		const std::vector<double> &len_,
		const std::vector<Eigen::Vector4d> &mid_,
		const std::vector<Eigen::Vector3d> &tangent_,
		const int &loc_last_idx_,
		const int &loc_offset_,
		const int &loc_int_,
		bool loc_init_);

double dLIPredict(
		int &loc_idx_,
		int loc_last_idx_,
		Eigen::Vector4d point_,
		std::vector<Eigen::Vector4d> mid_,
		std::vector<double> len_,
		std::vector<Eigen::Vector3d> tangent_,
		int loc_offset_,
		int loc_init_, //0 for true
		int LOC_INT_);

int decideSectorInterval(
		int &sec_idx_,
		Eigen::Vector3d &delta_t_,
		const Eigen::Vector4d &point_,
		const std::vector<Eigen::Vector4d> &mid_,
		const std::vector<Eigen::Vector3d> &tangent_,
		const std::vector<Eigen::Vector3d> &normal_,
		const int &loc_idx_,
		const int &SEC_INT_);

//int decideCurvature(
//	point_d point_,
//	std::vector<point_d> &curve_mem_,
//	double &curve_,
//	int num_points_);

int folderSelect1(
	const struct dirent *entry);

int folderSelect2(
	const struct dirent *entry);

int fileSelect(
	const struct dirent *entry);

bool copyFile(
		const std::string &SRC,
		const std::string &DEST);

bool directoryCheck(
		const std::string &path_);

#endif /* CORE_H_ */
