/*
 * core.cpp
 *
 *  Created on: Mar 26, 2017
 *      Author: chen
 */

#include "core.h"

bool directionCheck(
		Eigen::Vector3d A,
		Eigen::Vector3d B)
{
	for (int i = 0; i < 3; i++)
		if (((A(i) >= 0) && (B(i) < 0)) || ((A(i) < 0) && (B(i) >= 0)))
			return false;
	return true;
}

int checkBoundarySphere(
		Eigen::Vector4d &point_,
		std::vector<Eigen::Vector4d> centroids_)
{
	point_[3] = -1;
	for (int ii = 0; ii < centroids_.size(); ii++)
	{
		double location_contact = pdfExp(BOUNDARY_VAR, 0.0,
				V4d3d(point_ - centroids_[ii]).norm());
//		double boundary = sqrt(-log(centroids_[ii].l)*BOUNDARY_VAR*2);
		if (max_(location_contact, (double) centroids_[ii][3])) //|| pdfExp(0.005,0.0,(l2Norm(tmp_diff)-boundary))>0.75)
		{
			point_[3] = ii;
			break;
		}
	}
	return EXIT_SUCCESS;
}

int checkBoundaryCuboid(
		Eigen::Vector4d &point_,
		Eigen::Vector3d box_min_,
		Eigen::Vector3d box_max_)
{
	if (point_[0] < box_max_[0] && point_[1] < box_max_[1]
			&& point_[2] < box_max_[2] && point_[0] > box_min_[0]
			&& point_[1] > box_min_[1] && point_[2] > box_min_[2])
	{
		return EXIT_SUCCESS;
	}
	else
	{
		point_[3] = -1;
		return EXIT_FAILURE;
	}
}

int decideBoundarySphere(
		Eigen::Vector4d &point_,
		std::vector<Eigen::Vector4d> centroids_)
{
	return checkBoundarySphere(point_, centroids_);
}

int decideBoundaryCuboid(
		Eigen::Vector4d &point_,
		Eigen::Vector3d box_min_,
		Eigen::Vector3d box_max_)
{
	return checkBoundaryCuboid(point_, box_min_, box_max_);
}

int decideBoundaryClosest(
		Eigen::Vector4d &point_,
		std::vector<Eigen::Vector4d> centroids_)
{
	point_[3] = -1;
	std::vector<double> tmp;
	for (int ii = 0; ii < centroids_.size(); ii++)
	{
		tmp.push_back(pdfExp(
		BOUNDARY_VAR, 0.0, V4d3d(point_ - centroids_[ii]).norm()));
	}
	point_[3] = distance(tmp.begin(), max_element(tmp.begin(), tmp.end()));
	return EXIT_SUCCESS;
}

double checkSurfaceDistance(
		Eigen::Vector4d centroids_,
		Eigen::Vector4d surface_eq_)
{
	return centroids_[0] * surface_eq_[0] + centroids_[1] * surface_eq_[1]
			+ centroids_[2] * surface_eq_[2] - surface_eq_[3];
}

bool decideSurface(
		Eigen::Vector4d centroids_,
		Eigen::Vector4d surface_eq_,
		double limit_)
{
	if (fabs(checkSurfaceDistance(centroids_, surface_eq_)) < limit_)
	{
		return true;
	}
	else
	{
		return false;
	}
}

double dLI(
		int &loc_idx_,
		const Eigen::Vector4d &point_,
		const std::vector<double> &len_,
		const std::vector<Eigen::Vector4d> &mid_,
		const std::vector<Eigen::Vector3d> &tangent_,
		const int &loc_last_idx_,
		const int &loc_offset_,
		const int &loc_int_,
		bool loc_init_)
{
	int idx_tmp;
	double d1, d2, d3, d4, d5, d6, d6min, d6min2;
	d6min = 10;
	Eigen::Vector3d proj_dir_tmp, beg, mid, end;

	/* Added an offset to prevent the prediction from jumping too much */
	int idx1 = (loc_last_idx_ < 0 ? 0 : loc_last_idx_);
	int idx2 = (idx1 + loc_offset_ > loc_int_ ? loc_int_ : idx1 + loc_offset_);
	for (int l = idx1; l < idx2; l++)
	{
		mid = V4d3d(mid_[l]) * mid_[l][3];
		beg = mid - (tangent_[l] * (len_[l] / 2));
		end = mid + (tangent_[l] * (len_[l] / 2));
		proj_dir_tmp = tangent_[l] * ((V4d3d(point_) - mid).dot(tangent_[l]));

		d1 = (beg - mid).norm();
		d2 = (end - mid).norm();
		d3 = (beg - (mid + proj_dir_tmp)).norm();
		d4 = (end - (mid + proj_dir_tmp)).norm();
		d5 = (beg - end).norm();

		if (directionCheck(proj_dir_tmp, tangent_[l]))
		{
			if (l == idx1)
			{
				d6min2 = d3 - d5;
				d6 = d6min2;
			}
			/* TODO small error deviation (deadzone) */
			if (d4 <= d2 && (d3 - d5) < 0.01)
			{
				d6 = d3 - d5;
			}
		}
		else
		{
			if (l == idx1)
			{
				d6min2 = d4 - d5;
				d6 = d6min2;
			}
			if (d3 <= d1 && (d4 - d5) < 0.01)
			{
				d6 = d4 - d5;
			}
		}

		/* ignore the first loc_int so that we don't get "stuck" at curves */
		if (l == idx1)
		{
			continue;
		}

		/* breaks only when the location is larger than the current one */
		if (min_(d6, d6min))
		{
			d6min = d6;
			idx_tmp = l;
		}
		else
		{
			if (d6min < 0.0001 && l > idx1)
			{
				break;
			}
		} //inside
	}

	/*to prevent unknown locations at start and end */
	if (d6min > 0.001 && loc_init_)
	{
		loc_idx_ = loc_last_idx_;
	}
	else
	{
		loc_idx_ = idx_tmp;
	}

	return d6min;
}

double dLIPredict(
		int &loc_idx_,
		int loc_last_idx_,
		Eigen::Vector4d point_,
		std::vector<Eigen::Vector4d> mid_,
		std::vector<double> len_,
		std::vector<Eigen::Vector3d> tangent_,
		int loc_offset_,
		int loc_init_, //0 for true
		int LOC_INT_)
{
	int idx_tmp = loc_last_idx_;
	double d1, d2, d3, d4, d5, d6, d6min, d6min2, min_dist;
	d1 = d2 = d3 = d4 = d5 = d6 = 0.0;
	d6min = min_dist = 10;
	Eigen::Vector3d proj_dir_tmp, beg, mid, end;

	// Added an offset buffer to prevent the location prediction from jumping too much
	int idx1 = (
			loc_last_idx_ - (loc_offset_ / 2) < 0 ?
					0 : loc_last_idx_ - (loc_offset_ / 2));
	int idx2 = (idx1 + loc_offset_ > LOC_INT_ ? LOC_INT_ : idx1 + loc_offset_);
	for (int l = idx1; l < idx2; l++)
	{
		mid = V4d3d(mid_[l]) * mid_[l][3];
		beg = mid - (tangent_[l] * (len_[l] / 2));
		end = mid + (tangent_[l] * (len_[l] / 2));
		proj_dir_tmp = tangent_[l] * ((V4d3d(point_) - mid).dot(tangent_[l]));

		d1 = (beg - mid).norm();
		d2 = (end - mid).norm();
		d3 = (beg - (mid + proj_dir_tmp)).norm();
		d4 = (end - (mid + proj_dir_tmp)).norm();
		d5 = (beg - end).norm();

		if (directionCheck(proj_dir_tmp, tangent_[l]))
		{
			// ignore the first loc_int so that we don't get "stuck" at curves
			if (l == idx1)
			{
				d6min2 = d3 - d5;
				d6 = d6min2;
				continue;
			}
			if (d4 <= d2 && (d3 - d5) < 0.005) //### TODO small error deviation (deadzone)
			{
				d6 = d3 - d5;
				if (min_((V4d3d(point_) - mid).norm(), min_dist))
				{
					min_dist = (V4d3d(point_) - mid).norm();
					idx_tmp = l;
				}
			}
		}
		else
		{
			// ignore the first loc_int so that we don't get "stuck" at curves
			if (l == idx1)
			{
				d6min2 = d4 - d5;
				d6 = d6min2;
				continue;
			}
			if (d3 <= d1 && (d4 - d5) < 0.005)
			{
				d6 = d4 - d5;
				if (min_((V4d3d(point_) - mid).norm(), min_dist))
				{
					min_dist = (V4d3d(point_) - mid).norm();
					idx_tmp = l;
				}
			}
		}
	}

	if (loc_init_ == 0) // initial
	{
		min_dist = 10;
		int tmp = 0;
		for (int l = idx1; l < idx2; l++)
		{
			mid = V4d3d(mid_[l]) * mid_[l][3];
			if (min_((V4d3d(point_) - mid).norm(), min_dist))
			{
				min_dist = (V4d3d(point_) - mid).norm();
				tmp = l;
			}
		}

		if (tmp < idx_tmp)
		{
			loc_idx_ = tmp;
		}
		else
		{
			loc_idx_ = idx_tmp;
		}
	}
	else
	{
		loc_idx_ = idx_tmp;
	}

	return d6;
}

int decideSectorInterval(
		int &sec_idx_,
		Eigen::Vector3d &delta_t_,
		const Eigen::Vector4d &point_,
		const std::vector<Eigen::Vector4d> &mid_,
		const std::vector<Eigen::Vector3d> &tangent_,
		const std::vector<Eigen::Vector3d> &normal_,
		const int &loc_idx_,
		const int &SEC_INT_)
{
	//projection of point onto the tangent at mid
	Eigen::Vector3d mid = V4d3d(mid_[loc_idx_]) * mid_[loc_idx_][3];

	Eigen::Vector3d proj_dir_tmp = tangent_[loc_idx_]
			* ((V4d3d(point_) - mid).dot(tangent_[loc_idx_]));

	delta_t_ = V4d3d(point_) - (proj_dir_tmp + mid);

	double angle_tmp = atan2((delta_t_.cross(normal_[loc_idx_])).norm(),
			delta_t_.dot(normal_[loc_idx_]));

	if (!directionCheck(normal_[loc_idx_].cross(delta_t_), tangent_[loc_idx_]))
	{
		angle_tmp *= -1;
	}
	angle_tmp = fmod((2 * M_PI + angle_tmp), (2 * M_PI));
	sec_idx_ = ceil(angle_tmp * (SEC_INT_ / 2) / M_PI) - 1;
	return EXIT_SUCCESS;
}

//int decideCurvature(
//	point_d point_,
//	std::vector<point_d> &curve_mem_,
//	double &curve_,
//	int num_points_)
//{
//	int s = curve_mem_.size()-1;
//	if (s+1<num_points_)
//	{
//		curve_mem_.push_back(point_);
//		curve_ = 0.0;
//	}
//	else
//	{
//		curve_mem_.erase(curve_mem_.begin());
//		curve_mem_.push_back(point_);
//		curve_ =
//				(l2Norm(minusPoint(curve_mem_[0], 	curve_mem_[s/2])) +
//				 l2Norm(minusPoint(curve_mem_[s/2],	curve_mem_[s]))) /
//				 l2Norm(minusPoint(curve_mem_[0],	curve_mem_[s]));
//		curve_ -= 1.0;
//	}
//	// HACK TODO
//	if (curve_>0.4) {curve_ = 0.4;}
//	return EXIT_SUCCESS;
//}

int folderSelect1(
		const struct dirent *entry)
{
	if (entry->d_name[2] == '_')
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int folderSelect2(
		const struct dirent *entry)
{
	size_t found_extension = std::string(entry->d_name).find(".");
	if ((int) found_extension == -1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int fileSelect(
		const struct dirent *entry)
{
	size_t found_extension = std::string(entry->d_name).find(".txt");
	if ((int) found_extension == -1)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

bool copyFile(
		const std::string &SRC,
		const std::string &DEST)
{
	std::ifstream src(SRC, std::ios::binary);
	std::ofstream dest(DEST, std::ios::binary);
	dest << src.rdbuf();
	return src && dest;
}

std::string currentDirectory()
{
	char buff[FILENAME_MAX];
	std::string cwd(getcwd(buff, FILENAME_MAX)); // buff and return are the same
	return cwd;
}

bool directoryCheck(
		const std::string &path_)
{
	if (path_.empty())
	{
		return false;
	}
	std::size_t found = path_.find("/");
	while (found != std::string::npos)
	{
		std::string path
		{ path_.begin(), path_.begin() + found };
		DIR *dir;
		bool exist = false;
		dir = opendir(path.c_str());
		if (dir != NULL)
		{
			exist = true;
			closedir(dir);
		}
		if (!exist)
		{
			mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		}
		found = path_.find("/", found + 1);
	}
	return true;
}
