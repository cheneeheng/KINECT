/*
 * util.h
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

#ifndef PFTRACKER_H_
#define PFTRACKER_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>
#include <dbot/builder/particle_tracker_builder.h>
#include <dbot/camera_data.h>
#include <dbot/pose/free_floating_rigid_bodies_state.h>
#include <dbot/simple_wavefront_object_loader.h>
#include <dbot/simple_camera_data_provider.h>
#include <dbot/virtual_camera_data_provider.h>
#include <dbot/tracker/particle_tracker.h>
//#include <dbot/model/depth_pixel_model.h>
#include <fl/util/profiling.hpp>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>

class PFTracker
{

private:
	size_t n_rows, n_cols;

    int downsampling_factor;

	std::shared_ptr<dbot::ParticleTracker> tracker;
	std::vector<dbot::ParticleTracker::State> initial_poses;

	Eigen::Matrix<double, -1, 1> eigen_image;

	int ReadYamlFile(cv::FileStorage &fs_, std::string file_path_);

public:
	PFTracker();
	virtual ~PFTracker();

	virtual int Build(std::string config_path_);

	virtual void Initialize();

	virtual dbot::ParticleTracker::State Track(
			cv::Mat img_depth);

	virtual int ObjectIndex();

	virtual int DownsamplingFactor()
	{
		return downsampling_factor;
	}

	virtual std::vector<dbot::ParticleTracker::State> InitialPoses()
	{
		return initial_poses;
	}

	virtual void InitialPoses(const std::vector<dbot::ParticleTracker::State> &x_)
	{
		initial_poses = x_;
	}

	virtual Eigen::Vector3d EulerAngle(const Eigen::Matrix<double, 3, 3> &x_);

};




#endif /* PFTRACKER_H_ */
