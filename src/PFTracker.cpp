/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

//#define DBSCAN

#include "PFTracker.h"


PFTracker::PFTracker() : tracker(nullptr)
{
}

PFTracker::~PFTracker() { }

int PFTracker::Build(std::string config_path_)
{
	cv::FileStorage fs;
    std::string path_; // FOR ROS

    std::string object_package; // FOR ROS
    std::string object_directory;
    std::vector<std::string> object_meshes;

    std::string camera_info_topic; // FOR ROS
    std::string depth_image_topic; // FOR ROS
    dbot::CameraData::Resolution resolution;
    std::string camera_frame_id;

    /* ------------------------------ */
    /* - Few types we will be using - */
    /* ------------------------------ */
    typedef dbot::FreeFloatingRigidBodiesState<> 	State;
    typedef dbot::ParticleTracker 					Tracker;
    typedef dbot::ParticleTrackerBuilder<Tracker> 	TrackerBuilder;
    typedef TrackerBuilder::TransitionBuilder 		TransitionBuilder;
    typedef TrackerBuilder::SensorBuilder 			SensorBuilder;

    dbot::RbSensorBuilder<State>::Parameters params_obsrv;

    TrackerBuilder::Parameters params_tracker;

    /* ------------------------------ */
    /* - Create the object model    - */
    /* ------------------------------ */
    // get object parameters
    /// \todo nh.getParam does not check whether the parameter exists in the
    /// config file. this is dangerous, we should use ri::read instead
	path_ = config_path_ + "/config/object.yaml";
	if(this->ReadYamlFile(fs, path_)==EXIT_FAILURE){ return EXIT_FAILURE; }
	cv::FileNode nnn0 = fs["object"];
	cv::FileNodeIterator it = nnn0["meshes"].begin();
	cv::FileNodeIterator it_end = nnn0["meshes"].end(); // Go through the node
	for (; it != it_end; ++it) object_meshes.push_back((std::string)*it);
	//object_package = (std::string)nnn0["package"]; // FOR ROS
	object_directory = (std::string)nnn0["directory"];
    // Use the ORI to load the object model usign the
    // SimpleWavefrontObjectLoader
    dbot::ObjectResourceIdentifier ori;
    //ori.package_path(ros::package::getPath(object_package)); // FOR ROS
    ori.directory("../" + object_directory);
    ori.meshes(object_meshes);
    auto object_model_loader = std::shared_ptr<dbot::ObjectModelLoader>(
        new dbot::SimpleWavefrontObjectModelLoader(ori));

    // Load the model usign the simple wavefront load and center the frames
    // of all object part meshes
    bool center_object_frame = false;
    auto object_model = std::make_shared<dbot::ObjectModel>(
        object_model_loader, center_object_frame);

 	/* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
	path_ = config_path_ + "/config/camera.yaml";
	if(this->ReadYamlFile(fs, path_)==EXIT_FAILURE){ return EXIT_FAILURE; }
	cv::FileNode nn0 = fs["resolution"];
	camera_info_topic = (std::string)fs["camera_info_topic"];
	depth_image_topic = (std::string)fs["depth_image_topic"];
	downsampling_factor = (int)fs["downsampling_factor"];
	resolution.width = (int)nn0["width"];
	resolution.height = (int)nn0["height"];

    auto camera_data_provider =
			std::shared_ptr<dbot::CameraDataProvider>(
        			new dbot::VirtualCameraDataProvider(
							downsampling_factor, camera_frame_id));
    auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

	/* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    // We will use a linear observation model built by the object transition
    // model builder. The linear model will generate a random walk.
    dbot::ObjectTransitionBuilder<State>::Parameters params_state;
	path_ = config_path_ + "/config/particle_tracker_gpu.yaml";
	if(this->ReadYamlFile(fs, path_)==EXIT_FAILURE){ return EXIT_FAILURE; }
	cv::FileNode n0 = fs["particle_filter"];
	cv::FileNode n1 = n0["object_transition"];
	params_state.linear_sigma_x  = (float)n1["linear_sigma_x"];
	params_state.linear_sigma_y  = (float)n1["linear_sigma_y"];
	params_state.linear_sigma_z  = (float)n1["linear_sigma_z"];
	params_state.angular_sigma_x = (float)n1["angular_sigma_x"];
	params_state.angular_sigma_y = (float)n1["angular_sigma_y"];
	params_state.angular_sigma_z = (float)n1["angular_sigma_z"];
	params_state.velocity_factor = (float)n1["velocity_factor"];
    params_state.part_count 	 = object_meshes.size();
    auto state_trans_builder = std::shared_ptr<TransitionBuilder>(
        new dbot::ObjectTransitionBuilder<State>(params_state));

	/* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
	params_obsrv.use_gpu = true;
	cv::FileNode n2 = n0["gpu"];
	params_obsrv.sample_count = (int)n2["sample_count"];
	params_obsrv.use_custom_shaders = false;
	params_obsrv.vertex_shader_file = (std::string)n2["vertex_shader_file"];
	params_obsrv.fragment_shader_file = (std::string)n2["fragment_shader_file"];
	params_obsrv.geometry_shader_file = (std::string)n2["geometry_shader_file"];
	cv::FileNode n3 = n0["observation"];
	cv::FileNode n4 = n3["occlusion"];
	params_obsrv.occlusion.p_occluded_visible = (float)n4["p_occluded_visible"];
	params_obsrv.occlusion.p_occluded_occluded = (float)n4["p_occluded_occluded"];
	params_obsrv.occlusion.initial_occlusion_prob = (float)n4["initial_occlusion_prob"];
	cv::FileNode n5 = n3["kinect"];
	params_obsrv.kinect.tail_weight  = (float)n5["tail_weight"];
	params_obsrv.kinect.model_sigma  = (float)n5["model_sigma"];
	params_obsrv.kinect.sigma_factor = (float)n5["sigma_factor"];
    params_obsrv.delta_time = 1. / 30.;
    auto sensor_builder =
        std::shared_ptr<SensorBuilder>(new dbot::RbSensorBuilder<State>(
            object_model, camera_data, params_obsrv));

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    params_tracker.evaluation_count = params_obsrv.sample_count;
	params_tracker.moving_average_update_rate = (float)n0["moving_average_update_rate"];
	params_tracker.max_kl_divergence = (float)n0["max_kl_divergence"];
	params_tracker.center_object_frame = true;
    auto tracker_builder = dbot::ParticleTrackerBuilder<Tracker>(
        state_trans_builder, sensor_builder, object_model, params_tracker);
    tracker = tracker_builder.build();
      
    initial_poses.push_back(Tracker::State(ori.count_meshes()));

	n_rows = 480 / downsampling_factor;
	n_cols = 640 / downsampling_factor;
	eigen_image.resize(n_rows * n_cols, 1);

	return EXIT_SUCCESS;	
}

void PFTracker::Initialize()
{
	tracker->initialize(initial_poses);
}

dbot::ParticleTracker::State PFTracker::Track(
			cv::Mat img_depth)
{
	for (size_t row = 0; row < n_rows; row++)
		for (size_t col = 0; col < n_cols; col++)
		    eigen_image(row * n_cols + col) =
				(double)img_depth.at<uint16_t>(
						row * downsampling_factor,
						col * downsampling_factor)/1000.0;

	return tracker->track(eigen_image);
}

int PFTracker::ObjectIndex()
{
	return tracker->object_pixel_index();
}

int PFTracker::ReadYamlFile(cv::FileStorage &fs_, std::string file_path_)
{
	fs_.release();
	fs_.open(file_path_, CV_STORAGE_READ);
	if(!fs_.isOpened())
	{
		std::cerr << "Couldn't open .yaml" << std::endl;
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

Eigen::Vector3d PFTracker::EulerAngle(const Eigen::Matrix<double, 3, 3> &rm_)
{
	double m00 = rm_(0,0);
	double m02 = rm_(0,2);
	double m10 = rm_(1,0);
	double m11 = rm_(1,1);
	double m12 = rm_(1,2);
	double m20 = rm_(2,0);
	double m22 = rm_(2,2);

	Eigen::Vector3d rot(0.0,0.0,0.0);

	// Assuming the angles are in radians.
	if (m10 > 0.998)
	{ // singularity at north pole
		rot[0] = 0;
		rot[1] = M_PI/2;
		rot[2] = atan2(m02,m22);
	}
	else if (m10 < -0.998)
	{ // singularity at south pole
		rot[0] = 0;
		rot[1] = -M_PI/2;
		rot[2] = atan2(m02,m22);
	}
	else
	{
		rot[0] = atan2(-m12,m11);
		rot[1] = asin(m10);
		rot[2] = atan2(-m20,m00);
	}
	
	return (rot *= 180.0/M_PI);
}





