/*
 * ActionPrediction.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 *      Detail: SM prediction and LA contact check.
 */

#ifndef ACTIONPREDICTION_H_
#define ACTIONPREDICTION_H_

#include "core.h"
#include "CData.h"
#include "Evaluate.h"
#include "ObjectPrediction.h"
#include "NBayesClassifier.h"

enum class RANGE
{
	ZERO, IN, OUT, EXCEED
};

#define P_WIN_VAR 0.001
#define P_ERR_VAR 0.01

/**
 * Carries out main action prediction inferences.
 */
class ActionPrediction:
		public Evaluate,
		public ObjectPrediction,
		public NBayesClassifier
{

private:
	using node_t = CGraph::node_t;

	/**
	 * Online data structure for node.
	 *
	 * @data acc Acceleration.
	 * @data vel Velocity.
	 * @data surface_dist Distance from a surface.
	 * @data curvature Curvature value of current motion. 1 for line.
	 */
	struct predict_n
	{
		double acc, vel, surface_dist, curvature;
	};

	/**
	 * Online data structure for edge.
	 *
	 * @data acc Acceleration.
	 * @data vel Velocity.
	 * @data curvature Curvature value of current motion. 1 for line.
	 * @data range Flag for whether the motion is within the maximum variance.
	 *             Depends on enum class RANGE.
	 * @data err Prediction error that corresponds to the deviation from the
	 *           maximum variance.
	 * @data pct_err Prediction error normalized.
	 * @data window Radius of sector-map using the maximum variance.
	 */
	struct predict_e // prediction for edge
	{
		double acc, vel, curvature;
		std::vector<double> range, err, pct_err, window;
	};

	// Start node
	int label1_ap;

	// Goal node
	int label2_ap;

	// Flag to activate sector-map learning.
	bool learn;

	// Flag to activate prediction with object state.
	bool os_flag;

	// Flag to indicate change from LA to SM and vice versa.
	bool la_sm_change;

	// Node based on label1_ap. Valid only when object is in LA.
	node_t node_ap;

	// Online information of the edge.
	predict_e pred_sm;

	// Online information of the node.
	predict_n pred_la;

	// List of last known location interval in SM.
	std::vector<int> last_loc;

	// List of initialization flags for SM.
	std::vector<int> init;

	// List of flags to check if motion is with maximum variance.
	std::vector<int> range_in;

	// Memory for pred_sm to calculate the average.
	std::vector<predict_e> pred_sm_mem;

	// Memory for pred_LA to calculate the average.
	std::vector<predict_n> pred_la_mem;

	// Memory for pva_avg to rebuild SM if needed.
	std::vector<std::vector<Eigen::Vector4d> > pva_avg_mem;

	/**
	 * Reshapes the predict_e data structure.
	 *
	 * @param P_ predict_e edge.
	 * @param size Number of SM possibilities involved.
	 */
	virtual void ReshapePredictEdge(
			predict_e &P_,
			const int &size);

	/**
	 * Reshapes the predict_e data structure.
	 *
	 * @param P_ predict_n node.
	 */
	virtual void ReshapePredictNode(
			predict_n &P_);

	/**
	 * Evaluates goal and maximum variance of SM.
	 *
	 * @param cdata_ Data container.
	 */
	virtual void EvaluateLAList(
			std::shared_ptr<CData> cdata_);

public:

	/**
	 * Constructor for class ActionPrediction.
	 *
	 * @param os_flag_ Flag to activate object state evaluation.
	 */
	ActionPrediction(
			bool os_flag_);

	/**
	 * Destructor for class ActionPrediction.
	 */
	virtual ~ActionPrediction();

	/**
	 * Main prediction function.
	 * Evaluates hand-object contact.
	 *
	 * @param cdata_ Data container.
	 */
	virtual void PredictExt(
			std::shared_ptr<CData> cdata_);

	/**
	 * Class member data initialization.
	 *
	 * @param cdata_ Data container.
	 * @param learn_ Flag to activate SM learning.
	 */
	virtual void Init(
			std::shared_ptr<CData> cdata_,
			bool learn_);

	/**
	 * Prediction function.
	 * Checks for hand-object contact.
	 * Checks if motion is within LA or SM.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int Predict(
			std::shared_ptr<CData> cdata_);

	/**
	 * Checks for LA contact.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int ContactTrigger(
			std::shared_ptr<CData> cdata_);

	/**
	 * Determines which LA is the closest.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int DecideBoundaryClosestExt(
			std::shared_ptr<CData> cdata_);

	/**
	 * Determines contact with LA modeled as a sphere.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int DecideBoundarySphereExt(
			std::shared_ptr<CData> cdata_);

	/**
	 * Determines contact with LA modeled as a cuboid.
	 *
	 * @param point_ Trajectory point.
	 * @param box_max_ Maximum OOB point.
	 * @param box_min_ Minimum OOB point.
	 */
	virtual int DecideBoundaryCuboidExt(
			Eigen::Vector4d &point_,
			Eigen::Vector3d box_min_,
			Eigen::Vector3d box_max_);

	/**
	 * Online evaluation of node.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int EvaluateNode(
			std::shared_ptr<CData> cdata_);

	/**
	 * Online evaluation of edge.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int EvaluateEdge(
			std::shared_ptr<CData> cdata_);

	/**
	 * Check for movement.
	 *
	 * @param cdata_ Data container.
	 * @param x_ Flag for LA (false) or SM (true).
	 */
	virtual int DecideMovement(
			std::shared_ptr<CData> cdata_,
			bool x_);

	/**
	 * Sector-map predictions based on motion trajectory.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int PredictFromSectorMap(
			std::shared_ptr<CData> cdata_);

	/**
	 * Determines current location and sector interval within the SM.
	 *
	 * @param cdata_ Data container.
	 * @param delta_t_ Difference from the maximum allowed variance of SM.
	 * @param sec_idx_ Sector interval.
	 * @param loc_idx_ Location interval.
	 * @param loc_last_idx_ Last known location interval.
	 * @param init_ Initialization flag.
	 */
	virtual double DecideLocSecInterval(
			std::shared_ptr<CData> cdata_,
			Eigen::Vector3d &delta_t_,
			int &sec_idx_,
			int &loc_idx_,
			int &loc_last_idx_,
			int &init_);

	/**
	 * Determines current location interval within the SM.
	 *
	 * @param loc_idx_ Current location interval.
	 * @param loc_last_idx_ Last known location interval.
	 * @param point_ Trajectory point.
	 * @param mid_ List of midpoints of a SM.
	 * @param len_ List of the lengths of each location interval of a SM.
	 * @param tangent_ List of tangents of a SM.
	 * @param loc_offset_ Offset to limit search space.
	 * @param loc_init_  Flag for initial search of a SM. 0 for true.
	 * @param loc_int_ Number of location intervals. A parameter.
	 */
	virtual double DecideLocationIntervalExt(
			int &loc_idx_,
			const int &loc_last_idx_,
			const Eigen::Vector4d &point_,
			const std::vector<Eigen::Vector4d> &mid_,
			const std::vector<double> &len_,
			const std::vector<Eigen::Vector3d> &tangent_,
			const int &loc_offset_,
			const int &loc_init_,
			const int &loc_int_);

	/**
	 * Determines current sector interval within the SM.
	 *
	 * @param sec_idx_ Current sector interval.
	 * @param loc_idx_ Current location interval.
	 * @param delta_t_ Difference from the maximum allowed variance of SM.
	 * @param point_ Trajectory point.
	 * @param mid_ List of midpoints of a SM.
	 * @param tangent_ List of tangents of a SM.
	 * @param normal_ List of normals of a SM.
	 * @param sec_int_ Number of sector intervals. A parameter.
	 */
	virtual int DecideSectorIntervalExt(
			int &sec_idx_,
			const int &loc_idx_,
			Eigen::Vector3d &delta_t_,
			const Eigen::Vector4d &point_,
			const std::vector<Eigen::Vector4d> &mid_,
			const std::vector<Eigen::Vector3d> &tangent_,
			const std::vector<Eigen::Vector3d> &normal_,
			const int &sec_int_);

	/**
	 * Determines the goal LA.
	 *
	 * @param label2_ Goal LA.
	 * @param sm_i_ Sector-map.
	 * @param delta_t_ Difference from the maximum allowed variance of SM.
	 * @param loc_error_ Error distance for the case when the trajectory point
	 *                   lies in between 2 location intervals but is not
	 *                   detected as either of them. Occurs for trajectory with
	 *                   high curvatures.
	 */
	virtual bool DecideGoal(
			const int &label2_,
			const double &sm_i_,
			const double &delta_t_,
			const double &loc_error_);

	/**
	 * Determines the maximum variance of a certain location interval.
	 *
	 * @param sm_ SM.
	 * @param sec_int Current sector interval.
	 * @param loc_idx_ Current location interval.
	 * @param label2_ Goal LA.
	 */
	virtual int DecideWindow(
			const std::vector<double> &sm_,
			const int &sec_int,
			const int &loc_idx_,
			const int &label2_);

	/**
	 * Evaluates and update the action state for node.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int UpdateNode(
			std::shared_ptr<CData> cdata_);

	/**
	 * Evaluates and update the action state for edge.
	 *
	 * @param cdata_ Data container.
	 */
	virtual int UpdateEdge(
			std::shared_ptr<CData> cdata_);

	/**
	 * Rebuilds the SM.
	 *
	 * @param pva_avg_ Data needed to rebuild.
	 * @param label1_ Start node.
	 * @param label2_ Goal node.
	 */
	//		int RebuildSectorMap(
	//			std::vector<std::vector<point_d> > pva_avg_,
	//			int	label1_,
	//			int label2_);
};

#endif /* ACTIONPREDICTION_H_ */
