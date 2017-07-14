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

#define RANGE_NULL		0
#define	RANGE_IN		1
#define RANGE_OUT		2
#define RANGE_EXCEED	3

#define RELEASE			0
#define GRABBED			1
#define RELEASE_CLOSE	2
#define GRABBED_CLOSE	3

#define P_WIN_VAR 0.001
#define P_ERR_VAR 0.01

class ActionPrediction : public CData,
						 public Evaluate,
						 public ObjectPrediction
{

private:
	using node_t = CGraph::node_t;

	struct predict_n // prediction for node
	{
		double 	acc; // acc
		double 	vel; // velocity limit 0/1
		double 	surface_dist; // surface distance
		double 	curvature; // curvature value : 1 for line
	};
	struct predict_e // prediction for edge
	{
		double 				acc; // acc
		double 				vel; // velocity limit 0/1
		double 				curvature; // curvature value : 1 for line
		std::vector<double>	range; // in or outside
		std::vector<double> err; // prediction error = diff from the sectormap
		std::vector<double> pct_err; // prob shared between multiple predictions of inside
		std::vector<double>	err_diff; // change in the error compared to original
		std::vector<double>	pct_err_diff; // change in the error compared to original
		std::vector<double>	window; // knot in trajectory
	};

	int label1_ap; // start
	int label2_ap; // goal
	bool learn;
	bool os_flag;
	bool la_sm_change; // change flag from sm to la
	node_t node_ap; //from label1
	predict_e pred_sm; // edge_prediction
	predict_n pred_la; // node_prediction
	std::vector<int> last_loc;
	std::vector<int> init;
	std::vector<int> range_in;
	std::vector<predict_e> pred_sm_mem;
	std::vector<predict_n> pred_la_mem;
	std::vector<std::vector<Eigen::Vector4d> > pva_avg_mem; // rebuild SM

	virtual void reshapePredictEdge(
			predict_e &P_,
			const int &size);
	virtual void reshapePredictNode(
			predict_n &P_,
			const int &size);

public:
	ActionPrediction(
			const std::string &object_,
			const int &loc_int_,
			const int &sec_int_,
			std::shared_ptr<CKB> KB_,
			std::shared_ptr<COS> OS_,
			bool os_flag_);
	virtual ~ActionPrediction();

	virtual void PredictExt();
	virtual void Init(
			bool learn_);
	virtual int Predict();
	virtual int ContactTrigger();
	virtual int DecideBoundaryClosestExt();
	virtual int DecideBoundarySphereExt();
	virtual int DecideBoundaryCuboidExt(
			Eigen::Vector4d &point_,
			Eigen::Vector3d box_min_,
			Eigen::Vector3d box_max_);
	virtual int NodePrediction();
	virtual int EdgePrediction();
	virtual int DecideMovement(bool x_);
	virtual int PredictFromSectorMap();
	virtual double DecideLocSecInt(
			Eigen::Vector3d &delta_t_,
			int &sec_idx_,
			int &loc_idx_,
			int &loc_last_idx_,
			int &init_);
	virtual double DecideLocationIntervalExt(
			int &loc_idx_,
			const int &loc_last_idx_,
			const Eigen::Vector4d &point_,
			const std::vector<Eigen::Vector4d> &mid_,
			const std::vector<double> &len_,
			const std::vector<Eigen::Vector3d> &tangent_,
			const int &loc_offset_,
			const int &loc_init_);
	virtual int DecideSectorIntervalExt(
			int &sec_idx_,
			const int &loc_idx_,
			Eigen::Vector3d &delta_t_,
			const Eigen::Vector4d &point_,
			const std::vector<Eigen::Vector4d> &mid_,
			const std::vector<Eigen::Vector3d> &tangent_,
			const std::vector<Eigen::Vector3d> &normal_);
	virtual bool DecideGoal(
			const int &label2_,
			const double &sm_i_, //sectorstd::map single
			const double &delta_t_,
			const double &loc_error_);
	virtual int DecideWindow(
			const std::vector<double> &sm_,
			const int &loc_idx_,
			const int &label2_);
	virtual int EvaluateNodePrediction();
	virtual int EvaluateEdgePrediction();

	//		int RebuildSectorMap(
	//			std::vector<std::vector<point_d> > pva_avg_,
	//			int	label1_,
	//			int label2_);

};

#endif /* ACTIONPREDICTION_H_ */
