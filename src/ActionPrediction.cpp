/*
 * ActionPrediction.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "ActionPrediction.h"

ActionPrediction::ActionPrediction(
		bool os_flag_)
		: label1_ap(-1),
				label2_ap(-1),
				learn(false),
				os_flag(os_flag_),
				la_sm_change(true),
				node_ap
				{ },
				pred_sm
				{ },
				pred_la
				{ }
{
}

ActionPrediction::~ActionPrediction()
{
}

void ActionPrediction::EvaluateLAList(
		std::shared_ptr<CData> cdata_)
{
	auto g = cdata_->AS->Goal();
	auto w = cdata_->AS->Window();
	for (int i = cdata_->KB->AC()["GEOMETRIC"].first;
			i < cdata_->KB->AC()["GEOMETRIC"].second + 1; i++)
	{
		g[cdata_->KB->AL()[i]] = 0.0;
		w[cdata_->KB->AL()[i]] = 0.0;
	}
	cdata_->AS->Goal(g);
	cdata_->AS->Window(w);
}

void ActionPrediction::ReshapePredictEdge(
		predict_e &P_,
		const int &size)
{
	P_.acc = 0.0;
	P_.vel = 0.0;
	P_.curvature = 0.0;
	reshapeVector(P_.range, size);
	reshapeVector(P_.err, size);
	reshapeVector(P_.pct_err, size);
	reshapeVector(P_.window, size);
}

void ActionPrediction::ReshapePredictNode(
		predict_n &P_)
{
	P_.acc = 0.0;
	P_.vel = 0.0;
	P_.surface_dist = 0.0;
	P_.curvature = 0.0;
}

void ActionPrediction::Init(
		std::shared_ptr<CData> cdata_,
		bool learn_)
{
	learn = learn_;
	la_sm_change = true;

	reshapeVector(init, cdata_->G->GetNumberOfNodes());
	reshapeVector(range_in, cdata_->G->GetNumberOfNodes());
	reshapeVector(last_loc, cdata_->G->GetNumberOfNodes());
	ReshapePredictEdge(pred_sm, cdata_->G->GetNumberOfNodes());
	ReshapePredictNode(pred_la);

	al_eval = cdata_->KB->AL();
	ac_eval = cdata_->KB->AC();

	cdata_->AS->Grasp(GRAB::RELEASE);
	cdata_->AS->Label1(-1);
	cdata_->AS->Label2(-1);
	cdata_->AS->OS(0);
	cdata_->AS->Velocity(0.0);
	cdata_->AS->Probability(-1);
	cdata_->AS->SurfaceFlag(-1);
	cdata_->AS->SurfaceName("");
	cdata_->AS->SurfaceDistance(0.0);

	this->EvaluateLAList(cdata_);

	cdata_->pva->clear();
	cdata_->pva->resize(3);
	*(cdata_->contact) = 0;
}

void ActionPrediction::PredictExt(
		std::shared_ptr<CData> cdata_)
{
	if (*(cdata_->contact) == 1)
	{
		if (cdata_->AS->Grasp() == GRAB::RELEASE)
		{
			cdata_->AS->Grasp(GRAB::GRABBED_CLOSE);
			this->Predict(cdata_);
		}
		else
		{
			cdata_->AS->Grasp(GRAB::GRABBED);
			this->Predict(cdata_);
		}
	}
	else
	{
		if (cdata_->AS->Grasp() == GRAB::GRABBED)
		{
			cdata_->AS->Grasp(GRAB::RELEASED_CLOSE);
			this->Predict(cdata_);
		}
		else
		{
			cdata_->AS->Grasp(GRAB::RELEASE);
			cdata_->AS->Label2(cdata_->AS->Label1());
			cdata_->AS->Velocity(0.0);
			// cdata_->AS->pct_err 	= -1; // should be -1 because it is in LA
			// cdata_->AS->sur 		= 0; // should just follow whatever that was determined beforehand
			cdata_->AS->SurfaceDistance(0.0);
			this->EvaluateLAList(cdata_);
		}
	}
}

int ActionPrediction::Predict(
		std::shared_ptr<CData> cdata_)
{
	// 1. Contact trigger
	// 1.1 Check if the object is within a sphere volume of the location areas
	this->ContactTrigger(cdata_);

	// 2. Prediction during motion
	if ((*(cdata_->pva))[0][3] < 0)
	{
		if (!la_sm_change)
		{
			pred_la_mem.clear();
		}
		la_sm_change = true;
		pva_avg_mem.push_back(*(cdata_->pva)); // rebuild SM
		this->EvaluateEdge(cdata_);
	}

	// 3. Prediction within location area
	else
	{
		if (la_sm_change)
		{
			// i. clear SM prediction, reset SM initial and last location flags
			{
				pred_sm_mem.clear();
				reshapeVector(init, cdata_->G->GetNumberOfNodes());
				reshapeVector(last_loc, cdata_->G->GetNumberOfNodes());
			}

			// ii. update label 1
			{
				label1_ap = (*(cdata_->pva))[0][3];
				node_ap = cdata_->G->GetNode(label1_ap);
				this->UpdateOS(*(cdata_->OS), cdata_->G->GetObject(),
						node_ap.name);
			}

			// iii. update action state_eval
			{
				for (int i = 0; i < cdata_->KB->AL().size(); i++)
				{
					if (node_ap.name == cdata_->KB->AL()[i])
					{
						cdata_->AS->Label1(i);
						cdata_->AS->Label2(i);
					}
				}
				this->EvaluateLAList(cdata_);
				cdata_->AS->OS(cdata_->OS->OSLabel());
				cdata_->AS->Velocity(V4d3d((*(cdata_->pva))[1]).norm());
				cdata_->AS->Probability(-1);
				cdata_->AS->SurfaceFlag(node_ap.surface_flag);
				cdata_->AS->SurfaceName(node_ap.name);
			}
		}

		this->EvaluateNode(cdata_);

		la_sm_change = false;
	}

	return EXIT_SUCCESS;
}

int ActionPrediction::ContactTrigger(
		std::shared_ptr<CData> cdata_)
{
	// initial case
	if (label1_ap < 0)
	{
		this->DecideBoundaryClosestExt(cdata_);
		cdata_->AS->Grasp(GRAB::GRABBED_CLOSE);
	}
	else
	{
		// give a starting location during change from release to move and vice versa
		if ((cdata_->AS->Grasp() == GRAB::GRABBED_CLOSE)
				|| (cdata_->AS->Grasp() == GRAB::RELEASED_CLOSE))
		{
			this->DecideBoundaryClosestExt(cdata_);
		}
		else
		{
			this->DecideBoundarySphereExt(cdata_);

			if ((*(cdata_->pva))[0][3] >= 0)
			{
				if (cdata_->G->GetNode((int) (*(cdata_->pva))[0][3]).surface_flag
						> 0)
				{
					Eigen::Vector4d point_rot, point_rot2;
					Eigen::Matrix4d T;
					auto tmp = (*(cdata_->pva))[0];
					auto flag = cdata_->G->GetNode((int) tmp[3]).surface_flag;
					T << cdata_->KB->SurfaceRotation()[flag - 1].row(0), 0,
						 cdata_->KB->SurfaceRotation()[flag - 1].row(1), 0,
						 cdata_->KB->SurfaceRotation()[flag - 1].row(2), 0,
						 0, 0, 0, 0;

					// Last element of T is zero because last std::vector element is used for labeling.
					// Point transform to the coordinate system of the surface.
					// p' = [R,R*t]*p
					point_rot =
							(T * tmp)
							- (T * cdata_->G->GetNode((int) tmp[3]).centroid);
					point_rot[3] = tmp[3];

					this->DecideBoundaryCuboidExt(point_rot,
							cdata_->G->GetNode((int) tmp[3]).cuboid_min,
							cdata_->G->GetNode((int) tmp[3]).cuboid_max);
					(*(cdata_->pva))[0][3] = point_rot[3];
				}

				// prevent from going to unknown goal locations during middle of movement
				for (int i = 0; i < cdata_->G->GetNumberOfNodes(); i++)
				{
					auto tmp = (*(cdata_->pva))[0];
					if (pred_sm.pct_err[i] > 0
							&& last_loc[i] > cdata_->G->GetLocInt() / 10
							&& last_loc[i]
									< cdata_->G->GetLocInt()
											- (cdata_->G->GetLocInt() / 10))
					{
						if (pred_sm.pct_err[(int) tmp[3]] == 0
								&& pred_sm.range[(int) tmp[3]]
										!= (int) RANGE::EXCEED)
						{
							(*(cdata_->pva))[0][3] = -1;
						}
						break;
					}
				}
			}
		}
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::DecideBoundarySphereExt(
		std::shared_ptr<CData> cdata_)
{
	return decideBoundarySphere((*(cdata_->pva))[0],
			cdata_->G->GetCentroidList());
}

int ActionPrediction::DecideBoundaryCuboidExt(
		Eigen::Vector4d &point_,
		Eigen::Vector3d cuboid_min_,
		Eigen::Vector3d cuboid_max_)
{
	Eigen::Vector3d tmp = cuboid_max_ - cuboid_min_;			// tmp[1] = 0.0;
	cuboid_min_ -= (tmp * 0.2);
	cuboid_max_ += (tmp * 0.2);
	return decideBoundaryCuboid(point_, cuboid_min_, cuboid_max_);
}

int ActionPrediction::DecideBoundaryClosestExt(
		std::shared_ptr<CData> cdata_)
{
	return decideBoundaryClosest((*(cdata_->pva))[0],
			cdata_->G->GetCentroidList());
}

int ActionPrediction::EvaluateEdge(
		std::shared_ptr<CData> cdata_)
{
	ReshapePredictEdge(pred_sm, cdata_->G->GetNumberOfNodes());

	// 1. Check for motion
	this->DecideMovement(cdata_, true);

	// 2. Check if the trajectory is within the range of sector std::map
	this->PredictFromSectorMap(cdata_);

	// prediction average
	{
		pred_sm_mem.push_back(pred_sm);
		if (pred_sm_mem.size() > 3)
		{
			pred_sm_mem.erase(pred_sm_mem.begin());
		}
	}

	// 3. Predict the goal based on the trajectory error from sector std::map
	this->UpdateEdge(cdata_);

	return EXIT_SUCCESS;
}

int ActionPrediction::EvaluateNode(
		std::shared_ptr<CData> cdata_)
{
	ReshapePredictNode(pred_la);

	// 1. check movement
	this->DecideMovement(cdata_, false);

	// 2. check surface distance
	if (node_ap.surface_flag > 0)
	{
		pred_la.surface_dist = checkSurfaceDistance((*(cdata_->pva))[0],
				cdata_->KB->SurfaceEquation()[node_ap.surface_flag - 1]);
	}

	// prediction average
	{
		pred_la_mem.push_back(pred_la);
		if (pred_la_mem.size() > 3)
		{
			pred_la_mem.erase(pred_la_mem.begin());
		}
	}

	// 3. Evaluate (for now whether it is moving on the surface)
	this->UpdateNode(cdata_);

	return EXIT_SUCCESS;
}

// #### TODO: vel limit
int ActionPrediction::DecideMovement(
		std::shared_ptr<CData> cdata_,
		bool x_)
{
	// edge
	if (x_)
	{
		if (V4d3d((*(cdata_->pva))[1]).norm() < 0.001)
		{
			pred_sm.vel = pred_sm.acc = 0.0;
		}
		else
		{
			pred_sm.vel = V4d3d((*(cdata_->pva))[1]).norm();
			pred_sm.acc = V4d3d((*(cdata_->pva))[2]).norm();
		}
	}
	// node
	else
	{
		if (V4d3d((*(cdata_->pva))[1]).norm() < 0.001)
		{
			pred_la.vel = pred_la.acc = 0.0;
		}
		else
		{
			pred_la.vel = V4d3d((*(cdata_->pva))[1]).norm();
			pred_la.acc = V4d3d((*(cdata_->pva))[2]).norm();
		}
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::PredictFromSectorMap(
		std::shared_ptr<CData> cdata_)
{
	reshapeVector(range_in, cdata_->G->GetNumberOfNodes());

	for (int i = 0; i < cdata_->G->GetNumberOfNodes(); i++)
	{
		pred_sm.range[i] = (int) RANGE::ZERO;
		pred_sm.err[i] = 0.0;
		pred_sm.pct_err[i] = 0.0;
		pred_sm.window[i] = 0.0;
		label2_ap = i;

		if (label1_ap == i)
		{
			continue;
		}
		if (cdata_->G->GetEdgeCounter(label1_ap, i, 0) == 0)
		{
			continue;
		}

		Eigen::Vector3d delta_t;
		int loc_idx	{ -1 }, sec_idx	{ -1 };

		// LOC SEC INT
		int init_tmp = init[i];
		double tmp_dis = this->DecideLocSecInterval(cdata_, delta_t, sec_idx,
				loc_idx, last_loc[i], init_tmp);

		init[i] = init_tmp;
		if (last_loc[i] == 0)
		{
			init[i] = 0;
		}

		// Goal LA
		if (!this->DecideGoal(i,
				cdata_->G->GetEdgeSectorMap(label1_ap, i, 0)[loc_idx
						* cdata_->G->GetSecInt() + sec_idx], delta_t.norm(),
				tmp_dis))
		{
			continue;
		}

		// window
		this->DecideWindow(cdata_->G->GetEdgeSectorMap(label1_ap, i, 0),
				cdata_->G->GetSecInt(), loc_idx, i);
	}

//	// Give priority to range in
//	int sum_tmp = accumulate(range_in.begin(), range_in.end(), 0.0, addFunction);
//	if (sum_tmp>0)
//	{
//		for(int i=0;i<cdata_->G->GetNumberOfNodes();i++)
//		{
//			if (pred_sm.range[i]!=(int)RANGE::IN)
//			{
//				pred_sm.pct_err[i] *= 0.5;
//			}
//		}
//	}

	return EXIT_SUCCESS;
}

double ActionPrediction::DecideLocSecInterval(
		std::shared_ptr<CData> cdata_,
		Eigen::Vector3d &delta_t_,
		int &sec_idx_,
		int &loc_idx_,
		int &loc_last_idx_,
		int &init_)
{
	double tmp_dis, offset, last;

	if (init_ == 0)
	{
		offset = cdata_->G->GetLocInt() * 0.75;
	}
	else
	{
		offset = cdata_->G->GetLocInt() * 0.50;
	}

	last = loc_last_idx_;
	tmp_dis = this->DecideLocationIntervalExt(loc_idx_, loc_last_idx_,
			((*(cdata_->pva))[0] - node_ap.centroid),
			cdata_->G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
			cdata_->G->GetEdge(label1_ap, label2_ap, 0).loc_len,
			cdata_->G->GetEdge(label1_ap, label2_ap, 0).tan,
			(int) round(offset), init_, cdata_->G->GetLocInt());

	if (loc_idx_ > 0)
	{
		init_ = 1;
	}

	// going forward
	if (loc_idx_ - loc_last_idx_ > 0)
	{
		for (int i = loc_last_idx_ + 1; i < loc_idx_ + 1; i++)
		{
			this->DecideSectorIntervalExt(sec_idx_, i, delta_t_,
					((*(cdata_->pva))[0] - node_ap.centroid),
					cdata_->G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
					cdata_->G->GetEdge(label1_ap, label2_ap, 0).tan,
					cdata_->G->GetEdge(label1_ap, label2_ap, 0).nor,
					cdata_->G->GetSecInt());
		}
	}
	// going backward
	else if (loc_idx_ - loc_last_idx_ < 0)
	{
		for (int i = loc_idx_ + 1; i < loc_last_idx_ + 1; i++)
		{
			this->DecideSectorIntervalExt(sec_idx_, i, delta_t_,
					((*(cdata_->pva))[0] - node_ap.centroid),
					cdata_->G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
					cdata_->G->GetEdge(label1_ap, label2_ap, 0).tan,
					cdata_->G->GetEdge(label1_ap, label2_ap, 0).nor,
					cdata_->G->GetSecInt());
		}
	}
	// same loc int
	else
	{
		if (init_ == 1 && last > 0)
		{
			loc_idx_ = last;
		}
		this->DecideSectorIntervalExt(sec_idx_, loc_idx_, delta_t_,
				((*(cdata_->pva))[0] - node_ap.centroid),
				cdata_->G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
				cdata_->G->GetEdge(label1_ap, label2_ap, 0).tan,
				cdata_->G->GetEdge(label1_ap, label2_ap, 0).nor,
				cdata_->G->GetSecInt());
	}

	loc_last_idx_ = loc_idx_;
	return tmp_dis;
}

double ActionPrediction::DecideLocationIntervalExt(
		int &loc_idx_,
		const int &loc_last_idx_,
		const Eigen::Vector4d &point_,
		const std::vector<Eigen::Vector4d> &mid_,
		const std::vector<double> &len_,
		const std::vector<Eigen::Vector3d> &tangent_,
		const int &loc_offset_,
		const int &loc_init_,
		const int &loc_int_)
{
	return dLIPredict(loc_idx_, loc_last_idx_, point_, mid_, len_, tangent_,
			loc_offset_, loc_init_, loc_int_);
}

int ActionPrediction::DecideSectorIntervalExt(
		int &sec_idx_,
		const int &loc_idx_,
		Eigen::Vector3d &delta_t_,
		const Eigen::Vector4d &point_,
		const std::vector<Eigen::Vector4d> &mid_,
		const std::vector<Eigen::Vector3d> &tangent_,
		const std::vector<Eigen::Vector3d> &normal_,
		const int &sec_int_)
{
	return decideSectorInterval(sec_idx_, delta_t_, point_, mid_, tangent_,
			normal_, loc_idx_, sec_int_);
}

bool ActionPrediction::DecideGoal(
		const int &label2_,
		const double &sm_i_, //sector map single
		const double &delta_t_,
		const double &loc_error_)
{
	if (loc_error_ > 0.15) //## TODO :NEED TO VERIFY
	{
		pred_sm.range[label2_] = (int) RANGE::EXCEED;
		pred_sm.err[label2_] = delta_t_ - sm_i_;
		pred_sm.pct_err[label2_] = pdfExp(P_ERR_VAR, 0.0, pred_sm.err[label2_]);
		return false;
	}
	if (delta_t_ <= sm_i_)
	{
		range_in[label2_] = 1;
		pred_sm.range[label2_] = (int) RANGE::IN;
		pred_sm.pct_err[label2_] = 0.999999;
	}
	else
	{
		pred_sm.range[label2_] = (int) RANGE::OUT;
		pred_sm.err[label2_] = delta_t_ - sm_i_;
		pred_sm.pct_err[label2_] = pdfExp(P_ERR_VAR, 0.0, pred_sm.err[label2_]);
	}
	return true;
}

int ActionPrediction::DecideWindow(
		const std::vector<double> &sm_,
		const int &sec_int_,
		const int &loc_idx_,
		const int &label2_)
{
	double max_val = 0.0;
	for (int s = 0; s < sec_int_; s++)
	{
		max_val = std::max(sm_[loc_idx_ * sec_int_ + s], max_val);
	}
	pred_sm.window[label2_] = max_val;
	if (max_val > 0)
	{
		pred_sm.window[label2_] = std::min(pdfExp(P_WIN_VAR, 0, max_val), 1.0);
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::UpdateNode(
		std::shared_ptr<CData> cdata_)
{
	std::vector<double> vel;
	std::vector<double> surface_dist;
	for (int i = 0; i < pred_la_mem.size(); i++)
		for (auto i : pred_la_mem)
		{
			vel.push_back(i.vel);
			surface_dist.push_back(i.surface_dist);
		}

	surface_dist_eval = average(surface_dist);
	vel_eval = pred_la_mem.back().vel;
	label1_eval = label1_ap;

	win_eval.clear();
	pct_eval.clear();
	pct_eval_mem.clear();

	this->UpdateStateNode(*(cdata_->G), *(cdata_->AS));

	return EXIT_SUCCESS;
}

int ActionPrediction::UpdateEdge(
		std::shared_ptr<CData> cdata_)
{
	std::vector<std::vector<double> > err;
	err.resize(cdata_->G->GetNumberOfNodes());
	std::vector<std::vector<double> > win;
	win.resize(cdata_->G->GetNumberOfNodes());
	reshapeVector(pct_eval, cdata_->G->GetNumberOfNodes());
	reshapeVector(win_eval, cdata_->G->GetNumberOfNodes());

	if (pct_eval_mem.empty())
	{
		reshapeVector(pct_eval_mem, cdata_->G->GetNumberOfNodes());
	}

	for (auto pred_sm : pred_sm_mem)
	{
		for (int ii = 0; ii < cdata_->G->GetNumberOfNodes(); ii++)
		{
			win[ii].push_back(pred_sm.window[ii]);
			err[ii].push_back(pred_sm.pct_err[ii]);
		}
	}

	double sum_tmp = 0.0;
	double counter = 0.0;
	bool flag = false;
	for (int i = 0; i < cdata_->G->GetNumberOfNodes(); i++)
	{
		win_eval[i] = average(win[i]);
		pct_eval[i] = average(err[i]);

		if (cdata_->G->GetEdgeCounter(label1_ap, i, 0) > 0)
		{
			counter += 1.0;
		}

		if (last_loc[i] > cdata_->G->GetLocInt() / 10)
		{
			flag = true;
		}

		sum_tmp += pct_eval[i];
	}

	for (int i = 0; i < cdata_->G->GetNumberOfNodes(); i++)
	{
		if (flag)
		{
			if (sum_tmp == 0.0)
			{
				pct_eval[i] = 0.0;
			}
			else
			{
				pct_eval[i] /= sum_tmp;
			}
		}
		else
		{
			if (cdata_->G->GetEdgeCounter(label1_ap, i, 0) > 0)
			{
				pct_eval[i] = 1.0 / counter;
			}
		}
	}

	// Object state evaluation
	if (os_flag)
	{
		this->Prior(cdata_, label1_ap);
		this->Classify(pct_eval);
	}

	vel_eval = pred_sm_mem.back().vel;
	label1_eval = label1_ap;
	surface_dist_eval = 0.0;

	this->UpdateStateEdge(*(cdata_->G), *(cdata_->AS));

	return EXIT_SUCCESS;
}

//int ActionPrediction::RebuildSectorMap(
//	std::vector<std::vector<point_d> > pva_avg_,
//	int	label1_,
//	int label2_)
//{
//	if (pva_avg_.size() < 5) { return EXIT_FAILURE; }
//
//	std::vector<point_d> pts_avg, vel_avg;
//	for(int i=0;i<pva_avg_.size();i++)
//	{
//		pts_avg.push_back(pva_avg_[i][0]);
//		vel_avg.push_back(pva_avg_[i][0]);
//	}
//
//	this->SetLabel1SM(label1_);
//	this->SetLabel2SM(label2_);
//	this->UpdateSectorMap(G, pts_avg);
//
//	//VISUALIZE
//	if(0)
//	{
//		std::vector<point_d> point_zero; std::vector<std::string> label_zero;
//		std::vector<std::vector<unsigned char> > color_code; colorCode(color_code);
//		showConnection(G, pts_avg, label_zero, color_code, true);
//	}
//
//	return EXIT_SUCCESS;
//}
