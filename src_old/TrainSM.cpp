/*
 * TrainSM.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "TrainSM.h"

TrainSM::TrainSM(
		const int &loc_int_,
		const int &sec_int_)
		:	label1_sm(-1),
			label2_sm(-1),
			label_idx_sm(-1),
			loc_int(loc_int_),
			sec_int(sec_int_)
{
}

TrainSM::~TrainSM() {}

//stack<clock_t> tictoc_stack;
//void tic() {tictoc_stack.push(clock());}
//void toc()
//{
//	cout << "Time elapsed: "
//       << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
//       << std::endl;
//	tictoc_stack.pop();
//}

void TrainSM::ClearSM()
{
	label1_sm=label2_sm=label_idx_sm=-1;
	pos_sm.clear();
	pos_ind_sm.clear();
	vel_sm.clear();
}

int TrainSM::FitCurve(
	std::vector<Eigen::Vector4d> &points_est_,
	std::vector<Eigen::Vector3d> &coeffs_)
{
	// Performs curve fitting to obtain the coefficients of the curve.
	std::vector<Eigen::Vector3d> covs;
	reshapeVector(points_est_,(pos_ind_sm.size())*4);
	polyCurveFitPoint(pos_ind_sm, points_est_, coeffs_, covs, DEGREE, true);

	// Visualize by comparing both original and estimated points.
	if(0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		std::vector<Eigen::Vector4d> P = pos_ind_sm;
		P.insert(P.end(),points_est_.begin(),points_est_.end());
		for(int i=0;i<pos_ind_sm.size();i++) { P[i][3] = 1; }
		std::vector<std::string> label_; label_.resize(2);
		std::vector<std::string> label_ref; label_ref.resize(2);
		std::vector<int> loc_idx_zero;
		VTK->ShowData(P, label_, label_ref, loc_idx_zero, color_code, true, false, false);
	}

	printer(18);
	return EXIT_SUCCESS;
}

int TrainSM::DecideSectorIntervalExt(
	std::shared_ptr<CGraph::edge_t> edge_,
	Eigen::Vector4d point_,
	Eigen::Vector3d &delta_t_,
	int &sec_idx_,
	const int &loc_idx_)
{
	return decideSectorInterval(
					sec_idx_, delta_t_, point_, edge_->loc_mid,
					edge_->tan, edge_->nor, loc_idx_, sec_int);
}

double TrainSM::DecideLocationIntervalExt(
	std::shared_ptr<CGraph::edge_t> edge_,
	const Eigen::Vector4d &point_,
	int &loc_idx_,
	const int &loc_last_idx_,
	const int &loc_offset_,
	bool loc_init_)
{
	return dLI(
			loc_idx_, point_, edge_->loc_len, edge_->loc_mid,
			edge_->tan, loc_last_idx_, loc_offset_, loc_int, loc_init_);
}

int TrainSM::AdjustSectorMap(
	std::shared_ptr<CGraph::edge_t> edge_,
	Eigen::Vector4d point_,
	int &loc_last_idx_,
	int &loc_curr_idx_,
	double &delta_t_mem_,
	bool &loc_init_,
	int loc_offset_)
{
	Eigen::Vector3d delta_t, delta_t_zero;
	int sec_idx = -1;
	int loc_idx = -1;
	int loc_last_idx_mem = loc_last_idx_;
	bool mem = loc_init_;
	// to deal with cases where the beginning is out of the sectormap
	// required for the fitting part only
	// also for the ending

	if (this->DecideLocationIntervalExt(edge_, point_,
			loc_idx, loc_last_idx_, loc_offset_, loc_init_) > 0.001)
	{
		if(mem || loc_last_idx_==loc_int-1)
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
		else
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
	}

	// find the delta_t
	this->DecideSectorIntervalExt(
			edge_, point_, delta_t, sec_idx, loc_idx);

	// to fill up the line if loc_int is valid but tangent is not aligned to traj (at a curve)
	double tmpmin = 10.0;
	double tmpdeltatmin = 0.0;
	int min_idx = 0;

	if (loc_idx==loc_last_idx_)
	{
		min_idx			= loc_idx*sec_int + sec_idx;
		tmpdeltatmin	= delta_t.norm();
		tmpmin			=
				V4d3d(point_ -
					  (edge_->loc_mid[loc_idx] *
					   (double)edge_->loc_mid[loc_idx][3])).norm();
	}
	else if (loc_idx>loc_last_idx_)
	{
		for(int ll=loc_last_idx_+1;ll<loc_idx+1;ll++)
		{
			this->DecideSectorIntervalExt(
					edge_, point_, delta_t_zero, sec_idx, ll);
			if (
					min_(
							V4d3d(point_ -
									(edge_->loc_mid[loc_idx] *
									(double)edge_->loc_mid[loc_idx][3])).norm(),
							tmpmin))
			{
				min_idx 		= ll*sec_int + sec_idx;
				tmpdeltatmin	= delta_t_zero.norm();
				tmpmin			=
						V4d3d(point_ -
							  (edge_->loc_mid[loc_idx] *
							   (double)edge_->loc_mid[loc_idx][3])).norm();
			}
		}
	}

	if (loc_init_)
	{
		// if point starts before curve

		Eigen::Vector3d proj_dir_tmp =
				edge_->tan[0] *
				(V4d3d(
						point_ - (
								edge_->loc_mid[0]*
								(double)edge_->loc_mid[0][3]))
						.dot(edge_->tan[0]));
		if (!directionCheck(proj_dir_tmp,edge_->tan[0]))
		{
			loc_init_ = true;
			loc_last_idx_ = 0;
			loc_curr_idx_ = 0;
			return EXIT_SUCCESS;
		}
		else
		{
			loc_init_ = false;
			edge_->sector_map[min_idx] =
					std::max(edge_->sector_map[min_idx], tmpdeltatmin);
//					if (edge_->sector_map[min_idx]>0.3)  {return EXIT_FAILURE;}
			loc_last_idx_ = min_idx/sec_int;
			loc_curr_idx_ = min_idx/sec_int;
			return EXIT_SUCCESS;
		}
	}

	edge_->sector_map[min_idx] = std::max(edge_->sector_map[min_idx], tmpdeltatmin);
//			if (edge_->sector_map[min_idx]>0.3)  {return EXIT_FAILURE;}

	if ((min_idx/sec_int) - loc_last_idx_ > 1)
	{
		for(int l=loc_last_idx_+1;l<(min_idx/sec_int);l++)
		{
			this->DecideSectorIntervalExt(
					edge_, point_, delta_t_zero, sec_idx, l);
			int tmp_idx = l*sec_int + sec_idx;
			edge_->sector_map[tmp_idx] =
					std::max(edge_->sector_map[tmp_idx], delta_t_zero.norm());
		}
		loc_curr_idx_ = (min_idx/sec_int);
	}
	else if ((min_idx/sec_int) - loc_last_idx_ == 1)
	{
		loc_curr_idx_ = (min_idx/sec_int);
	}

	loc_last_idx_ = loc_curr_idx_;

	return EXIT_SUCCESS;
}

int TrainSM::AdjustCurve(
	std::shared_ptr<CGraph::edge_t> edge_,
	Eigen::Vector4d point_,
	int &loc_last_idx_,
	bool &loc_init_,
	int loc_offset_)
{
	Eigen::Vector3d delta_t_zero;
	int sec_idx 			= -1;
	int loc_idx 			= -1;
	int loc_last_idx_mem 	= loc_last_idx_;
	bool mem 				= loc_init_;
	double tmpdlie 			= 0.0;

	tmpdlie =
			this->DecideLocationIntervalExt(
					edge_, point_, loc_idx, loc_last_idx_,
					loc_offset_, loc_init_);

	if (tmpdlie > 0.01)
	{
		if(mem || loc_last_idx_==loc_int-1)
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_FAILURE;
		}
		else
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_FAILURE;
		}
	}

	// to fill up the line if loc_int is valid but tangent is not aligned to traj (at a curve)
	decideSectorInterval(
			sec_idx, delta_t_zero, point_, edge_->loc_mid, edge_->tan,
			edge_->nor, loc_idx, sec_int);
//	this->DecideSectorIntervalExt(
//			edge_, point_, delta_t_zero, sec_idx, loc_idx);

	int tmp_idx = loc_idx*sec_int + sec_idx;
	edge_->sector_map[tmp_idx] =
			std::max(edge_->sector_map[tmp_idx], delta_t_zero.norm());

	if (loc_idx>10)
	{
		for(int l=loc_idx-2;l<loc_idx;l++)
		{
			decideSectorInterval(
					sec_idx, delta_t_zero, point_, edge_->loc_mid,
					edge_->tan, edge_->nor, l, sec_int);
//			this->DecideSectorIntervalExt(
//					edge_, point_, delta_t_zero, sec_idx, l);
			int tmp_idx = l*sec_int + sec_idx;
			edge_->sector_map[tmp_idx] =
					std::max(edge_->sector_map[tmp_idx], delta_t_zero.norm());
		}
	}

	loc_last_idx_ = loc_idx;

	return EXIT_SUCCESS;
}

int TrainSM::AdjustCurveExt(
	std::shared_ptr<CGraph> G_,
	std::vector<Eigen::Vector3d> coeffs_)
{
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double N = G_->GetEdgeCounter(label1_sm,label2_sm,0);
	double total_len = 0.0;
	auto edge_tmp 	  =
			std::make_shared<CGraph::edge_t>(
					G_->GetEdge(label1_sm, label2_sm, 0));
	auto edge_tmp_mem =
			std::make_shared<CGraph::edge_t>(
					G_->GetEdge(label1_sm, label2_sm, 0));

	// [CURVE FIT]*************************************************************
//	polyCurveLength(total_len, 0, integral_limit_, coeffs_);
//	edge_tmp.total_len = total_len;

	for(int i=1;i<pos_ind_sm.size();i++)
	{
		total_len += V4d3d(pos_ind_sm[i]-pos_ind_sm[i-1]).norm();
	}
	edge_tmp->total_len = total_len;

	for(int i=0;i<DEGREE;i++)
	{
		cx[i] = coeffs_[i][0];
		cy[i] = coeffs_[i][1];
		cz[i] = coeffs_[i][2];
	}

	int mem = 0;
	double tmp_len = 0.0;
	int integral_limit_ = 1000;

	for(int i=0;i<loc_int;i++)
	{
		double lim[3], len[3];
		lim[0] = lim[1] = lim[2] = -1.0;
		len[0] =  (total_len/loc_int)* i;
		len[1] = ((total_len/loc_int)* i + (total_len/loc_int)*0.5);
		len[2] =  (total_len/loc_int)*(i+1);
		// ### HACK: resample points along curve and cal length.
		for(int ii=mem;ii<integral_limit_*100+1;ii++)
		{
			double tl = 0.0;
			double t  = (double)ii/100.0;
			polyCurveLength(tl, 0.0, t, coeffs_, DEGREE);
			if      (tl>=len[0] && lim[0]<0.0) {lim[0] = t;}
			else if (tl>=len[1] && lim[1]<0.0) {lim[1] = t;}
			else if (tl>=len[2] && lim[2]<0.0) {lim[2] = t; mem = ii; break;}
		}

		// Checking length values
		if(0)
		{
			double tmplen;
			polyCurveLength(tmplen, 0.0, lim[0], coeffs_, DEGREE);
			printf("length : %.4f %.4f %.4f ", len[0], tmplen, lim[0]);
			polyCurveLength(tmplen, 0.0, lim[1], coeffs_, DEGREE);
			printf("length : %.4f %.4f %.4f ", len[1], tmplen, lim[1]);
			polyCurveLength(tmplen, 0.0, lim[2], coeffs_, DEGREE);
			printf("length : %.4f %.4f %.4f \n", len[2], tmplen, lim[2]);
		}

		if (lim[2]<0) {lim[2] = integral_limit_;}

		Eigen::Vector3d p_tan, p_nor, p_mid;
		p_mid[0] = gsl_poly_eval(cx, DEGREE, lim[1]);
		p_mid[1] = gsl_poly_eval(cy, DEGREE, lim[1]);
		p_mid[2] = gsl_poly_eval(cz, DEGREE, lim[1]);

		// N : counter
		if (N==0)
		{
			// at location interval 0
			if (i==0)
			{
				cal_tangent_normal(p_tan, p_nor, lim[1], coeffs_, true);
				edge_tmp->nor[i] = p_nor.normalized();
				edge_tmp->tan[i] = p_tan.normalized();

			}
			// rotates the normal at location interval 0
			else
			{
				cal_tangent_normal(p_tan, p_nor, lim[1], coeffs_, false);
				edge_tmp->tan [i] = p_tan.normalized();
				Eigen::Matrix3d R = rodriguezRot(edge_tmp->tan[0], edge_tmp->tan[i]);
				edge_tmp->nor[i] = R.inverse().transpose() * edge_tmp->nor[0];
			}
		}
		else
		{
			cal_tangent_normal(p_tan, p_nor, lim[1], coeffs_, false);
			edge_tmp->nor[i] = p_nor.normalized();
			edge_tmp->tan[i] = p_tan.normalized();
		}

		edge_tmp->loc_len[i] =
				((p_mid + (p_tan * (lim[0]-lim[1]))) -
				 (p_mid + (p_tan * (lim[2]-lim[1])))).norm();

		//change origin to LA label1
		p_mid   = p_mid - V4d3d(pos_ind_sm[0]);
		tmp_len = p_mid.norm();
		edge_tmp->loc_mid[i]		= V3d4d(p_mid.normalized());
		edge_tmp->loc_mid[i][3]	= tmp_len;

	}
	// *************************************************************[CURVE FIT]

	// [ADJUSTMENT]************************************************************
	if (N>0)
	{
		CGraph::node_t node_tmp = G_->GetNode(label1_sm);

		// [AVERAGE]***********************************************************
		for(int l=0;l<loc_int;l++)
		{
			edge_tmp->loc_mid[l]	=
					((N/(N+1)) *
							(edge_tmp_mem->loc_mid[l] *
									edge_tmp_mem->loc_mid[l][3] +
										node_tmp.centroid)) +
					((1/(N+1)) *
							(edge_tmp->loc_mid[l] *
									edge_tmp->loc_mid[l][3] +
										node_tmp.centroid)) ;

			edge_tmp->loc_mid[l] = edge_tmp->loc_mid[l] - node_tmp.centroid;
			tmp_len = V4d3d(edge_tmp->loc_mid[l]).norm();
			edge_tmp->loc_mid[l] =
					V3d4d(V4d3d(edge_tmp->loc_mid[l]).normalized());
			edge_tmp->loc_mid[l][3] = tmp_len;

			edge_tmp->loc_len[l] =
					(edge_tmp_mem->loc_len[l]*N/(N+1)) +
					(edge_tmp->loc_len[l]*1/(N+1));

			edge_tmp->tan[l] =
					(edge_tmp_mem->tan[l] * N/(N+1)) +
					(edge_tmp->tan[l] 	 * 1/(N+1));
			edge_tmp->tan[l].normalize();

			// for angles close to 0
			if (edge_tmp_mem->tan[l].dot(edge_tmp->tan[l]) > 0.99)
			{
				edge_tmp->nor[l] = edge_tmp_mem->nor[l];
			}
			else
			{
				auto R = rodriguezRot(edge_tmp_mem->tan[l], edge_tmp->tan[l]);
				edge_tmp->nor[l] = R.inverse().transpose() * edge_tmp_mem->nor[l];
			}
		}
		// ***********************************************************[AVERAGE]

		// [SECTOR MAP]********************************************************
		reshapeVector(edge_tmp->sector_map, loc_int*sec_int);
		std::vector<double> delta_t_mem(3, 0.0);
		Eigen::Vector3d tmpN;
		Eigen::Vector4d p_old;
		Eigen::AngleAxisd aa;
		for(int l=0;l<loc_int;l++)
		{
			for(int s=0;s<sec_int;s++)
			{
				// [OLD POINT]*************************************************
				aa.angle() = 2*M_PI*fmod((s+0.5),(double)sec_int)/sec_int;
				aa.axis() = edge_tmp_mem->tan[l];
				tmpN = rodriguezVec(aa, edge_tmp_mem->nor[l]);
				p_old = (edge_tmp_mem->loc_mid[l]*edge_tmp_mem->loc_mid[l][3]) +
						V3d4d(tmpN * edge_tmp_mem->sector_map[l*sec_int+s]);
				// *************************************************[OLD POINT]
				bool tmp_init = false;
				int last; //##TODO
				if (l<10) 	{ last = 0;		}
				else 		{ last = l-10;	}
				this->AdjustCurve(edge_tmp, p_old, last, tmp_init, 20);
			} //s
		} //l
		// ********************************************************[SECTOR MAP]
	}
	// ************************************************************[ADJUSTMENT]

	G_->SetEdge(label1_sm, label2_sm, 0, *edge_tmp);
	printer(19);

	return EXIT_SUCCESS;
}

int TrainSM::FitSectorMap(
	std::shared_ptr<CGraph::edge_t> edge_,
	Eigen::Vector4d point_,
	int &loc_last_idx_,
	int loc_offset_,
	bool &loc_init_)
{
	int sec_idx = -1, loc_idx = -1, loc_last_idx_mem = loc_last_idx_;
	bool mem_init = loc_init_;
	Eigen::Vector3d delta_t, delta_t_zero;

	double tmp =
			this->DecideLocationIntervalExt(
					edge_, point_, loc_idx, loc_last_idx_, loc_offset_,
					loc_init_);

	// Cases where point is too far out from a location interval.
	if (tmp > 0.01)
	{
		if(mem_init || loc_last_idx_==loc_int-1)
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
		else
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
	}

	// Find delta_t
	this->DecideSectorIntervalExt(edge_, point_, delta_t, sec_idx, loc_idx);

	// To fill up the holes if the location index jumps a few forward.
	double delta_t_min	= delta_t.norm();
	int min_idx			= loc_idx*sec_int + sec_idx;

	if (loc_init_)
	{
		if (loc_idx>0) loc_init_ = false;
		edge_->sector_map[min_idx] =
				std::max(edge_->sector_map[min_idx], delta_t_min);
		if (edge_->sector_map[min_idx]>0.9)  {return EXIT_FAILURE;} // TO BIGGGGGGGGG
		loc_last_idx_ = min_idx/sec_int;
		return EXIT_SUCCESS;
	}

	edge_->sector_map[min_idx] = std::max(edge_->sector_map[min_idx], delta_t_min);
	if (edge_->sector_map[min_idx]>0.9)  {return EXIT_FAILURE;} // TO BIGGGGGGGGG change too big possible for new edge

	if ((min_idx/sec_int) - loc_last_idx_ > 1 && !mem_init)
	{
		for(int l=loc_last_idx_+1;l<(min_idx/sec_int);l++)
		{
			this->DecideSectorIntervalExt(
					edge_, point_, delta_t_zero, sec_idx, l);
			int tmp_idx = l*sec_int + sec_idx;
			edge_->sector_map[tmp_idx] =
					std::max(edge_->sector_map[tmp_idx], delta_t_zero.norm());
			if (edge_->sector_map[min_idx]>0.9)  {return EXIT_FAILURE;} // TO BIGGGGGGGGG
		}
		loc_last_idx_ = (min_idx/sec_int);
	}
	else if ((min_idx/sec_int) - loc_last_idx_ == 1)
	{
		loc_last_idx_ = (min_idx/sec_int);
	}
	//else
	// we do not consider going back for now

	return EXIT_SUCCESS;

}

int TrainSM::FitSectorMapInit(
	std::shared_ptr<CGraph> G_,
	std::vector<Eigen::Vector4d> &points_,
	const int &loc_offset_)
{
	int loc_last_idx = 0, loc_curr_idx = 0, offset = loc_offset_;
	double delta_t_mem = 0.0;
	bool init = true;
	std::size_t idx = 0;

	for(auto p : points_)
	{
		if (idx==0) { continue; } idx++;

		auto edge_tmp =
				std::make_shared<CGraph::edge_t>(
						G_->GetEdge(label1_sm, label2_sm, 0));

		if (init)	{offset = loc_int;}
		else		{offset = loc_offset_;}

		this->AdjustSectorMap(
				edge_tmp, p - G_->GetNode(label1_sm).centroid,
				loc_last_idx, loc_curr_idx, delta_t_mem, init, offset);

		G_->SetEdge(label1_sm, label2_sm, 0, *edge_tmp);
	}
	printer(22);
	return EXIT_SUCCESS;
}

int TrainSM::FitSectorMapExt(
	std::shared_ptr<CGraph> G_,
	const int &loc_offset_)
{
	bool flag_init = true;
	int offset = loc_offset_, loc_last_idx = 0;
	std::size_t idx = 0;

	for(auto pos_ind : pos_ind_sm)
	{
		if (idx==0||idx==pos_ind_sm.size()-2) { continue; } idx++;

		auto edge_tmp =
				std::make_shared<CGraph::edge_t>(
						G_->GetEdge(label1_sm, label2_sm, 0));

		if (flag_init)	{offset = loc_int*3/4;}
		else			{offset = loc_offset_;}

		if (this->FitSectorMap(
				edge_tmp, pos_ind - G_->GetNode(label1_sm).centroid,
				loc_last_idx, offset, flag_init)==EXIT_FAILURE)
		{printer(31); return EXIT_FAILURE;}

		G_->SetEdge(label1_sm, label2_sm, 0, *edge_tmp);

		//VISUALIZE
		if(0)
		{
			auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
			std::vector<std::vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			std::vector<Eigen::Vector4d> point_zero = {pos_ind};
			std::vector<std::string> label_zero;
			VTK->ShowConnection(G_.get(), point_zero, label_zero, color_code, true);
		}
	}

	printer(20);
	return EXIT_SUCCESS;
}

int TrainSM::UpdateSectorMap(
	std::shared_ptr<CGraph> G_)
{
	/**
	 * @Function
	 * fitCurve()				: Fit the curve based on pure trajectory points.
	 *							  Estimate points that are used to do fitting later.
	 * adjustCurveExt()			: Obtain the tan and nor from the estimated points. (only for the first run)
	 * 							  Adjust the tan and nor from the estimated points.
	 * fitSectorMapInit()		: Fit the estimated points to the sector map. (only for the first run)
	 * fitSectorMapExt()		: Fit the estimated points to the sector map.
	 * findSectorMapConstraint(): Check the constraints of  the sector map.
	 * G_.setEdgeCounter()	: Increment the counter.
	 */

	std::vector<Eigen::Vector4d> points_est;
	std::vector<Eigen::Vector3d> coeffs;

	if (G_->GetEdgeCounter(label1_sm,label2_sm,0) == 0)
	{
		this->FitCurve(points_est, coeffs);
		this->AdjustCurveExt(G_, coeffs);
//		this->FitSectorMapInit(G_, points_avg_, loc_int/2);
		if(this->FitSectorMapExt(G_, loc_int/2)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
		G_->SetEdgeCounter(label1_sm, label2_sm, 0, 1);
	}
	else if (G_->GetEdgeCounter(label1_sm,label2_sm,0) < 50)
	{
		this->FitCurve(points_est, coeffs);
		if(this->FitSectorMapExt(G_, loc_int/2)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
		//VISUALIZE
		if(0)
		{
			auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
			std::vector<std::vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			std::vector<std::string> label_zero;
			VTK->ShowConnection(G_.get(), pos_ind_sm, label_zero, color_code, true);
		}
		this->AdjustCurveExt(G_, coeffs);
		//VISUALIZE
		if(0)
		{
			auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
			std::vector<std::vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			std::vector<std::string> label_zero;
			VTK->ShowConnection(G_.get(), pos_ind_sm, label_zero, color_code, true);
		}
		G_->SetEdgeCounter(label1_sm, label2_sm, 0, 1);
	}
	else
	{
		this->FitCurve(points_est, coeffs);
		if(this->FitSectorMapExt(G_, loc_int/2)==EXIT_FAILURE)
		{return EXIT_FAILURE;}
		G_->SetEdgeCounter(label1_sm, label2_sm, 0, 1);
	}

	//VISUALIZE
	if(0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		std::vector<std::string> label_zero;
		VTK->ShowConnection(G_.get(), pos_ind_sm, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}

int TrainSM::BuildSectorMap(
		std::shared_ptr<CGraph> G_,
		std::shared_ptr<CKB> kb_,
		std::shared_ptr<std::vector<std::vector<Eigen::Vector4d> > > pva_avg_,
		std::shared_ptr<std::vector<int> > contact_)
{
	this->ClearSM();

	// 1. Saving point and velocity separately.

	for(auto pva_ind : *pva_avg_)
	{
		pos_sm.push_back(pva_ind[0]);
		vel_sm.push_back(pva_ind[1]);
	}

	// 2. Split the points into blocks according to the LA determined.
	for(int i=0;i<pos_sm.size();i++)
	{
		if (pos_sm[i][3] >= 0)
		{
			// Initial location
			if	(label1_sm < 0)
			{
				label1_sm = pos_sm[i][3];
				continue;
			}
			else
			{
				// Check if location has changed
				if (label_idx_sm > 0)
				{
					label2_sm = pos_sm[i][3];

					// Back to the same position
					if (label2_sm==label1_sm)
					{
						label_idx_sm = -1;
						continue;
					}

					// To prevent only a few points evaluated for curve, might not need this ###
					if (i - label_idx_sm > 10)
					{
						pos_ind_sm.clear();
						pos_ind_sm = {
								pos_sm.begin()+label_idx_sm,
								pos_sm.begin()+i};

						// Adding the LA positions for start and end
						CGraph::node_t node_tmp = {};
						pos_ind_sm.insert(
								pos_ind_sm.begin(),
								G_->GetNode(label1_sm).centroid);
						pos_ind_sm.push_back(
								G_->GetNode(label2_sm).centroid);

						this->UpdateSectorMap(G_);
						this->FindWindowConstraint(G_);

						//VISUALIZE
						if(0)
						{
							auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
							std::vector<std::vector<unsigned char> > color_code;
							VTK->ColorCode(color_code);
							std::vector<std::string> label_zero;
							VTK->ShowConnection(G_.get(), pos_ind_sm, label_zero, color_code, true);
						}

					}

					label1_sm 		= label2_sm;
					label2_sm 		= -1;
					label_idx_sm 	= -1;
				}
			}
		}
		// saves the data number of initial location
		else { if (label1_sm >=0 && label_idx_sm < 0) { label_idx_sm = i; } }
	}
	return EXIT_SUCCESS;
}
