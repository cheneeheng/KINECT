/*
 * Test.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: chen
 */

#include "Test.h"

Test::Test(
		const std::string &obj_,
		const int &loc_int_,
		const int &sec_int_,
		const int &f_win_,
		std::shared_ptr<CKB> KB_,
		std::shared_ptr<COS> OS_,
		std::shared_ptr<std::vector<std::string> > msg_,
		const std::string &path_,
		bool object_state_)
		:	DF(new DataFilter()),
			APred(new ActionPrediction(obj_, loc_int_, sec_int_, KB_, OS_, object_state_)),
			AParse(new ActionParser(obj_, KB_, msg_, path_)),
			RF(new ReadFile()),
			WF(new WriteFile()),
			loc_int(loc_int_),
			sec_int(sec_int_),
			f_win(f_win_)
{
}

Test::~Test()
{
}

int Test::ReadKB(
		const std::string &path_)
{
	if (RF->ReadFileKB(path_, APred->KB)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	else
	{return EXIT_SUCCESS;}
}

int Test::ReadLA(
		const std::string &path_)
{
	if (RF->ReadFileLA(path_, APred->KB->AL(), APred->G)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	else
	{return EXIT_SUCCESS;}
}

int Test::ReadGraph(
		const std::string &path_)
{
	if (RF->ReadFileGraph(path_, APred->G)==EXIT_FAILURE)
	{return EXIT_FAILURE;}
	else
	{return EXIT_SUCCESS;}
}

int Test::SetMessage(std::shared_ptr<std::vector<std::string> > msg_)
{
	AParse->SetMsg(msg_);
	return EXIT_SUCCESS;
}

int Test::SetKB(CKB *kb_)
{
	*APred->KB.get() = *kb_;
	return EXIT_SUCCESS;
}

int Test::SetOS(COS *os_)
{
	*APred->OS = *os_;
	return EXIT_SUCCESS;
}

int Test::WriteWindow(
		const std::string &path_)
{
	WF->WriteFileWindow(APred->G.get(), path_);
	return EXIT_SUCCESS;
}

int Test::ApplyGauss(
		const int &num_x_,
		const int &num_y_)
{
	std::vector<std::vector<double> > k_xy; k_xy.resize(num_x_);
	for(int i=0;i<num_x_;i++) { k_xy[i].resize(num_x_); }
	gaussKernel(k_xy, num_x_, num_x_, 1);

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero; std::vector<std::string> label_zero;
		for(int i=0;i<APred->G->GetNumberOfNodes();i++)
		{ label_zero.push_back(APred->G->GetNode(i).name); }
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(APred->G.get(), point_zero, label_zero, color_code, true);
	}

	for(int i=0;i<APred->G->GetNumberOfNodes();i++)
	{
		for(int ii=0;ii<APred->G->GetNumberOfNodes();ii++)
		{
			if (i==ii) { continue; }

			std::vector<double> sm_tmp1 = APred->G->GetEdgeSectorMap(i, ii, 0);
			std::vector<double> sm_tmp2 = APred->G->GetEdgeSectorMap(i, ii, 0);

			for(int l=0;l<loc_int;l++)
			{
				for(int s=0;s<sec_int;s++)
				{
					double sum_tmp = 0.0;
					for(int gkx=0;gkx<num_x_;gkx++)
					{
						for(int gky=0;gky<num_y_;gky++)
						{
							int tmpl = l-(num_y_/2)+gky;
							if (tmpl < 0 || tmpl>=loc_int) continue;
							int tmps = (s-(num_x_/2)+gkx+sec_int)%sec_int;
							sum_tmp += sm_tmp2[tmpl*sec_int + tmps] * k_xy[gkx][gky];
						}
					}
					sm_tmp1[l*sec_int+s] = sum_tmp;
				}
			}
			APred->G->SetEdgeSectorMap(i,ii,0,sm_tmp1);
		}
	}

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero; std::vector<std::string> label_zero;
		for(int i=0;i<APred->G.get()->GetNumberOfNodes();i++)
		{ label_zero.push_back(APred->G.get()->GetNode(i).name); }
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnectionTest(APred->G.get(), point_zero, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}

int Test::TestInit()
{
	DF->ResetFilter();
	APred->Init(false);
	AParse->Init(3);
	return EXIT_SUCCESS;
}

int Test::TestFaceAdjust(Eigen::Vector4d face_parser)
{
	// When a graph is present.
	if(APred->G.get()->GetNumberOfNodes()>0)
	{
		// Initialize.
		double scale = 1.0;
		Eigen::Vector3d t;
		Eigen::Matrix3d R;
		Eigen::Matrix4d T = Matrix4d::Zero();
		CGraph::node_t old_node;
		CGraph::edge_t edge_tmp;

		// Check for the LA that we intend to change.
		for(int i=0;i<APred->G.get()->GetNumberOfNodes();i++)
		{
			old_node = APred->G.get()->GetNode(i);
			if (old_node.name=="FACE")
			{
				// 1. Change the SM that has the LA as goal.
				//    The start locations stay the same.
				//    Transform to new target goal.
				//    p' = [S]*[R]*p
				for(int ii=0;ii<APred->G.get()->GetNumberOfNodes();ii++)
				{
					edge_tmp = APred->G.get()->GetEdge(ii,i,0);

					if (edge_tmp.counter==0) { continue; }

					scale =
							V4d3d(face_parser		- APred->G.get()->GetNode(ii).centroid).norm() /
							V4d3d(old_node.centroid - APred->G.get()->GetNode(ii).centroid).norm();
					R =
							rodriguezRot(
									V4d3d(old_node.centroid - APred->G.get()->GetNode(ii).centroid),
									V4d3d(face_parser		- APred->G.get()->GetNode(ii).centroid));
					T = Matrix4d::Zero();
					T.block<3,3>(0,0) = R;
					T(3,3) = scale;

					for(int j=0;j<APred->G.get()->GetLocInt();j++)
					{
						edge_tmp.tan[j] = R*edge_tmp.tan[j];
						edge_tmp.nor[j] = R.inverse().transpose()*edge_tmp.nor[j];
						edge_tmp.loc_mid[j] = T*edge_tmp.loc_mid[j];
						edge_tmp.loc_len[j] = edge_tmp.loc_len[j]*scale;
					}

					APred->G.get()->SetEdge(ii,i,0,edge_tmp);
				}

				// 2. Change the SM that has the LA as start.
				//    The goal locations stay the same.
				//    Transform to new origin.
				//    p' = [S]*[R,t]*p
				for(int ii=0;ii<APred->G.get()->GetNumberOfNodes();ii++)
				{
					edge_tmp = APred->G.get()->GetEdge(i,ii,0);

					if (edge_tmp.counter==0) { continue; }

					t = V4d3d(face_parser - old_node.centroid);

					R =
							rodriguezRot(
									V4d3d(APred->G.get()->GetNode(ii).centroid - old_node.centroid),
									V4d3d(APred->G.get()->GetNode(ii).centroid - face_parser		 ));
					t += R*V4d3d(APred->G.get()->GetNode(ii).centroid - old_node.centroid);
					scale =
							V4d3d(APred->G.get()->GetNode(ii).centroid - face_parser).norm() /
							V4d3d(APred->G.get()->GetNode(ii).centroid - old_node.centroid).norm();

					T = Matrix4d::Zero();
					T.block<3,3>(0,0) = R;
					T(3,3) = scale;

					for(int j=0;j<edge_tmp.tan.size();j++)
					{
						edge_tmp.tan[j] = R*edge_tmp.tan[j];
						edge_tmp.nor[j] = R.inverse().transpose()*edge_tmp.nor[j];
						edge_tmp.loc_mid[j] = T*edge_tmp.loc_mid[j];
						edge_tmp.loc_len[j] = edge_tmp.loc_len[j]*scale;
					}

					APred->G.get()->SetEdge(i,ii,0,edge_tmp);
				}

				old_node.centroid.head(3) = V4d3d(face_parser);
				APred->G.get()->SetNode(old_node);
				break;
			}
		}
	}
	return EXIT_SUCCESS;
}

int Test::FilterData(
		const Eigen::Vector4d &pva_in_,
		const int &contact_in_)
{
	DF->PreprocessDataLive(pva_in_, *(APred->pva), f_win);
	DF->PreprocessContactLive(contact_in_, *(APred->contact), f_win);
	return EXIT_SUCCESS;
}

int Test::Predict()
{
	APred->PredictExt();
	return EXIT_SUCCESS;
}

int Test::Parser(
		const std::string &filename_,
		std::string &display_)
{
	AParse->Parse(APred->AS.get());
	AParse->Display(filename_, display_);
	return EXIT_SUCCESS;
}

int Test::GetData(const int &counter)
{
	std::vector<double> tmp;
	tmp.push_back(counter);
	tmp.push_back(APred->AS->Grasp());
	tmp.push_back(APred->AS->Label1());
	tmp.push_back(APred->AS->Label2());
	tmp.push_back(APred->AS->Velocity());
	tmp.push_back(APred->AS->SurfaceFlag());
	tmp.push_back((checkSurfaceDistance((*(APred->pva))[0], APred->KB->SurfaceEquation()[0])));
	tmp.push_back((double)((*(APred->pva))[0][0]));
	tmp.push_back((double)((*(APred->pva))[0][1]));
	tmp.push_back((double)((*(APred->pva))[0][2]));
	data_writeout.push_back(tmp);

	pvas.push_back(*(APred->pva));
	goals.push_back(APred->AS->Goal());
	windows.push_back(APred->AS->Window());

	if (APred->AS->Grasp()==RELEASE)
	{
		labels_predict.push_back("RELEASE");
	}
	else if (APred->AS->Probability()<0)
	{
		labels_predict.push_back(APred->KB.get()->AL()[APred->AS->Label2()]);
	}
	else
	{
		labels_predict.push_back("MOVE");
	}

	return EXIT_SUCCESS;
}

int Test::WriteResult(
		const std::string &filename_,
		const std::string &resultdir_,
		bool nolabel_)
{
	if (nolabel_)
	{
		WF->WriteFilePrediction(
				APred->G.get(), APred->KB.get(), (resultdir_ + filename_),
				labels_predict, goals, windows);
	}
	else
	{
		WF->WriteFilePrediction(
				APred->G.get(), APred->KB.get(), (resultdir_ + filename_),
				labels_parser, labels_predict, goals, windows);
	}
	WF->WriteFile_((resultdir_ + "_" + filename_), data_writeout);
	return EXIT_SUCCESS;
}

int Test::Testing(
	const std::string &filename_,
	const std::string &resultdir_,
		std::string &phrase_,
	bool face_)
{
	// [VARIABLES]**************************************************************
	bool nolabel {false};
	// **************************************************************[VARIABLES]

	// [Initialization] ********************************************************
	this->TestInit();

	std::string tmpname = filename_;
	replace(tmpname.begin(), tmpname.end(), '/', '_');

	printer(1);
	// ******************************************************** [Initialization]

	// [READ FILE]**************************************************************
	if (RF->ReadWord(filename_,',')==EXIT_FAILURE)
	{ return EXIT_FAILURE;	} printer(8);
	// **************************************************************[READ FILE]

	// [PARSE DATA]*************************************************************
	this->ClearParser();
	this->SetDataParser(RF->GetDataWordRF());
//	if (this->ParseData()==EXIT_FAILURE)
//	{
		nolabel = true;
		if (this->ParseDataNoLabel()==EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}
//	}
	printer(9);
	// *************************************************************[PARSE DATA]

	// [FACE ADJUST]************************************************************
	if (face_)
	{
		face_parser[1] *= -1;
		face_parser += Eigen::Vector4d(-0.1, 0, 0.1, -1);
		this->TestFaceAdjust(face_parser);
	}
	// *********************************************************** [FACE ADJUST]

	// [TEST] ******************************************************************
	for(int i=0;i<points_parser.size();i++)
	{
		// 1. Filter
		this->FilterData(points_parser[i], contact_parser[i]);
//		DF->PreprocessDataLive(points_parser[i], *(APred->pva), f_win);
//		DF->PreprocessContactLive(contact_parser[i], *(APred->contact), f_win);

		// 2. Prediction
		this->Predict();

//		if(APred->AS->label1>=0)
		this->GetData(i);

		this->Parser(tmpname, phrase_);

		// Visualize
		if (0)
		{
			auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
			std::vector<Eigen::Vector4d> point_zero; std::vector<std::string> label_zero;
			for(int ii=0;ii<i+1;ii++) point_zero.push_back(pvas[ii][0]);
			std::vector<std::vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			VTK->ShowConnectionTest(APred->G.get(), point_zero, label_zero, color_code, true);
		}

	}

	// writing results
	{
		std::string name_tmp = filename_;
		reverse(name_tmp.begin(),name_tmp.end());
		name_tmp.erase(name_tmp.begin()+name_tmp.find("/"),name_tmp.end());
		reverse(name_tmp.begin(),name_tmp.end());
		this->WriteResult(name_tmp, resultdir_, true);
	}

//	plotData(x,y);

//	std::vector<std::string> title; title.resize(1);
//	plotDatas(title, x, pyy);

	// Visualize
	if(0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<std::string>goal_action;
		for(int i=0;i<APred->G.get()->GetNumberOfNodes();i++)
		{
			goal_action.push_back(APred->G.get()->GetNode(i).name);
		}
		std::vector<Eigen::Vector4d> point_zero; std::vector<std::string> label_zero;
		for(int ii=0;ii<pvas.size();ii++) {point_zero.push_back(pvas[ii][0]);}
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		std::vector<int> 	loc_idx_zero;
		VTK->ShowData(
				point_zero, goal_action, APred->KB.get()->AL(),
				loc_idx_zero, color_code, true, false, false);

		for(int ii=0;ii<pvas.size();ii++)
		{
			if (!strcmp(labels_parser[ii].c_str(),"MOVE"))
				point_zero[ii][3] = -1;
			else if (!strcmp(labels_parser[ii].c_str(),"RELEASE"))
				point_zero[ii][3] = -1;
			else
				point_zero[ii][3] = 1;
		}
		VTK->ShowData(
				point_zero, goal_action, APred->KB.get()->AL(),
				loc_idx_zero, color_code, true, false, false);
	}

	// Visualize
	if (0)
	{
		auto VTK = std::make_shared<VTKExtra>(loc_int, sec_int);
		std::vector<Eigen::Vector4d> point_zero; std::vector<std::string> label_zero;
		for(int i=0;i<APred->G.get()->GetNumberOfNodes();i++)
		{
			label_zero.push_back(APred->G.get()->GetNode(i).name);
		}
		for(int ii=0;ii<pvas.size();ii++) point_zero.push_back(pvas[ii][0]);
		std::vector<std::vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowConnection(APred->G.get(), point_zero, label_zero, color_code, true);
	}

	// ****************************************************************** [TEST]

	return EXIT_SUCCESS;
}

