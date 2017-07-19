/*
 * Preprocessing.cpp
 *
 *  Created on: Jul 14, 2017
 *      Author: chen
 */

#include "Preprocessing.h"

Preprocessing::Preprocessing()
{
	// TODO Auto-generated constructor stub

}

Preprocessing::~Preprocessing()
{
	// TODO Auto-generated destructor stub
}

void Preprocessing::Surfaces(
		const std::string &src_,
		const std::string &dst_)
{
	ReadFile RF;
	WriteFile WF;
	std::vector<Eigen::Vector3d> bmin, bmid, bmax;
	std::vector<Eigen::Vector4d> peq;
	std::vector<Eigen::Matrix3d> rot;
	RF.ReadSurfaceFile(src_, &rot, &peq, &bmin, &bmid, &bmax);
	WF.WriteFileSurface(dst_, rot, peq, bmin, bmid, bmax);
}

void Preprocessing::Transitions(
		const std::string &kb_path_,
		const std::vector<std::string> &objs_,
		std::map<std::string, std::vector<int> > idxs_)
{
	std::map<std::string, std::vector<std::vector<int> > > transitions1;
	std::map<std::string, std::vector<std::vector<int> > > transitions2;
	std::map<std::string, std::vector<std::vector<int> > > transitions3;
	std::map<std::string, std::vector<std::vector<int> > > transitions4;

	ReadFile RF;
	WriteFile WF;

	for (auto obj : objs_)
	{
		auto os = std::make_shared<COS>();
		auto kb = std::make_shared<CKB>();
		auto label_list = std::make_shared<
				std::map<int, std::vector<std::string> > >();
		RF.ReadLabelSeq(kb_path_, label_list);
		RF.ReadFileOSObjectLabel(kb_path_, os);
		RF.ReadFileOSActionObjectState(kb_path_, os);
		RF.ReadFileKBActionLabel(kb_path_, kb);
		RF.ReadFileOSActionObjectLabelState(kb_path_, os);

		auto g1 = kb->AC()["GEOMETRIC"].first;
		auto g2 = kb->AC()["GEOMETRIC"].second;

		int s = g2 - g1 + 1;

		std::vector<std::vector<int> > transition;
		std::vector<int> seq;

		if (1) // Action
		{
			transition.clear();
			transition.resize(s);
			for (auto &i : transition)
			{
				i.resize(s);
			}
			for (auto i : idxs_[obj])
			{
				for (auto ii : (*label_list)[i])
				{
					if (ii == "MOVE")
					{
						continue;
					}
					if (ii == "RELEASE")
					{
						if (seq.empty())
						{
							continue;
						}
						for (int iii = 1; iii < seq.size(); iii++)
						{
							transition[seq[iii - 1]][seq[iii]]++;
						}
						seq.clear();
						continue;
					}
					for (int iii = g1; iii < g2 + 1; iii++)
					{
						if (kb->AL()[iii] == ii)
						{
							seq.push_back(iii - g1);
						}
					}
				}
			}
			Output(transition);
			transitions1[obj] = transition;
		}

		if (1) // object
		{
			transition.clear();
			transition.resize(os->OSLabelList().size());
			for (auto &i : transition)
			{
				i.resize(os->OSLabelList().size());
			}
			for (auto i : idxs_[obj])
			{
				for (auto ii : (*label_list)[i])
				{
					if (ii == "MOVE")
					{
						continue;
					}
					if (ii == "RELEASE")
					{
						if (seq.empty())
						{
							continue;
						}
						for (int iii = 1; iii < seq.size(); iii++)
						{
							transition[seq[iii - 1]][seq[iii]] += 1;
						}
						seq.clear();
						continue;
					}
					seq.push_back(os->LAOSMap()[obj][ii]);
				}
			}
			Output(transition);
			transitions2[obj] = transition;
		}

		if (1) // obj_action
		{
			transition.clear();
			transition.resize(os->OSLabelList().size());
			for (auto &i : transition)
			{
				i.resize(s);
			}
			for (auto i : idxs_[obj])
			{
				for (auto ii : (*label_list)[i])
				{
					if (ii == "MOVE" || ii == "RELEASE")
					{
						continue;
					}
					for (int iii = g1; iii < g2 + 1; iii++)
					{
						if (kb->AL()[iii] == ii)
						{
							transition[os->LAOSMap()[obj][ii]][iii - g1] += 1;
						}
					}
				}
			}
			Output(transition);
			transitions3[obj] = transition;
		}

		if (1) // AC()tion_obj
		{
			transition.clear();
			transition.resize(g2 - g1 + 1);
			for (auto &i : transition)
			{
				i.resize(os->OSLabelList().size());
			}
			for (auto i : idxs_[obj])
			{
				for (auto ii : (*label_list)[i])
				{
					if (ii == "MOVE" || ii == "RELEASE")
					{
						continue;
					}
					for (int iii = g1; iii < g2 + 1; iii++)
					{
						if (kb->AL()[iii] == ii)
						{
							transition[iii - g1][os->LAOSMap()[obj][ii]] += 1;
							break;
						}
					}
				}
			}
			Output(transition);
			transitions4[obj] = transition;
		}
	}

	WF.WriteOSTransition(kb_path_ + "transition_action.txt", transitions1);
	WF.WriteOSTransition(kb_path_ + "transition_obj.txt", transitions2);
	WF.WriteOSTransition(kb_path_ + "transition_obj_action.txt", transitions3);
	WF.WriteOSTransition(kb_path_ + "transition_action_obj.txt", transitions4);
}

template<class T>
void Preprocessing::Output(
		std::vector<std::vector<T> > x_)
{
	for (auto i : x_)
	{
		for (auto ii : i)
		{
			printf("%d ", ii);
		}
		printf("\n");
	}
}
