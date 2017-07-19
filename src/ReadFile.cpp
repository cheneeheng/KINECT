/*
 * ReadFile.cpp
 *
 *  Created on: Apr 16, 2017
 *      Author: chen
 *      Detail: Read data from file.
 */

#include "ReadFile.h"

using vvd_t = std::vector<std::vector<double> >;

Eigen::Matrix3d rotMat(
		double angle_,
		int x_)
{
	Eigen::Matrix3d R_;
	switch (x_)
	{
		case 0:
			R_(0, 0) = cos(angle_);
			R_(0, 1) = -sin(angle_);
			R_(0, 2) = 0;
			R_(1, 0) = sin(angle_);
			R_(1, 1) = cos(angle_);
			R_(1, 2) = 0;
			R_(2, 0) = 0;
			R_(2, 1) = 0;
			R_(2, 2) = 1;
			break;
		case 1:
			R_(0, 0) = 1;
			R_(0, 1) = 0;
			R_(0, 2) = 0;
			R_(1, 0) = 0;
			R_(1, 1) = cos(angle_);
			R_(1, 2) = -sin(angle_);
			R_(2, 0) = 0;
			R_(2, 1) = sin(angle_);
			R_(2, 2) = cos(angle_);
			break;
		case 2:
			R_(0, 0) = cos(angle_);
			R_(0, 1) = 0;
			R_(0, 2) = sin(angle_);
			R_(1, 0) = 0;
			R_(1, 1) = 1;
			R_(1, 2) = 0;
			R_(2, 0) = -sin(angle_);
			R_(2, 1) = 0;
			R_(2, 2) = cos(angle_);
			break;
	}
	return R_;
}

ReadFile::ReadFile()
		: list0
		{ },
				list1
				{ },
				list2
				{ },
				data_line_rf
				{ },
				data_word_rf
				{ }
{
	data_line_rf = new std::vector<std::string>;
	data_word_rf = new std::vector<std::vector<std::string> >;
}

ReadFile::~ReadFile()
{
	delete list0;
	delete list1;
	delete list2;
	delete data_line_rf;
	delete data_word_rf;
}

void ReadFile::ClearRF()
{
	list0 = list1 = list2=
	{};
	data_line_rf->clear();
	data_word_rf->clear();
}

int ReadFile::ReadWord(
		const std::string &path_,
		const char &delimiter)
{
	this->ClearRF();
	std::ifstream src_file(path_);
	while (src_file)
	{
		std::string file_line_;
		if (!getline(src_file, file_line_))
			break;
		std::istringstream line_(file_line_);
		std::vector<std::string> data_line_;
		while (line_)
		{
			std::string word;
			if (!getline(line_, word, delimiter))
				break;
			data_line_.push_back(word);
		}
		data_word_rf->push_back(data_line_);
	}
	if (!src_file.eof())
	{
		return EXIT_FAILURE;
	}
	else
	{
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadLine(
		const std::string &path_)
{
	this->ClearRF();
	std::ifstream src_file(path_);
	while (src_file)
	{
		std::string file_line_;
		if (!getline(src_file, file_line_))
			break;
		std::istringstream line_(file_line_);
		std::string data_line_;
		while (line_)
		{
			std::string word;
			if (!getline(line_, word, ' '))
				break;
			if (data_line_.empty())
				data_line_ = word;
			else
				data_line_ += (" " + word);
		}
		data_line_rf->push_back(data_line_);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileName(
		const std::string &dir_name_,
		const std::vector<int> &idx_,
		std::map<int, std::map<int, std::pair<int, std::string> > > *file_list_,
		int &sub_num_)
{
	this->ClearRF();

	std::vector<int> index; // index of action folder
	std::map<int, std::pair<int, std::string> > map_tmp; // file number, action, filename

	// Reading the subject directory
	// Reading the action directory
	// Reading the individual action data file

	sub_num_ = scandir(dir_name_.c_str(), &list0, folderSelect1, alphasort);
	if (sub_num_ == 0)
	{
		printer(32);
		return EXIT_FAILURE;
	}
	else
	{
		printer(33);
	}

	for (int f = 0; f < sub_num_; f++)
	{
		std::string dir_s = dir_name_ + "/" + list0[f]->d_name;

		int nn = scandir(dir_s.c_str(), &list1, folderSelect2, alphasort);
		if (nn == 0)
		{
			printer(25);
			return EXIT_FAILURE;
		}
		else
		{
			printer(26);
		}

		int c = 0;
		map_tmp.clear();
		index.clear();
		index = idx_;

		for (int ff = 0; ff < nn; ff++)
		{
			char num[4];
			sprintf(num, "%03d", index[0]);

			// takes only the action specify by index
			if (strcmp(list1[ff]->d_name, num))
			{
				continue;
			}

			dir_s = dir_name_ + "/" + list0[f]->d_name + "/"
					+ list1[ff]->d_name;

			int nnn = scandir(dir_s.c_str(), &list2, fileSelect, alphasort);
			for (int fff = 0; fff < nnn; fff++)
			{
				//start with 001
				map_tmp[c] = std::make_pair(index[0],
						dir_s + "/" + list2[fff]->d_name);
				c++;
			}

			index.erase(index.begin());
		}

		(*file_list_)[f] = map_tmp;
	}

	return EXIT_SUCCESS;
}

int ReadFile::ReadRefLabelFileName(
		std::string dir_name_,
		std::map<std::string, std::string> &label_list_)
{
	this->ClearRF();
	int n = scandir(dir_name_.c_str(), &list0, fileSelect, alphasort);
	if (n == 0)
	{
		return EXIT_FAILURE;
	}
	else
	{
		for (int f = 0; f < n; f++)
		{
			auto tmp = std::string(list0[f]->d_name);
			tmp.erase(tmp.begin() + (tmp.find(".")), tmp.end());
			label_list_[tmp] = dir_name_ + std::string(list0[f]->d_name);
		}
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadLabelSeq(
		const std::string &path_,
		std::shared_ptr<std::map<int, std::vector<std::string> > > label_)
{
	if (this->ReadWord(path_ + "label_sequence.txt", ',') == EXIT_FAILURE)
	{
		printer(36);
		return EXIT_FAILURE;
	}
	else
	{
		for (auto line : *data_word_rf)
		{
			std::vector<std::string> tmp;
			for (int word = 1; word < line.size(); word++)
			{
				tmp.push_back(line[word]);
			}
			(*label_)[stoi(line[0])] = tmp;
		}
		printer(37);
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadSurfaceFile(
		const std::string &path_,
		std::vector<Eigen::Matrix3d> *rotation_,
		std::vector<Eigen::Vector4d> *planeeq_,
		std::vector<Eigen::Vector3d> *boxmin_,
		std::vector<Eigen::Vector3d> *boxmid_,
		std::vector<Eigen::Vector3d> *boxmax_)
{
	if (this->ReadWord(path_, ',') == EXIT_FAILURE)
	{
		printer(38);
		return EXIT_FAILURE;
	}
	else
	{
		int c = 0, counter = 0;
		double tmp;
		Eigen::Vector4d planeeq_tmp = Eigen::Vector4d::Zero();
		Eigen::Vector3d boxmin_tmp, boxmid_tmp, boxmax_tmp, boxmidrot_tmp,
				y_tmp(0, 1, 0);
		boxmin_tmp = boxmid_tmp = boxmax_tmp = Eigen::Vector3d::Zero();
		for (int i = 0; i < data_word_rf->size(); i++)
		{
			if (c != stoi((*data_word_rf)[i][0]))
			{
				c = stoi((*data_word_rf)[i][0]);
				tmp = planeeq_tmp[3];
				planeeq_tmp = V3d4d(
						V4d3d((planeeq_tmp / counter)).normalized());
				planeeq_tmp[3] = tmp / counter;
				boxmin_tmp = boxmin_tmp / counter;
				boxmax_tmp = boxmax_tmp / counter;
				boxmid_tmp = boxmid_tmp / counter;
				counter = 0;

				Eigen::Matrix3d R = rodriguezRot(y_tmp, V4d3d(planeeq_tmp));
				rotation_->push_back(R);
				boxmidrot_tmp = R * boxmid_tmp;
				boxmin_tmp = R * boxmin_tmp - boxmidrot_tmp;
				boxmax_tmp = R * boxmax_tmp - boxmidrot_tmp;

				for (int xyz = 0; xyz < 3; xyz++)
				{
					tmp = boxmin_tmp[xyz];
					if (boxmax_tmp[xyz] < boxmin_tmp[xyz])
					{
						boxmin_tmp[xyz] = boxmax_tmp[xyz];
						boxmax_tmp[xyz] = tmp;
					}
					tmp = boxmax_tmp[xyz] - boxmin_tmp[xyz];
					boxmin_tmp[xyz] -= tmp / 3.0;
					boxmax_tmp[xyz] += tmp / 3.0;
				}

				planeeq_->push_back(planeeq_tmp);
				boxmin_->push_back(boxmin_tmp);
				boxmid_->push_back(boxmid_tmp);
				boxmax_->push_back(boxmax_tmp);

				planeeq_tmp = Eigen::Vector4d::Zero();
				boxmin_tmp = boxmid_tmp = boxmax_tmp = Eigen::Vector3d::Zero();
			}

			counter++;
			for (int ii = 0; ii < 4; ii++)
				planeeq_tmp[ii] += stod((*data_word_rf)[i][1 + ii]);
			for (int ii = 0; ii < 3; ii++)
				boxmid_tmp[ii] += stod((*data_word_rf)[i][5 + ii]);
			for (int ii = 0; ii < 3; ii++)
				boxmin_tmp[ii] += stod((*data_word_rf)[i][8 + ii]);
			for (int ii = 0; ii < 3; ii++)
				boxmax_tmp[ii] += stod((*data_word_rf)[i][11 + ii]);

			if (i == data_word_rf->size() - 1)
			{
				tmp = planeeq_tmp[3];
				planeeq_tmp = V3d4d(
						V4d3d((planeeq_tmp / counter)).normalized());
				planeeq_tmp[3] = tmp / counter;
				boxmin_tmp = boxmin_tmp / counter;
				boxmax_tmp = boxmax_tmp / counter;
				boxmid_tmp = boxmid_tmp / counter;
				counter = 0;

				Eigen::Matrix3d R = rodriguezRot(y_tmp, V4d3d(planeeq_tmp));
				rotation_->push_back(R);
				boxmidrot_tmp = R * boxmid_tmp;
				boxmin_tmp = R * boxmin_tmp - boxmidrot_tmp;
				boxmax_tmp = R * boxmax_tmp - boxmidrot_tmp;

				for (int xyz = 0; xyz < 3; xyz++)
				{
					tmp = boxmin_tmp[xyz];
					if (boxmax_tmp[xyz] < boxmin_tmp[xyz])
					{
						boxmin_tmp[xyz] = boxmax_tmp[xyz];
						boxmax_tmp[xyz] = tmp;
					}
					tmp = boxmax_tmp[xyz] - boxmin_tmp[xyz];
					boxmin_tmp[xyz] -= tmp / 3.0;
					boxmax_tmp[xyz] += tmp / 3.0;
				}

				planeeq_->push_back(planeeq_tmp);
				boxmin_->push_back(boxmin_tmp);
				boxmid_->push_back(boxmid_tmp);
				boxmax_->push_back(boxmax_tmp);

				planeeq_tmp = Eigen::Vector4d::Zero();
				boxmin_tmp = boxmid_tmp = boxmax_tmp = Eigen::Vector3d::Zero();
			}
		}
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadFileKB(
		const std::string &path_,
		std::shared_ptr<CKB> kb_)
{
	if (this->ReadFileKBSurface(path_, kb_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileKBSurfaceLimit(path_, kb_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileKBActionLabel(path_, kb_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileKBActionObjectLabel(path_, kb_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileKBActionTransition(path_, kb_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

int ReadFile::ReadFileKBSurface(
		const std::string &path_,
		std::shared_ptr<CKB> kb_)
{
	if (this->ReadWord(path_ + "surface.txt", ',') == EXIT_FAILURE)
	{
		printer(38);
		return EXIT_FAILURE;
	}
	else
	{
		auto smin_ = kb_->SurfaceMinPoint();
		auto smid_ = kb_->SurfaceMidPoint();
		auto smax_ = kb_->SurfaceMaxPoint();
		auto seq_ = kb_->SurfaceEquation();
		auto srot_ = kb_->SurfaceRotation();
		smin_.resize(data_word_rf->size());
		smid_.resize(data_word_rf->size());
		smax_.resize(data_word_rf->size());
		seq_.resize(data_word_rf->size());
		srot_.resize(data_word_rf->size());
		for (int i = 0; i < data_word_rf->size(); i++)
		{
			for (int ii = 0; ii < 4; ii++)
				seq_[i][ii] = stod((*data_word_rf)[i][1 + ii]);
			for (int ii = 0; ii < 3; ii++)
				smid_[i][ii] = stod((*data_word_rf)[i][5 + ii]);
			for (int ii = 0; ii < 3; ii++)
				smin_[i][ii] = stod((*data_word_rf)[i][8 + ii]);
			for (int ii = 0; ii < 3; ii++)
				smax_[i][ii] = stod((*data_word_rf)[i][11 + ii]);
			for (int ii = 0; ii < 3; ii++)
				for (int iii = 0; iii < 3; iii++)
					srot_[i](ii, iii) = stod(
							(*data_word_rf)[i][14 + (ii * 3 + iii)]);
		}
		kb_->SurfaceMinPoint(smin_);
		kb_->SurfaceMidPoint(smid_);
		kb_->SurfaceMaxPoint(smax_);
		kb_->SurfaceEquation(seq_);
		kb_->SurfaceRotation(srot_);
		printer(39);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileKBSurfaceLimit(
		const std::string &path_,
		std::shared_ptr<CKB> kb_)
{
	if (this->ReadWord(path_ + "surface_limit.txt", ',') == EXIT_FAILURE)
	{
		printer(45);
		return EXIT_FAILURE;
	}
	else
	{
		auto sl_ = kb_->SurfaceLimit();
		for (auto line : *data_word_rf)
		{
			sl_.push_back(stod(line[1]));
		}
		kb_->SurfaceLimit(sl_);
		printer(46);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileKBActionLabel(
		const std::string &path_,
		std::shared_ptr<CKB> kb_)
{

	if (this->ReadWord(path_ + "label_action.txt", ',') == EXIT_FAILURE)
	{
		printer(3);
		return EXIT_FAILURE;
	}
	else
	{
		auto ac_ = kb_->AC();
		auto al_ = kb_->AL();
		for (auto line : *data_word_rf)
		{
			if (line.size() > 1)
				ac_[line[0]] = std::make_pair(stoi(line[1]), stoi(line[2]));
			else
				al_.push_back(line[0]);
		}
		kb_->AC(ac_);
		kb_->AL(al_);
		printer(2);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileKBActionObjectLabel(
		const std::string &path_,
		std::shared_ptr<CKB> kb_)
{
	if (this->ReadWord(path_ + "action_obj_label.txt", ',') == EXIT_FAILURE)
	{
		printer(5);
		return EXIT_FAILURE;
	}
	else
	{
		auto ol_ = kb_->OL();
		for (auto line : *data_word_rf)
		{
			std::map<std::string, std::string> tmp2;
			for (int ii = 0; ii < (line.size() - 1) / 2; ii++)
			{
				tmp2[line[ii * 2 + 1]] = line[ii * 2 + 2];
			}
			ol_[line[0]] = tmp2;
		}
		kb_->OL(ol_);
		printer(4);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileKBActionTransition(
		const std::string &path_,
		std::shared_ptr<CKB> kb_)
{
	if (this->ReadWord(path_ + "transition_action.txt", ',') == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		auto s = kb_->AC()["GEOMETRIC"].second - kb_->AC()["GEOMETRIC"].first
				+ 1;
		auto ta_ = kb_->TransitionLA();
		for (auto line : *data_word_rf)
		{
			std::vector<double> tmp;
			vvd_t tmp2;
			for (int ii = 1; ii < line.size(); ii++)
			{
				tmp.push_back(stod(line[ii]));
				if (ii % s == 0)
				{
					tmp2.push_back(tmp);
					tmp.clear();
				}
			}
			ta_[line[0]] = tmp2;
		}
		kb_->TransitionLA(ta_);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileLA(
		const std::string &path_,
		const std::vector<std::string> &al_,
		std::shared_ptr<CGraph> Graph_)
{
	if (this->ReadWord(path_, ',') == EXIT_FAILURE)
	{
		printer(7);
		return EXIT_FAILURE;
	}
	else
	{
		CGraph::node_t node_tmp =
		{ };
		for (int i = 0; i < data_word_rf->size(); i++)
		{
			node_tmp = Graph_->GetNode(i);
			if (node_tmp.name.empty())
			{
				node_tmp.name = al_[stod((*data_word_rf)[i][0])];
				node_tmp.index = i;
				for (int ii = 0; ii < 4; ii++)
					node_tmp.centroid[ii] = stod((*data_word_rf)[i][1 + ii]);
				node_tmp.surface_flag = stoi((*data_word_rf)[i][5]);
				for (int ii = 0; ii < 3; ii++)
					node_tmp.cuboid_min[ii] = stod((*data_word_rf)[i][6 + ii]);
				for (int ii = 0; ii < 3; ii++)
					node_tmp.cuboid_max[ii] = stod((*data_word_rf)[i][9 + ii]);
				node_tmp.contact = stoi((*data_word_rf)[i][12]);
				Graph_->SetNode(node_tmp);
				Graph_->addEmptyEdgeForNewNode(i);
			}
			else
			{
				return EXIT_SUCCESS;
			}
		}
		printer(6);
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadFileGraph(
		const std::string &path_,
		std::shared_ptr<CGraph> Graph_)
{
	if (this->ReadWord(path_, ',') == EXIT_FAILURE)
	{
		printer(42);
		return EXIT_FAILURE;
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations = stoi((*data_word_rf)[0][1]);
		num_location_intervals = stoi((*data_word_rf)[1][1]);
		num_sector_intervals = stoi((*data_word_rf)[2][1]);

		int c, l1, l2, l3;
		c = l1 = l2 = l3 = 0;
		for (int i = 3; i < data_word_rf->size(); i++)
		{
			c = (i - 3) % 7;
			CGraph::edge_t edge_tmp = Graph_->GetEdge(l1, l2, l3);
			switch (c)
			{
				case 0:
					l1 = stoi((*data_word_rf)[i][0]);
					l2 = stoi((*data_word_rf)[i][1]);
					l3 = stoi((*data_word_rf)[i][2]);
					break;
				case 1:
					for (int iii = 0; iii < num_location_intervals; iii++)
					{
						for (int iv = 0; iv < 3; iv++)
						{
							edge_tmp.nor[iii][iv] = stod(
									(*data_word_rf)[i][iii * 3 + iv]);
						}
					}
					break;
				case 2:
					for (int iii = 0; iii < num_location_intervals; iii++)
					{
						for (int iv = 0; iv < 3; iv++)
						{
							edge_tmp.tan[iii][iv] = stod(
									(*data_word_rf)[i][iii * 3 + iv]);
						}
					}
					break;
				case 3:
					for (int iii = 0; iii < num_location_intervals; iii++)
					{
						for (int iv = 0; iv < 4; iv++)
						{
							edge_tmp.loc_mid[iii][iv] = stod(
									(*data_word_rf)[i][iii * 4 + iv]);
						}
					}
					break;
				case 4:
					for (int iii = 0; iii < num_location_intervals; iii++)
					{
						edge_tmp.loc_len[iii] = stod((*data_word_rf)[i][iii]);
					}
					break;
				case 5:
					edge_tmp.counter = stoi((*data_word_rf)[i][0]);
					break;
				case 6:
					for (int iii = 0;
							iii < num_location_intervals * num_sector_intervals;
							iii++)
					{
						edge_tmp.sector_map[iii] = stod(
								(*data_word_rf)[i][iii]);
					}
					break;
			}
			Graph_->SetEdge(l1, l2, l3, edge_tmp);
		}
		printer(41);
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadFileOS(
		const std::string &path_,
		std::shared_ptr<COS> os_)
{
	if (this->ReadFileOSObjectLabel(path_, os_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileOSTransition(path_, os_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileOSActionObjectLabelState(path_, os_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileOSActionObjectState(path_, os_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	if (this->ReadFileOSObjectActionState(path_, os_) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

int ReadFile::ReadFileOSObjectLabel(
		const std::string &path_,
		std::shared_ptr<COS> os_)
{
	if (this->ReadLine(path_ + "label_object.txt") == EXIT_FAILURE)
	{
		printer(49);
		return EXIT_FAILURE;
	}
	else
	{
		os_->OSLabelList(*data_line_rf);
		printer(50);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileOSTransition(
		const std::string &path_,
		std::shared_ptr<COS> os_)
{
	if (this->ReadWord(path_ + "transition_obj.txt", ',') == EXIT_FAILURE)
	{
		printer(51);
		return EXIT_FAILURE;
	}
	else
	{
		std::map<std::string, vvd_t> result;
		for (auto line : *data_word_rf)
		{
			int s = os_->OSLabelList().size();
			vvd_t tmp(s, std::vector<double>(s, 0.0));
			for (int word = 1; word < line.size(); word++)
			{
				tmp[(word - 1) / s][(word - 1) % s] = stod(line[word]);
			}
			result[line[0]] = tmp;
		}
		os_->TransitionOS(result);
		printer(52);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileOSActionObjectLabelState(
		const std::string &path_,
		std::shared_ptr<COS> os_)
{
	if (this->ReadWord(path_ + "action_obj_state.txt", ',') == EXIT_FAILURE)
	{
		printer(53);
		return EXIT_FAILURE;
	}
	else
	{
		std::map<std::string, std::map<std::string, int> > tmp;
		for (auto i : *data_word_rf)
		{
			for (int ii = 0; ii < (i.size() - 1) / 2; ii++)
			{
				for (int iii = 0; iii < os_->OSLabelList().size(); iii++)
				{
					if (os_->OSLabelList()[iii] == i[(ii + 1) * 2])
					{
						tmp[i[0]][i[(ii + 1) * 2 - 1]] = iii;
						break;
					}
				}
			}
		}
		os_->LAOSMap(tmp);
		printer(54);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileOSActionObjectState(
		const std::string &path_,
		std::shared_ptr<COS> os_)
{
	if (this->ReadWord(path_ + "transition_action_obj.txt", ',') == EXIT_FAILURE)
	{
		printer(53);
		return EXIT_FAILURE;
	}
	else
	{
		std::map<std::string, vvd_t> result;
		for (auto line : *data_word_rf)
		{
			int s = os_->OSLabelList().size();
			vvd_t tmp(line.size() / s, std::vector<double>(s, 0.0));
			for (int word = 1; word < line.size(); word++)
			{
				tmp[(word - 1) / s][(word - 1) % s] = stod(line[word]);
			}
			result[line[0]] = tmp;
		}
		os_->TransitionLAOS(result);
		printer(54);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileOSObjectActionState(
		const std::string &path_,
		std::shared_ptr<COS> os_)
{
	if (this->ReadWord(path_ + "transition_obj_action.txt", ',') == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		std::map<std::string, vvd_t> result;
		for (auto line : *data_word_rf)
		{
			int s = line.size() / os_->OSLabelList().size();
			vvd_t tmp(os_->OSLabelList().size(), std::vector<double>(s, 0.0));
			for (size_t word = 1; word < line.size(); word++)
			{
				tmp[(word - 1) / s][(word - 1) % s] = stod(line[word]);
			}
			result[line[0]] = tmp;
		}
		os_->TransitionOSLA(result);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadMsg(
		const std::string &path_,
		std::shared_ptr<std::vector<std::string> > message_)
{
	message_->clear();

	if (this->ReadLine(path_ + "message.txt") == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		for (auto i : *data_line_rf)
		{
			message_->push_back(i);
		}
		//for_each(data_line_rf->begin(), data_line_rf->end(), [&](auto *message_){return message_ = *data_line_rf;});
		//memcpy(message_, data_line_rf, sizeof(int)*G->size()); // should be faster
		//message_ = data_line_rf;
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadDataset(
		const std::string &path_,
		std::vector<Eigen::Vector3d> &t_,
		std::vector<std::vector<Eigen::Matrix3d> > &R_,
		std::map<std::string, Eigen::Vector3d> &offset_list_)
{
	bool flag = false;
	bool motion = false;
	std::string word = "";
	std::string word_part = "";
	Eigen::Vector3d offset(0.0, 0.0, 0.0);
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
	std::vector<Eigen::Matrix3d> rotation_ind;

	this->ClearRF();
	this->ReadWord("Dataset/poses.bvh", ' ');
	//this->ReadWord(path_, ' ');

	for (auto line : *data_word_rf)
	{
		for (int i = 0; i < line.size(); i++)
		{
			word = line[i];

			if (motion)
			{
				if (i < 3)
				{
					offset[i] = stod(word);
					if (i == 2)
					{
						t_.push_back(offset);
					}
				}
				else
				{
					rot *= rotMat(stod(word), i % 3);

					if (i % 3 == 2)
					{
						rotation_ind.push_back(rot);
						rot = Eigen::Matrix3d::Identity();
					}

					if (i == line.size() - 1)
					{
						R_.push_back(rotation_ind);
					}
				}
			}
			else
			{
				word.erase(remove(word.begin(), word.end(), '\t'), word.end());

				if (word == "ROOT" || word == "JOINT")
				{
					word_part = line[i + 1];
					flag = true;
					break;
				}

				if (word == "End")
				{
					word_part += ".END";
					flag = false;
					break;
				}

				if (word == "OFFSET" && flag)
				{
					offset[0] = stod(line[i + 1]);
					offset[1] = stod(line[i + 2]);
					offset[2] = stod(line[i + 3]);
					offset_list_[word_part] = offset;
					break;
				}
			}

			if (word == "Frame")
			{
				motion = true;
				break;
			}

		}
	}
	return EXIT_SUCCESS;
}
