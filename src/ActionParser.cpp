/*
 * ActionParser.cpp
 *
 *  Created on: May 21, 2017
 *      Author: chen
 *      Detail: Parse the action state using a decision tree.
 */

#include "ActionParser.h"

ActionParser::ActionParser(
		const std::string &obj_,
		std::shared_ptr<CData> cdata_,
		const std::string &path_)
		: obj(obj_),
				message(cdata_->msg),
				KB(cdata_->KB),
				delay_factor(-1),
				output(""),
				output_mem(""),
				label(""),
				label_mem(""),
				ob_ac(""),
				grasp_flag(true),
				o("object"),
				l("location"),
				a("action"),
				msg_path(path_),
				message_num(-1, -1)
{
}

ActionParser::~ActionParser()
{
}

int ActionParser::ReadMsg(
		std::string path_)
{
	message->clear();
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
			data_line_ = data_line_ + " " + word;
		}
		message->push_back(data_line_);
	}
	return EXIT_SUCCESS;
}

void ActionParser::Init(
		int delay_)
{
	delay_factor = delay_;
}

std::string ActionParser::Decode(
		std::string obj_,
		std::string loc_,
		std::string msg_)
{
	std::string output = "";

	if (msg_.find(o) != std::string::npos)
	{
		msg_.insert(msg_.find(o), obj_);
		msg_.erase(msg_.find(o), o.length());
	}

	if (msg_.find(l) != std::string::npos)
	{
		msg_.insert(msg_.find(l), loc_);
		msg_.erase(msg_.find(l), l.length());
	}

	if (msg_.find(a) != std::string::npos)
	{
		if (msg_.find(a) - msg_.find("are") == 4)
		{
			if (ob_ac.find(" ") != std::string::npos)
			{
				if (ob_ac[ob_ac.find(" ")-1]=='e')
				{
					ob_ac.erase(ob_ac.find(" ")-1, 1);
				}
				ob_ac.insert(ob_ac.find(" "), "ing");
			}
			else
			{
				ob_ac.insert(ob_ac.length(), "ing");
			}
		}

		if (msg_.find(a) - msg_.find("was") == 4)
		{
			if (ob_ac.find(" ") != std::string::npos)
			{
				ob_ac.erase(ob_ac.find(" "), ob_ac.length() - ob_ac.find(" "));
				if (ob_ac.back()=='e')
				{
					ob_ac.pop_back();
				}
				ob_ac.insert(ob_ac.length(), "ed");
			}
			else
			{
				if (ob_ac.back()=='e')
				{
					ob_ac.pop_back();
				}
				ob_ac.insert(ob_ac.length(), "ed");
			}
		}

		msg_.insert(msg_.find(a), ob_ac);
		msg_.erase(msg_.find(a), a.length());
		msg_ = this->Decode(obj_, loc_, msg_);
	}

	output = msg_;
	return output;
}

void ActionParser::Delay_(
		CAS *s_)
{
	if (state_mem.size() > 0)
	{
		if (s_->Label2() != state_mem.back().Label2())
		{
			state_mem.clear();
		}

		state_mem.push_back(*s_);

		if (state_mem.size() > delay_factor)
		{
			state_mem.erase(state_mem.begin());
		}
	}
	else
	{
		state_mem.push_back(*s_);
	}
}

void ActionParser::Parse(
		CAS *s_)
{
	this->Delay_(s_);
}

void ActionParser::DT0()
{
	if (state_mem.back().Grasp() == GRAB::GRABBED)
	{
		this->DT1();
	}
	else
	{
		if (state_mem[state_mem.size() - 2].Grasp() != GRAB::RELEASE)
		{
			grasp_flag = true;
			output = this->Decode(obj, loc, (*message)[0]);
			message_num.first = 0;
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[1]);
			message_num.first = 1;
		}
		label = "RELEASE";
	}
	if (grasp_flag && (state_mem.back().Label1() != state_mem.back().Label2()))
	{
		grasp_flag = false;
	}
}

// LA / SM
void ActionParser::DT1()
{
	// in LA
	if (state_mem.back().Probability() < 0)
	{
		this->DT2_1();
		label = KB->AL()[state_mem.back().Label2()];
	}
	// in SM
	else
	{
		this->DT2_2();
	}
}

void ActionParser::DT2_1()
{
	if (grasp_flag)
	{
		output = this->Decode(obj, loc, (*message)[2]);
		message_num.first = 2;
	}
	else
	{
		if (state_mem.back().SurfaceFlag() > 0)
		{
			this->DT3_1();
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[5]);
			message_num.first = 5;
		}
	}
}

// surface constraint
void ActionParser::DT3_1()
{
	if (state_mem.back().Velocity() > 0.001)
	{
		output = this->Decode(obj, loc, (*message)[3]);
		message_num.first = 3;
	}
	else
	{
		output = this->Decode(obj, loc, (*message)[4]);
		message_num.first = 4;
	}
}

// SM eval
void ActionParser::DT2_2()
{
	auto vec = KB->TransitionLA()[obj][state_mem.back().Label1()];
	auto sum = std::accumulate(vec.begin(), vec.end(), 0.0);

	if (state_mem.back().Label1() >= 0 && sum == 0.0)
	{
		output = this->Decode(obj, loc, (*message)[11]);
		message_num.first = 11;
		label = "UNKNOWN";
	}
	else
	{
		this->DT3_2();
	}
}

// prediction on SM
void ActionParser::DT3_2()
{
	std::vector<double> tmp, tmp_idx;
	for (int i = KB->AC()["GEOMETRIC"].first;
			i < KB->AC()["GEOMETRIC"].second + 1; i++)
	{
		if (state_mem.back().Goal()[KB->AL()[i]] > 0)
		{
			tmp.push_back(state_mem.back().Goal()[KB->AL()[i]]);
			tmp_idx.push_back(i);
		}
	}

	// only 1 confident traj
	if (tmp.size() == 1)
	{
		this->DT4_1(tmp.back());
	}
	// multiple confident traj
	else if (tmp.size() > 1)
	{
		this->DT4_2(tmp);
	}
	// unknown
	else
	{
		output = this->Decode(obj, loc, (*message)[10]);
		message_num.first = 10;
		label = "UNKNOWN";
	}
}

// only 1 confident traj
void ActionParser::DT4_1(
		double x_)
{
	if (state_mem.back().Velocity() > 0.001)
	{
		output = this->Decode(obj, loc, (*message)[6]);
		message_num.first = 6;
		label = "MOVE";
	}
	else
	{
		output = this->Decode(obj, loc, (*message)[7]);
		message_num.first = 7;
		label = "STOP";
	}
}

// multiple confident traj
void ActionParser::DT4_2(
		std::vector<double> x_)
{
	double sum_tmp = accumulate(x_.begin(), x_.end(), 0.0);
	for (int i = 0; i < x_.size(); i++)
	{
		x_[i] /= sum_tmp;
	}

	// uncertain if probability is less than 0.33 .
	if (*max_element(x_.begin(), x_.end()) > 0.33)
	{
		if (state_mem.back().Velocity() > 0.001)
		{
			output = this->Decode(obj, loc, (*message)[6]);
			message_num.first = 6;
			label = "MOVE";
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[7]);
			message_num.first = 7;
			label = "STOP";
		}
	}
	else
	{
		int idx = distance(x_.begin(), max_element(x_.begin(), x_.end()));
		x_.erase(x_.begin() + idx);
		idx = distance(x_.begin(), max_element(x_.begin(), x_.end()));
		if (state_mem.back().Velocity() > 0.001)
		{
			output = this->Decode(obj, "", (*message)[8]);
			message_num.first = 8;
			output = output + "Perhaps you are going to " + KB->AL()[idx] + ".";
			label = "MOVE";
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[9]);
			message_num.first = 9;
			label = "STOP";
		}
	}
}

void ActionParser::Display(
		const std::string &filename_,
		const int &timestamp_,
		std::string &display_)
{
	if (state_mem.size() >= delay_factor)
	{
		if (state_mem.back().Label2() < 0)
		{
			ob_ac = "";
		}
		else
		{
			ob_ac = (
					KB->OL()[obj][KB->AL()[state_mem.back().Label2()]].empty() ?
							KB->AL()[state_mem.back().Label2()] :
							KB->OL()[obj][KB->AL()[state_mem.back().Label2()]]);
		}

		loc = state_mem.back().SurfaceName();

		this->DT0();
	}

	std::ofstream write_file(msg_path + filename_, std::ios::app);

	// Outputs only when new message is being parsed.
	if (output_mem != output)
	{
		// Checks if the same message is used but with different content.
		if (message_num.first == message_num.second)
		{
			output = "No," + output;
			display_ = output;
			// printf("%s\n", output.c_str());
			write_file << std::to_string(timestamp_) << " " << output << "\n";
			output.erase(0,3);
		}
		else
		{
			display_ = output;
			// printf("%s\n", output.c_str());
			write_file << std::to_string(timestamp_) << " " << output << "\n";
		}
	}
	message_num.second = message_num.first;
	output_mem = output;

	label_mem != label ? label_mem = label : label_mem;

}

