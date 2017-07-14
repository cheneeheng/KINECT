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
		std::shared_ptr<CKB> KB,
		std::shared_ptr<std::vector<std::string> > msg_,
		const std::string &path_)
		:	obj(obj_),
		 	message(msg_),
		 	KB(KB),
			delay_factor(-1),
			output(""),
			output_mem(""),
			label(""),
			label_mem(""),
			repeat("i think"),
			ob_ac(""),
			grasp_flag(true),
			start_loc(true),
			o("object"),
			l("location"),
			a("action"),
			msg_path(path_)
{
	message_num = {-1,-1};
}

ActionParser::~ActionParser() {}

int ActionParser::ReadMsg(
		std::string path_)
{
	message->clear();
	std::ifstream src_file(path_);
	while (src_file)
	{
		std::string file_line_;
		if (!getline( src_file, file_line_ )) break;
		std::istringstream line_( file_line_ );
		std::string data_line_;
		while (line_)
		{
		   std::string word;
		   if (!getline( line_, word, ' ')) break;
		   data_line_ = data_line_ + " " + word;
		}
		message->push_back( data_line_ );
	}
	return EXIT_SUCCESS;
}

void ActionParser::Init(
		int delay_)
{
/*
	int heap_size = 210000;  // default scheme heap size
	int load_init_files = 1; // we want the festival init files loaded
	festival_initialize(load_init_files,heap_size);
	festival_eval_command("(voice_ked_diphone)");
*/
	delay_factor = delay_;
}

std::string ActionParser::Decode(
	std::string obj_,
	std::string loc_,
	std::string msg_)
{
	std::string output = "";

	if (msg_.find(o)!=std::string::npos)
	{
		msg_.insert(msg_.find(o),obj_);
		msg_.erase (msg_.find(o),o.length());
	}
	if (msg_.find(l)!=std::string::npos)
	{
		msg_.insert(msg_.find(l),loc_);
		msg_.erase (msg_.find(l),l.length());
	}
	if (msg_.find(a)!=std::string::npos)
	{
		if (msg_.find(a)-msg_.find("are") == 4)
		{
			if (ob_ac.find(" ")!=std::string::npos)
			{
				if (ob_ac[ob_ac.find(" ")-1]=='e') ob_ac.erase(ob_ac.find(" ")-1, 1);
				ob_ac.insert(ob_ac.find(" "),"ing");
			}
			else
			{
				ob_ac.insert(ob_ac.length(),"ing");
			}
		}
		if (msg_.find(a)-msg_.find("was") == 4)
		{
			if (ob_ac.find(" ")!=std::string::npos)
			{
				ob_ac.erase(ob_ac.find(" "),ob_ac.length()-ob_ac.find(" "));
				if (ob_ac.back()=='e') ob_ac.pop_back();
				ob_ac.insert(ob_ac.length(),"ed");
			}
			else
			{
				if (ob_ac.back()=='e') ob_ac.pop_back();
				ob_ac.insert(ob_ac.length(),"ed");
			}
		}
		msg_.insert(msg_.find(a),ob_ac);
		msg_.erase (msg_.find(a),a.length());
		msg_ = this->Decode(obj_,loc_,msg_);

//			if (!strcmp(msg_[i-2].c_str(),"are"))
//				output = output + "ing";
//			else if (!strcmp(msg_[i-2].c_str(),"was"))
//				output = output + "ed";
	}
	output = msg_;
	return output;
}

void ActionParser::Delay_(CAS *s_)
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

//	0. Grasp
//	- no: end
//	- yes: 1.
//
//	1. Position (pct_err)
//	- -1: 2.1. (in LA)
//	- 0: end
//	- xx: 2.2. (in SM)
//
//	2.1. LA
//	- start?
//	-- yes: end
//	-- no: surface?
//	--- yes: 3.1
//	--- no: end
//
//	2.2. SM
//	- yes: 3.2
//	- no: end
//
//	3.1. surface constraint
//	- vel>0 ?
//	-- yes: end
//	-- no: end
//
//	3.2 trajectory number
//	- just 1: 4.1.
//	- multiple: 4.2.
//
//	4.1. 1 trajectory
//	- pct_err > x% ?
//	--yes: end
//	--no: end
//
//	4.2. multiple trajectory
//	- ratio of max > x% ?
//	-- yes: end
//	-- no: ratio of next max - end

// GRASP
void ActionParser::DT0()
{
	if (!state_mem.back().Grasp())
	{
		if (state_mem[state_mem.size()-2].Grasp())
		{
			grasp_flag = true;
			output = this->Decode(obj, loc, (*message)[0]);
			message_num[0] = 0;
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[1]);
			message_num[0] = 1;
		}
		start_loc = true;
		label  = "RELEASE";
	}
	else
	{
		this->DT1();
	}
	if (grasp_flag && (state_mem.back().Label1()!=state_mem.back().Label2()))
	{
		grasp_flag 	= false;
	}
}

// LA / SM
void ActionParser::DT1()
{
	// in LA
	if (state_mem.back().Probability()<0)
	{
		this->DT2_1();
		label  = KB->AL()[state_mem.back().Label2()];
	}
	// in SM
	else
	{
		start_loc = false;
		this->DT2_2();
	}
}

// LA start loc?
void ActionParser::DT2_1()
{
	if(grasp_flag)
	{
		output = this->Decode(obj, loc, (*message)[2]);
		message_num[0] = 2;
	}
	else
	{
		if(state_mem.back().SurfaceFlag()>0)
		{
			this->DT3_1();
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[5]);
			message_num[0] = 5;
		}
	}
}

// surface constraint
void ActionParser::DT3_1()
{
	if(state_mem.back().Velocity()>0.001)
	{
		output = this->Decode(obj, loc, (*message)[3]);
		message_num[0] = 3;
	}
	else
	{
		output = this->Decode(obj, loc, (*message)[4]);
		message_num[0] = 4;
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
		message_num[0] = 11;
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
	for(int i=KB->AC()["GEOMETRIC"].first;i<KB->AC()["GEOMETRIC"].second+1;i++)
	{
		if(state_mem.back().Goal()[KB->AL()[i]]>0)
		{
			tmp.push_back(state_mem.back().Goal()[KB->AL()[i]]);
			tmp_idx.push_back(i);
		}
	}

	// only 1 confident traj
	if(tmp.size()==1)
	{
		this->DT4_1(tmp.back());
	}
	// multiple confident traj
	else if(tmp.size()>1)
	{
		this->DT4_2(tmp);
	}
	// unknown
	else
	{
		output = this->Decode(obj, loc, (*message)[10]);
		message_num[0] = 10;
		label  = "UNKNOWN";
	}
}

// only 1 confident traj
void ActionParser::DT4_1(
	double x_)
{
	if(state_mem.back().Velocity()>0.001)
	{
		output = this->Decode(obj, loc, (*message)[6]);
		message_num[0] = 6;
		label  = "MOVE";
	}
	else
	{
		output = this->Decode(obj, loc, (*message)[7]);
		message_num[0] = 7;
		label  = "STOP";
	}
}

// multiple confident traj
void ActionParser::DT4_2(
	std::vector<double> x_)
{
	double sum_tmp = accumulate(x_.begin(), x_.end(), 0.0);
	for(int i=0;i<x_.size();i++)
	{
		x_[i] /= sum_tmp;
	}

	if(*max_element(x_.begin(), x_.end()) > 0.33)
	{
		if(state_mem.back().Velocity()>0.001)
		{
			output = this->Decode(obj, loc, (*message)[6]);
			message_num[0] = 6;
			label  = "MOVE";
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[7]);
			message_num[0] = 7;
			label  = "STOP";
		}
	}
	else
	{
		int idx =
					distance(
							x_.begin(),
							max_element(x_.begin(), x_.end()));
		x_.erase(x_.begin()+idx);
		idx =
					distance(
							x_.begin(),
							max_element(x_.begin(), x_.end()));
		if(state_mem.back().Velocity()>0.001)
		{
			output = this->Decode(obj, "", (*message)[8]);
			message_num[0] = 8;
			output = output + "Perhaps you are going to " + KB->AL()[idx] + ".";
			label  = "MOVE";
		}
		else
		{
			output = this->Decode(obj, loc, (*message)[9]);
			message_num[0] = 9;
			label  = "STOP";
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
		if(state_mem.back().Label2() < 0)
		{
			ob_ac = "";
		}
		else
		{
			ob_ac =
					(KB->OL()[obj][KB->AL()[state_mem.back().Label2()]].empty() ?
							KB->AL()[state_mem.back().Label2()] :
							KB->OL()[obj][KB->AL()[state_mem.back().Label2()]]);
		}

		loc = state_mem.back().SurfaceName();

		this->DT0();
	}

	std::ofstream write_file(msg_path + filename_, std::ios::app);

	// Extra info for transitions
	if (output_mem!=output)
	{
//		if ((output_mem.find(repeat)!=std::string::npos) && (output.find(repeat)!=std::string::npos))
//		{
//			output = "No," + output;
//		}
//		if (!strncmp(output_mem.c_str(),repeat.c_str(),strlen(repeat.c_str())) &&
//			!strncmp(output.c_str(),repeat.c_str(),strlen(repeat.c_str())))
		if (message_num[0]==message_num[1])
		{
			output = "No," + output;
			display_ = output;
			//if (display_)
			//{
			//	printf("%s\n", output.c_str());
				//festival_say_text(output.c_str());
			//}
			write_file << std::to_string(timestamp_) << " " << output << "\n";
			output.erase(0,3);
		}
		else
		{
			display_ = output;
			//if (display_)
			//{
			//	printf("%s\n", output.c_str());
				//festival_say_text(output.c_str());
			//}
			write_file << std::to_string(timestamp_) << " " << output << "\n";
		}
	}
	message_num[1] = message_num[0];
	output_mem = output;

	label_mem!=label ? label_mem = label : label_mem;

}

