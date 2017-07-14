//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#include "memory"
#include "TestCase.h"
#include "festival.h"

//=============================================================================
// Global
//=============================================================================

void transitions();
void surfaces();

//=============================================================================
// MAIN
//=============================================================================
int main(
		int argc,
		char *argv[])
{
	printf(
			"==============================================================================\n");
	printf(
			"|| SYSTEM START                                                             ||\n");
	printf(
			"==============================================================================\n");

//	surfaces();

//	transitions();

//std::shared_ptr<TestCase> TC(new TestCase());
	auto TC = std::make_shared<TestCase>();
//	TC->Choose(100);
//	TC->Choose(200);
//	TC->Choose(300);
//	TC->Choose(1000);
//	TC->Choose(2000);
	TC->Choose(3000);

//	// Label
//	TC->Choose(0);

//	// Train Ind.
//	TC->Choose(1);
//	TC->Choose(2);
//	TC->Choose(3);
//	TC->Choose(4);

//	// Test
//	TC->Choose(5);
//	TC->Choose(6);
//	TC->Choose(7);
//	TC->Choose(8);

//	TC->Choose(9);
//	TC->Choose(10);
//	TC->Choose(11);
//	TC->Choose(12);
//
//	TC->Choose(13);
//	TC->Choose(14);
//	TC->Choose(15);
//	TC->Choose(16);

//	TC->Choose(9);
//	TC->Choose(13);

////	TC->Choose(10);
//	TC->Choose(14);
//
////	TC->Choose(11);
//	TC->Choose(15);
//
////	TC->Choose(12);
//	TC->Choose(16);

//	TC->Choose(17);
//	TC->Choose(18);
//	TC->Choose(19);
//	TC->Choose(20);

//	// Train Cup
//	TC.Choose(1, TRN);
//	// Train Org
//	TC.Choose(2, TRN);
//	// Train Spg
//	TC.Choose(3, TRN);
//	// Train Knf
//	TC.Choose(4, TRN);

//	// Train Cup
//	TC.Choose(1, TST);
//	// Train Org
//	TC.Choose(2, TST);
//	// Train Spg
//	TC.Choose(3, TST);
//	// Train Knf
//	TC.Choose(4, TST);

//	TC.Choose(3, TST);

//	TC.Choose(5, DPL);

//	delete TC;

//	{
//		std::map<string, std::vector3d> offset_list;
//		std::vector<std::vector3d> translation;
//		std::vector<std::vector<Matrix3d> > rotation;
//		unique_ptr<ReadFile> RF(new ReadFile());
//		RF->ReadDataset(" ",translation,rotation,offset_list);
//	}

//    EST_Wave wave;
//    int heap_size = 210000;  // default scheme heap size
//    int load_init_files = 1; // we want the festival init files loaded
//
//    festival_initialize(load_init_files,heap_size);
//
//    // Say simple file
//    festival_say_file("/etc/motd");
//
//    festival_eval_command("(voice_ked_diphone)");
//    // Say some text;
//    festival_say_text("hello world");
//
//    // Convert to a waveform
//    festival_text_to_wave("hello world",wave);
//    //wave.save("/tmp/wave.wav","riff");
//
//    // festival_say_file puts the system in async mode so we better
//    // wait for the spooler to reach the last waveform before exiting
//    // This isn't necessary if only festival_say_text is being used (and
//    // your own wave playing stuff)
//    festival_wait_for_spooler();

//	int heap_size = 210000;  // default scheme heap size
//	int load_init_files = 1; // we want the festival init files loaded
//	festival_initialize(load_init_files, heap_size);
//	for (int i = 0; i < 100; i++)
//	{
////		sleep(5);
//		festival_eval_command("(voice_ked_diphone)");
//		festival_say_text("i think you are going to drink");
//	}

	printf(
			"==============================================================================\n");
	printf(
			"|| SYSTEM END                                                               ||\n");
	printf(
			"==============================================================================\n");

	return 0;
}

//=============================================================================
// MISC FUNC
//=============================================================================
template<class T>
void outputMatrix(
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

void surfaces()
{
	ReadFile RF;
	WriteFile WF;
	std::vector<Eigen::Vector3d> bmin, bmid, bmax;
	std::vector<Eigen::Vector4d> peq;
	std::vector<Matrix3d> rot;
	RF.ReadSurfaceFile("Data2/kb/surface_.txt", &rot, &peq, &bmin, &bmid,
			&bmax);
	WF.WriteFileSurface("Data2/kb/surface.txt", rot, peq, bmin, bmid, bmax);
}

void transitions()
{
	std::vector<string> objs = { "CUP", "APP", "SPG" };
	std::map<string, std::vector<std::vector<int> > > transitions1;
	std::map<string, std::vector<std::vector<int> > > transitions2;
	std::map<string, std::vector<std::vector<int> > > transitions3;
	std::map<string, std::vector<std::vector<int> > > transitions4;

	ReadFile RF;
	WriteFile WF;

	for (auto obj : objs)
	{
		auto os = make_shared<COS>();
		auto kb = make_shared<CKB>();
		auto label_list = make_shared<std::map<int, std::vector<string> > >();
		RF.ReadLabelSeq("Data2/kb/", label_list);
		RF.ReadFileOSObjectLabel("Data2/kb/", os);
		RF.ReadFileOSActionObjectState("Data2/kb/", os);
		RF.ReadFileKBActionLabel("Data2/kb/", kb);
		RF.ReadFileOSActionObjectLabelState("Data2/kb/", os);

		auto g1 = kb->AC()["GEOMETRIC"].first;
		auto g2 = kb->AC()["GEOMETRIC"].second;

		int s = g2 - g1 + 1;

		std::vector<std::vector<int> > transition;
		std::vector<int> seq, idxs;

		//			if (obj == "CUP") { idxs = {1}; }
		//			if (obj == "ORG") { idxs = {2}; }
		//			if (obj == "SPG") { idxs = {3,4,5,6,7,8}; }
		//			if (obj == "KNF") { idxs = {9}; }
		if (obj == "CUP")
		{
			idxs = {1,2};
		}
		if (obj == "APP")
		{
			idxs = {3,4};
		}
		if (obj == "SPG")
		{
			idxs = {5,6};
		}

		if (1) // AC()tion
		{
			transition.clear();
			transition.resize(s);
			for (auto &i : transition)
			{
				i.resize(s);
			}
			for (auto i : idxs)
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
			outputMatrix(transition);
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
			for (auto i : idxs)
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
			outputMatrix(transition);
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
			for (auto i : idxs)
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
			outputMatrix(transition);
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
			for (auto i : idxs)
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
			outputMatrix(transition);
			transitions4[obj] = transition;
		}
	}

	WF.WriteOSTransition("Data2/kb/transition_action.txt", transitions1);
	WF.WriteOSTransition("Data2/kb/transition_obj.txt", transitions2);
	WF.WriteOSTransition("Data2/kb/transition_obj_action.txt", transitions3);
	WF.WriteOSTransition("Data2/kb/transition_action_obj.txt", transitions4);
}

