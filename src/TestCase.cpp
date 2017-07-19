/*
 * TestCase.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: chen
 */

#include "TestCase.h"

//=============================================================================
// CONSTUCTOR / DESTRUCTOR
//=============================================================================

TestCase::TestCase()
		: sub_num(0),
				file_list(
						new std::map<int,
								std::map<int, std::pair<int, std::string> > >),
				label_list(new std::map<int, std::vector<std::string> >),
				RF(new ReadFile),
				WF(new WriteFile)
{

#ifdef PC
	EVAL = "../Scene1";
	EVAL2 = "../Scene2";
	RESULT = "../Result";
	RESULT2 = "../Result2";
	KB_DIR = "../kb";
	DATA_DIR = "../recording";
#else
	EVAL = "Scene1";
	EVAL2 = "Scene2";
	RESULT = "Result";
	RESULT2 = "Result2";
	KB_DIR = "kb";
	DATA_DIR = "recording";
#endif

	dict[1] = "CUP";
#ifdef DATA1
	dict[2] = "ORG";
#elif DATA2
	dict[2] = "APP";
#else
	dict[2] = "ORG";
#endif
	dict[3] = "SPG";
	dict[4] = "KNF";
	dict[5] = "PT1";
	dict[6] = "PT2";

	printf(
			"==============================================================================\n");
	printf(
			"|| TEST CASE INITIALIZED                                                    ||\n");
	printf(
			"==============================================================================\n");
}

TestCase::~TestCase()
{
	printf(
			"==============================================================================\n");
	printf(
			"|| TEST CASE ENDED                                                          ||\n");
	printf(
			"==============================================================================\n");
}

//=============================================================================
// CHOOSING
//=============================================================================

void TestCase::Choose(
		int x_)
{
	std::vector<int> idxs;

	switch (x_)
	{
		case 0:
		{
			idxs =
			{	1,2,3,4,5,6,7,8,9,10,11};
			this->Lbl(idxs);
			break;
		}
		case 100:
		{
			PARENT = "Data1";
			KB_DIR = "kb";
			DATA_DIR = "recording";
			EVAL = "Scene_Staticface";
			RESULT = "Result_Staticface";
			{
				idxs.clear(); idxs =
				{	1};
				this->Trn(idxs, dict[1], false);
				this->Tst(idxs, dict[1], true, false, false);
				idxs.clear(); idxs =
				{	2};
				this->Trn(idxs, dict[2], false);
				this->Tst(idxs, dict[2], true, false, false);
				idxs.clear(); idxs =
				{	3,4,5,6,7,8};
				this->Trn(idxs, dict[3], false);
				this->Tst(idxs, dict[3], true, false, false);
				idxs.clear(); idxs =
				{	9};
				this->Trn(idxs, dict[4], false);
				this->Tst(idxs, dict[4], true, false, false);
			}
			break;
		}
		case 200:
		{
			PARENT = "Data1";
			KB_DIR = "kb";
			DATA_DIR = "recording";
			EVAL = "Scene_Moveface";
			RESULT = "Result_Moveface";
			{
				idxs.clear(); idxs =
				{	1};
				this->Trn(idxs, dict[1], true);
				this->Tst(idxs, dict[1], true, true, false);
				idxs.clear(); idxs =
				{	2};
				this->Trn(idxs, dict[2], true);
				this->Tst(idxs, dict[2], true, true, false);
				idxs.clear(); idxs =
				{	3,4,5,6,7,8};
				this->Trn(idxs, dict[3], true);
				this->Tst(idxs, dict[3], true, true, false);
				idxs.clear(); idxs =
				{	9};
				this->Trn(idxs, dict[4], true);
				this->Tst(idxs, dict[4], true, true, false);
			}
			break;
		}
		case 300:
		{
			PARENT = "Data1";
			KB_DIR = "kb";
			DATA_DIR = "recording";
			EVAL = "Scene_Moveface";
			RESULT = "Result_ObjectState";
			{
				idxs.clear(); idxs =
				{	1};
				this->Tst(idxs, dict[1], true, true, true);
				idxs.clear(); idxs =
				{	2};
				this->Tst(idxs, dict[2], true, true, true);
				idxs.clear(); idxs =
				{	3,4,5,6,7,8};
				this->Tst(idxs, dict[3], true, true, true);
				idxs.clear(); idxs =
				{	9};
				this->Tst(idxs, dict[4], true, true, true);
			}
			break;
		}
		case 1000:
		{
			PARENT = "Data2";
			KB_DIR = "kb";
			DATA_DIR = "recording";
			EVAL = "Scene_Staticface";
			RESULT = "Result_Staticface";
			{
				idxs.clear(); idxs =
				{	1,2};
				this->TrnInd(idxs, dict[1], false);
				this->Tst(idxs, dict[1], true, false, false);
				idxs.clear(); idxs =
				{	3,4};
				this->TrnInd(idxs, dict[2], false);
				this->Tst(idxs, dict[2], true, false, false);
				idxs.clear(); idxs =
				{	5,6};
				this->TrnInd(idxs, dict[3], false);
				this->Tst(idxs, dict[3], true, false, false);
			}
			break;
		}
		case 2000:
		{
			PARENT = "Data2";
			KB_DIR = "kb";
			DATA_DIR = "recording";
			EVAL = "Scene_Moveface";
			RESULT = "Result_Moveface";
			{
				idxs.clear(); idxs =
				{	1,2};
				this->TrnInd(idxs, dict[1], true);
				this->Tst(idxs, dict[1], true, true, false);
				idxs.clear(); idxs =
				{	3,4};
				this->TrnInd(idxs, dict[2], true);
				this->Tst(idxs, dict[2], true, true, false);
				idxs.clear(); idxs =
				{	5,6};
				this->TrnInd(idxs, dict[3], true);
				this->Tst(idxs, dict[3], true, true, false);
			}
			break;
		}
		case 3000:
		{
			PARENT = "Data2";
			KB_DIR = "kb";
			DATA_DIR = "recording";
			EVAL = "Scene_Moveface";
			RESULT = "Result_ObjectState";
			{
				idxs.clear(); idxs =
				{	1,2};
				this->Tst(idxs, dict[1], true, true, true);
				idxs.clear(); idxs =
				{	3,4};
				this->Tst(idxs, dict[2], true, true, true);
				idxs.clear(); idxs =
				{	5,6};
				this->Tst(idxs, dict[3], true, true, true);
			}
			break;
		}
		default:
		{
			break;
		}
	}
}

//=============================================================================
// TEST CASES
//=============================================================================

int TestCase::TrnInd(
		std::vector<int> idx_,
		std::string object_,
		bool face_)
{
	bool flag;
	std::string dir_s, path;
	std::pair<int, std::string> pair_tmp(-1, "");

	auto cdata = std::make_shared<CData>(object_, LOC_INT, SEC_INT);

	printf(
			"==============================================================================\n");
	printf(
			"|| TrnInd   START                                                           ||\n");
	printf(
			"==============================================================================\n");

	this->ReadFileExt(idx_);

	// Subject
	for (int f = 0; f < sub_num; f++)
	{
		// construct Train class object.
		auto T = std::make_shared<Train>(LOC_INT, SEC_INT, FILTER_WIN);

		// create a data container.
		auto cdata = std::make_shared<CData>(object_, LOC_INT, SEC_INT);

		// read saved information.
		this->ReadFileCData(cdata, object_, false);

		// to check if a LA has already been seen.
		std::vector<int> al_tmp_idx(cdata->KB->AL().size(), 0);

		for (int ff = 0; ff < sub_num; ff++)
		{
			// Data per subject
			for (int fff = 0; fff < (*file_list)[ff].size(); fff++)
			{
				flag = false;

				// create directory if not valid
				dir_s = PARENT + "/" + EVAL + "/" + object_ + "/"
						+ std::to_string(f) + "/";
				directoryCheck(dir_s);

				// read available location areas
				path = dir_s + "location_area.txt";
				RF->ReadFileLA(path, cdata->KB->AL(), cdata->G);

				// action filename std::pair
				pair_tmp = (*file_list)[ff][fff];

				// for initial labelling
				if (1)
				{
					for (int i = 0; i < cdata->KB->AL().size(); i++)
					{
						if (al_tmp_idx[i] > 0)
							continue;
						for (int ii = 0;
								ii < (*label_list)[pair_tmp.first].size(); ii++)
						{
							if (!strcmp(cdata->KB->AL()[i].c_str(),
									(*label_list)[pair_tmp.first][ii].c_str()))
							{
								flag = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				// learning process
				if (T->Learning(cdata->G, cdata->KB, pair_tmp.second, flag,
						face_) == EXIT_FAILURE)
				{
					printer(29);
					return EXIT_FAILURE;
				}
				printer(30);

				// writing location areas data
				if (cdata->G->GetNumberOfNodes() > 0)
				{
					remove(path.c_str());
					WF->WriteFileLA(cdata->G.get(), cdata->KB.get(),
							path.c_str());
				}
			}
		}

		path = PARENT + "/" + EVAL + "/" + object_ + "/" + std::to_string(f)
				+ "/graph.txt";
		WF->WriteFileGraph(cdata->G.get(), path);
		path = PARENT + "/" + EVAL + "/" + object_ + "/" + std::to_string(f)
				+ "/window.txt";
		WF->WriteFileWindow(cdata->G.get(), path);

	}

	printf(
			"==============================================================================\n");
	printf(
			"|| TrnInd  END                                                              ||\n");
	printf(
			"==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Trn(
		std::vector<int> idx_,
		std::string object_,
		bool face_)
{
	bool flag_new_label;
	std::string dir_s, path;
	std::pair<int, std::string> pair_tmp(-1, "");

	auto cdata = std::make_shared<CData>(object_, LOC_INT, SEC_INT);

	printf(
			"==============================================================================\n");
	printf(
			"|| Trn START                                                                ||\n");
	printf(
			"==============================================================================\n");

	this->ReadFileExt(idx_);

	// Subject
	for (int f = 0; f < sub_num; f++)
	{
		// construct Train class object.
		auto T = std::make_shared<Train>(LOC_INT, SEC_INT, FILTER_WIN);

		// create a data container.
		auto cdata = std::make_shared<CData>(object_, LOC_INT, SEC_INT);

		// read saved information.
		this->ReadFileCData(cdata, object_, false);

		// to check if a LA has already been seen.
		std::vector<int> al_tmp_idx(cdata->KB->AL().size(), 0);

		for (int ff = 0; ff < sub_num; ff++)
		{
			if (f == ff)
			{
				continue;
			}

			// Data per subject
			for (int fff = 0; fff < (*file_list)[ff].size(); fff++)
			{
				flag_new_label = false;

				// create directory if not valid
				dir_s = PARENT + "/" + EVAL + "/" + object_ + "/"
						+ std::to_string(f) + "/";
				directoryCheck(dir_s);

				// read available location areas
				path = dir_s + "location_area.txt";
				RF->ReadFileLA(path, cdata->KB->AL(), cdata->G);

				// action filename std::pair
				pair_tmp = (*file_list)[ff][fff];

				// for initial labelling
				if (1)
				{
					for (int i = 0; i < cdata->KB->AL().size(); i++)
					{
						if (al_tmp_idx[i] > 0)
							continue;
						for (auto label : (*label_list)[pair_tmp.first])
						{
							if (cdata->KB->AL()[i] == label)
							{
								flag_new_label = true;
								al_tmp_idx[i] = 1;
							}
						}
					}
				}

				// learning process
				if (T->Learning(cdata->G, cdata->KB, pair_tmp.second,
						flag_new_label, face_) == EXIT_FAILURE)
				{
					printer(29);
					return EXIT_FAILURE;
				}
				printer(30);

				// writing location areas data
				if (cdata->G->GetNumberOfNodes() > 0)
				{
					remove(path.c_str());
					WF->WriteFileLA(cdata->G.get(), cdata->KB.get(),
							path.c_str());
				}
			}
		}

		path = PARENT + "/" + EVAL + "/" + object_ + "/" + std::to_string(f)
				+ "/graph.txt";
		WF->WriteFileGraph(cdata->G.get(), path);
		path = PARENT + "/" + EVAL + "/" + object_ + "/" + std::to_string(f)
				+ "/window.txt";
		WF->WriteFileWindow(cdata->G.get(), path);

	}

	printf(
			"==============================================================================\n");
	printf(
			"|| Trn END                                                                  ||\n");
	printf(
			"==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Tst(
		std::vector<int> idx_,
		std::string object_,
		bool gauss_,
		bool face_,
		bool os_)
{
	printf(
			"==============================================================================\n");
	printf(
			"|| TST START                                                                ||\n");
	printf(
			"==============================================================================\n");

	std::string dir_s, path;

	this->ReadFileExt(idx_);

	// Subject
	for (int f = 0; f < sub_num; f++)
	{
		// create a data container
		auto cdata = std::make_shared<CData>(object_, LOC_INT, SEC_INT);

		// read saved information
		this->ReadFileCData(cdata, object_, os_);

		// directory of learned data of a subject
		dir_s = PARENT + "/" + EVAL + "/" + object_ + "/" + std::to_string(f);

		// read available location areas
		path = dir_s + "/location_area.txt";
		if (RF->ReadFileLA(path, cdata->KB->AL(), cdata->G) == EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}

		// read available sector-map
		path = dir_s + "/graph.txt";
		if (RF->ReadFileGraph(path, cdata->G) == EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}

		// directory to parsed message
		dir_s = PARENT + "/" + RESULT + "/" + object_ + "/" + std::to_string(f)
				+ "/ParsedResult/";
		directoryCheck(dir_s);

		auto T = std::make_shared<Test>(object_, LOC_INT, SEC_INT, FILTER_WIN,
				cdata, dir_s, os_);

		// apply gauss filter
		if (gauss_)
		{
			T->ApplyGauss(5, 5);
		}

		// write window constraint
		path = dir_s + "/window.txt";
		if (T->WriteWindow(path) == EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}

		dir_s = PARENT + "/" + RESULT + "/" + object_ + "/" + std::to_string(f)
				+ "/";
		directoryCheck(dir_s);

		// Data per subject
		//for(int ff=0;ff<(*file_list)[f].size();ff++)
		for (auto ff : (*file_list)[f])
		{
			// action filename std::pair
			// pair_tmp = file_list[ff][fff];

			char num[4];
			sprintf(num, "%03d", ff.second.first);

//			if(strcmp(num,"006")) continue;
//			if(ff.second.second!="recording/02_Bogdan/001/170419114329.txt") continue;

			path = dir_s + std::string(num) + "/";
			directoryCheck(path);

			if (T->Testing(ff.second.second, path, face_) == EXIT_FAILURE)
			{
				printer(43);
				return EXIT_FAILURE;
			}
			else
			{
				printer(44);
			}
		}
	}

	printf(
			"==============================================================================\n");
	printf(
			"|| TST END                                                                  ||\n");
	printf(
			"==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::Lbl(
		std::vector<int> idx_)
{

	int n, b, e;
	n = b = e = 0;
	std::string path;

	std::map<std::string, std::string> label_ref_list;
	std::vector<Eigen::Vector3d> labels_ref;
	std::vector<std::string> labels_ref_name;

	DataParser P;

	printf(
			"==============================================================================\n");
	printf(
			"|| LABELLING START                                                          ||\n");
	printf(
			"==============================================================================\n");

	// Reading label_list
	path = PARENT + "/" + KB_DIR + "/";
	if (RF->ReadLabelSeq(path, label_list) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	// Reading file names for location references of the labels
	path = PARENT + "/label/";
	if (RF->ReadRefLabelFileName(path, label_ref_list) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	// Parsing the data from the reference files
	typedef std::map<std::string, std::string>::iterator it_type;
	for (it_type itr = label_ref_list.begin(); itr != label_ref_list.end();
			itr++)
	{
		if (RF->ReadWord(itr->second, ',') == EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}
		printer(47);

		b = e;
		e = b + RF->GetDataWordRF().size();
		P.SetDataParser(RF->GetDataWordRF());
		P.ParseDataNoLabel();
		std::vector<Eigen::Vector4d> tmp = P.GetPointParser();
		std::vector<Eigen::Vector3d> tmp2;
		tmp2.resize(tmp.size());
		for (int i = 0; i < tmp.size(); i++)
			tmp2[i] = V4d3d(tmp[i]);
		labels_ref.insert(labels_ref.end(), tmp2.begin(), tmp2.end());
		for (int i = b; i < e; i++)
		{
			labels_ref_name.push_back(itr->first);
		}
	}

	// Reading data file name
	if (RF->ReadFileName(std::string(PARENT + "/" + DATA_DIR), idx_,
			file_list.get(), n) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	printer(26);

	// Subject
	for (int f = 0; f < n; f++)
	{
		// Data per subject
		for (int ff = 0; ff < (*file_list)[f].size(); ff++)
		{
			if (RF->ReadWord((*file_list)[f][ff].second, ',') == EXIT_FAILURE)
			{
				return EXIT_FAILURE;
			}
			printer(8);

			P.SetDataParser(RF->GetDataWordRF());
			P.ParseDataNoLabel();
			WF->RewriteDataFile((*file_list)[f][ff].second, RF->GetDataWordRF(),
					P.GetPointParser(), P.GetContactParser(), P.GetFaceParser(),
					labels_ref, labels_ref_name,
					(*label_list)[(*file_list)[f][ff].first]);
			printer(48);

			// Visualize
			if (0)
			{
				VTKExtra *VTK = new VTKExtra(LOC_INT, SEC_INT);
				std::vector<std::vector<unsigned char> > color_code;
				VTK->ColorCode(color_code);
				std::vector<std::string> goal_action, al;
				goal_action.resize(5);
				std::vector<int> loc_idx_zero;
				VTK->ShowData(P.GetPointParser(), goal_action, al, loc_idx_zero,
						color_code, true, false, false);
				delete VTK;
			}
		}
	}

	printf(
			"==============================================================================\n");
	printf(
			"|| LABELLING END                                                            ||\n");
	printf(
			"==============================================================================\n");

	return EXIT_SUCCESS;
}

int TestCase::ReadFileCData(
		std::shared_ptr<CData> cdata_,
		const std::string &object_,
		bool os_)
{
	std::string path;

	/**
	 * Reading surface
	 * Reading action labels
	 * - reads the labels and initializes a zero list prediction/filter with
	 *   the same length as the label
	 * Reading object specific labels
	 * - reads the object specific labels and saves them
	 */
	path = PARENT + "/" + KB_DIR + "/";
	if (RF->ReadFileKB(path, cdata_->KB) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	/* read object state if needed */
	if (os_)
	{
		path = PARENT + "/" + KB_DIR + "/";
		if (RF->ReadFileOS(path, cdata_->OS) == EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}
	}

	/* read parse message */
	path = PARENT + "/" + KB_DIR + "/";
	if (RF->ReadMsg(path, cdata_->msg) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	/* Check if data was trained */
	directoryCheck(PARENT + "/" + EVAL + "/" + object_ + "/");

	return EXIT_SUCCESS;
}

int TestCase::ReadFileExt(
		const std::vector<int> &idx_)
{
	std::string path;

	// Reading data filenames
	path = PARENT + "/" + DATA_DIR;
	if (RF->ReadFileName(path, idx_, file_list.get(), sub_num) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	printer(26);

	// Reading action sequence label
	path = PARENT + "/" + KB_DIR + "/";
	if (RF->ReadLabelSeq(path, label_list) == EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

