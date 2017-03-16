/*
 * readWriteFIle.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#include "readWriteFile.h"

#ifdef PC
	string SCENE = "../Scene/";
#else
	string SCENE = "./Scene/";
#endif

void writeSurfaceFile(
	vector<vector<double> > surface_)
{
	directoryCheck(SCENE + string("Kitchen"));
	string path = SCENE + "Kitchen/surface.txt";
	if (!ifstream(path))
	{
		ofstream write_file(path, std::ios::app);
		for(int i=0;i<surface_.size();i++)
		{
			write_file << i;
			for(int ii=0;ii<4;ii++)
			{
				write_file << ",";
				write_file << surface_[i][ii];
			}
			write_file << "\n";
		}
	}
}

void writeSurfaceFile(
	Graph Graph_)
{
	directoryCheck(SCENE + Graph_.getScene());
	string path = SCENE + Graph_.getScene() + "/surface.txt";
	vector<vector<double> > surface = Graph_.getSurface();
	if (!ifstream(path))
	{
		ofstream write_file(path, std::ios::app);
		for(int i=0;i<surface.size();i++)
		{
			write_file << i;
			for(int ii=0;ii<4;ii++)
			{
				write_file << ",";
				write_file << surface[i][ii];
			}
		}
		write_file << "\n";
	}
}

void writeLabelFile(
	Graph Graph_,
	string path_,
	int movloc_)
{
	switch(movloc_)
	{
		case 0:
			if (!ifstream(path_))
			{
				vector<node_tt> node_tmp = Graph_.getNodeList();
				ofstream write_file(path_, ios::app);
				for(int i=0;i<node_tmp.size();i++)
				{
					write_file << node_tmp[i].name       << ","
							   << node_tmp[i].location.x << ","
							   << node_tmp[i].location.y << ","
							   << node_tmp[i].location.z << ","
							   << node_tmp[i].boundary   << ","
							   << node_tmp[i].surface	 << ","
							   << node_tmp[i].surface_boundary;
					write_file << "\n";
				}
			}
			break;
		case 1:
			if (!ifstream(path_))
			{
				vector<string> label_tmp = Graph_.getMovLabel();
				ofstream write_file(path_, ios::app);
				for(int i=0;i<label_tmp.size();i++)
				{
					write_file << label_tmp[i];
					if (i<label_tmp.size()-1)
						write_file << ",";
				}
				write_file << "\n";
			}
			break;
	}
}

void writeLearnedDataFile(
	Graph Graph_,
	string path_,
	int type_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	sector_para_t para = Graph_.getSectorPara();

	vector<point_t> data;
	vector<double> 	sec;

	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << "Locations," 			<< num_locations 	<< "\n";
		write_file << "Location Intervals," << para.loc_int 	<< "\n";
		write_file << "Sector Intervals," 	<< para.sec_int 	<< "\n";
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				data.clear(); sec.clear();
				write_file << "Edge,"    << i
						   << ",Number," << ii << "\n";
				switch (type_)
				{
					case 0:
						data = edges[i][ii].loc_start;
						break;
					case 1:
						data = edges[i][ii].loc_mid;
						break;
					case 2:
						data = edges[i][ii].loc_end;
						break;
					case 3:
						data = edges[i][ii].tan;
						break;
					case 4:
						data = edges[i][ii].nor;
						break;
					case 5:
						write_file << Graph_.getCounter(i,ii)
								   << "\n";
						break;
					case 6:
						sec = edges[i][ii].sector_map;
						for(int iii=0;iii<sec.size();iii++)
						{
							write_file << sec[iii];
							if (iii<sec.size()-1) 	{ write_file << ",";  }
							else 					{ write_file << "\n"; }
						}
						break;
					case 7:
						sec = edges[i][ii].sector_const;
						for(int iii=0;iii<sec.size();iii++)
						{
							write_file << sec[iii];
							if (iii<sec.size()-1) 	{ write_file << ",";  }
							else 					{ write_file << "\n"; }
						}
						break;
				}
				if (type_ > 4) {continue;}
				for(int iii=0;iii<para.loc_int;iii++)
				{
					write_file << data[iii].x << ",";
					write_file << data[iii].y << ",";
					write_file << data[iii].z;
					if (iii<data.size()-1) 	{ write_file << ",";  }
					else 					{ write_file << "\n"; }
				}
			}
		}
	}
}

int fileSelect(
	const struct dirent *entry)
{
	int n;
	size_t found_extension = string(entry->d_name).find(".txt");
	if ((int)found_extension == -1)
		n = 0;
	else
		n = 1;
	return n;
}

void readFile(
	const char *name,
	vector<vector<string> > &data_full,
	char delimiter)
{
	ifstream src_file( name );
	while (src_file)
	{
		string file_line_;
		if (!getline( src_file, file_line_ )) break;
		istringstream line_( file_line_ );
		vector <string> data_line_;
		while (line_)
		{
		   string word;
		   if (!getline( line_, word, delimiter)) break;
		   data_line_.push_back( word );
		}
		data_full.push_back( data_line_ );
	}

	if (!src_file.eof())
		cerr << "FILE ERROR!!!\n";
}

void readSurfaceFile(
	Graph &Graph_)
{
	string path;
	vector<vector<string> > data;

	path = SCENE + Graph_.getScene() + "/surface.txt";
	readFile(path.c_str(), data, ',');

	vector<vector<double> > surface_;
	reshapeVector(surface_, data.size());

	for(int i=0;i<data.size();i++)
	{
		int tmp = atoi(data[i][0].c_str());
		for(int ii=0;ii<7;ii++)
			surface_[tmp].push_back(atof(data[i][ii+1].c_str()));
	}

	Graph_.addSurface(surface_);
}

void readLocation(
	vector<vector<string> > data_,
	vector<string> 	&label_,
	vector<point_t> &locations_,
	vector<double> 	&locations_boundary_,
	vector<int> 	&surfaces_num_,
	vector<double> 	&surfaces_boundary_)
{
	int num_locations = data_.size();
	if(data_.empty())
	{
		printf("[WARNING] : Location data is empty.");
	}
	else
	{
		reshapeVector(locations_,		  	num_locations);
		reshapeVector(locations_boundary_,	num_locations);
		reshapeVector(surfaces_num_,		num_locations);
		reshapeVector(surfaces_boundary_,	num_locations);
		reshapeVector(label_,		     	num_locations+1);
		for(int i=0;i<data_.size();i++)
		{
			label_[i] 		  	  		=      data_[i][0];
			locations_[i].x 		  	= atof(data_[i][1].c_str());
			locations_[i].y 		  	= atof(data_[i][2].c_str());
			locations_[i].z 		  	= atof(data_[i][3].c_str());
			locations_[i].cluster_id	= UNCLASSIFIED;
			locations_boundary_[i] 	  	= atof(data_[i][4].c_str());
			surfaces_num_[i] 	  	  	= atof(data_[i][5].c_str());
			surfaces_boundary_[i] 	   	= atof(data_[i][6].c_str());
		}
	}
	printf("Reviewing location labels...\n");
	for(int i=0;i<num_locations;i++)
	{
		printf("LLabel %d : %s\n", i, label_[i].c_str());
	}
}

void readLocation_(
	Graph &Graph_,
	vector<vector<string> > data_)
{
	vector<string> 	label;
	vector<point_t> locations;
	vector<double> 	locations_boundary;
	vector<int> 	surfaces_num;
	vector<double> 	surfaces_boundary;
	vector<data_t>  data_tmp;

	readLocation(
			data_, label, locations, locations_boundary,
			surfaces_num, surfaces_boundary);

	for(int i=0;i<locations.size();i++)
		Graph_.addNode(label[i], i, -1,
					   locations[i], locations_boundary[i],
					   surfaces_num[i], surfaces_boundary[i], data_tmp);
}

void readMovement(
	Graph &Graph_,
	vector<vector<string> > data_)
{
	vector<string> label;
	if(data_.empty())
	{
		printf("[WARNING] : Movement data is empty.");
	}
	else
	{
		reshapeVector(label, data_[0].size());
		for(int ii=0;ii<data_[0].size();ii++)
			label[ii] = data_[0][ii];
	}
	printf("Reviewing movement labels for OBJ...\n");
	for(int i=0;i<data_[0].size();i++)
	{
		printf("MLabel %d : %s\n", i, label[i].c_str());
	}
	Graph_.updateMovLabel(label);
}

void readSectorFile(
	Graph &Graph_,
	int type_)
{
	vector<data_t> data_zero;

	string path;
	switch(type_)
	{
		case 0:
			path =  SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_max.txt";
			break;
		case 1:
			path =  SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/sec_data_const.txt";
			break;
	}

	vector<vector<string> > data;
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Sector data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][1].c_str());
		num_location_intervals 	= atoi(data[1][1].c_str());
		num_sector_intervals 	= atoi(data[2][1].c_str());

		if (Graph_.getEdgeList().empty())
			Graph_.initEdge(num_location_intervals,num_sector_intervals);

		vector<vector<edge_tt> > edges = Graph_.getEdgeList();

		int c = 0;

		for(int i=0;i<Sqr(num_locations);i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				vector<double> sector_map   = edges[i][ii].sector_map;
				vector<double> sector_const = edges[i][ii].sector_const;
				c+=2;
				int tmp = c+2;
				for(int iii=0;iii<num_location_intervals*num_sector_intervals;iii++)
				{
					switch(type_)
					{
						case 0:
							sector_map[iii]		= atof(data[tmp][iii].c_str());
							break;
						case 1:
							sector_const[iii]	= atof(data[tmp][iii].c_str());
							break;
					}
				}
				switch(type_)
				{
					case 0:
						Graph_.updateEdgeSector(sector_map, i/num_locations, i%num_locations, ii);
						break;
					case 1:
						Graph_.updateEdgeConst(sector_const, i/num_locations, i%num_locations, ii);
						break;
				}
			}
		}
	}
	// Should we add the option to edit the data?
}

void readLocationFile(
	Graph &Graph_,
	int type_)
{
	vector<data_t> data_zero;

	string path;
	switch(type_)
	{
		case 0:
			path = 	SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data_beg.txt";
			break;
		case 1:
			path = 	SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data_mid.txt";
			break;
		case 2:
			path = 	SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data_end.txt";
			break;
		case 3:
			path = 	SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data_tangent.txt";
			break;
		case 4:
			path = 	SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/loc_data_normal.txt";
			break;
	}

	vector<vector<string> > data;
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Location data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][1].c_str());
		num_location_intervals 	= atoi(data[1][1].c_str());
		num_sector_intervals 	= atoi(data[2][1].c_str());

		if (Graph_.getEdgeList().empty())
			Graph_.initEdge(num_location_intervals,num_sector_intervals);

		vector<vector<edge_tt> > edges = Graph_.getEdgeList();

		int c = 0;

		for(int i=0;i<Sqr(num_locations);i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				c+=2;
				int tmp = c+2;
				for(int iii=0;iii<num_location_intervals;iii++)
				{
					switch(type_)
					{
						case 0:
							edges[i][ii].loc_start[iii].x = atof(data[tmp][iii*3+0].c_str());
							edges[i][ii].loc_start[iii].y = atof(data[tmp][iii*3+1].c_str());
							edges[i][ii].loc_start[iii].z = atof(data[tmp][iii*3+2].c_str());
							break;
						case 1:
							edges[i][ii].loc_mid[iii].x = atof(data[tmp][iii*3+0].c_str());
							edges[i][ii].loc_mid[iii].y = atof(data[tmp][iii*3+1].c_str());
							edges[i][ii].loc_mid[iii].z = atof(data[tmp][iii*3+2].c_str());
							break;
						case 2:
							edges[i][ii].loc_end[iii].x = atof(data[tmp][iii*3+0].c_str());
							edges[i][ii].loc_end[iii].y = atof(data[tmp][iii*3+1].c_str());
							edges[i][ii].loc_end[iii].z = atof(data[tmp][iii*3+2].c_str());
							break;
						case 3:
							edges[i][ii].tan[iii].x = atof(data[tmp][iii*3+0].c_str());
							edges[i][ii].tan[iii].y = atof(data[tmp][iii*3+1].c_str());
							edges[i][ii].tan[iii].z = atof(data[tmp][iii*3+2].c_str());
							break;
						case 4:
							edges[i][ii].nor[iii].x = atof(data[tmp][iii*3+0].c_str());
							edges[i][ii].nor[iii].y = atof(data[tmp][iii*3+1].c_str());
							edges[i][ii].nor[iii].z = atof(data[tmp][iii*3+2].c_str());
							break;
					}
				}
				switch(type_)
				{
					case 0:
						Graph_.updateEdgeLocStartMidEnd(
								edges[i][ii].loc_start, edges[i][ii].loc_mid, edges[i][ii].loc_end,
								i/num_locations, i%num_locations, ii);
						break;
					case 1:
						Graph_.updateEdgeLocStartMidEnd(
								edges[i][ii].loc_start, edges[i][ii].loc_mid, edges[i][ii].loc_end,
								i/num_locations, i%num_locations, ii);
						break;
					case 2:
						Graph_.updateEdgeLocStartMidEnd(
								edges[i][ii].loc_start, edges[i][ii].loc_mid, edges[i][ii].loc_end,
								i/num_locations, i%num_locations, ii);
						break;
					case 3:
						Graph_.updateEdgeTangent(
								edges[i][ii].tan,
								i/num_locations, i%num_locations, ii);
						break;
					case 4:
						Graph_.updateEdgeNormal(
								edges[i][ii].nor,
								i/num_locations, i%num_locations, ii);
						break;
				}
			}
		}
	}
	// Should we add the option to edit the data?




}

void readCounterFile(
	Graph &Graph_,
	int type_)
{
	vector<data_t> data_zero;

	string path = SCENE + Graph_.getScene() + "/" + Graph_.getObject() + "/counter.txt";;

	vector<vector<string> > data;
	readFile(path.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Counter data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][1].c_str());
		num_location_intervals 	= atoi(data[1][1].c_str());
		num_sector_intervals 	= atoi(data[2][1].c_str());

		if (Graph_.getEdgeList().empty())
			Graph_.initEdge(num_location_intervals,num_sector_intervals);

		vector<vector<edge_tt> > edges = Graph_.getEdgeList();

		int c = 0;
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				c+=2;
				int tmp = c+2;
				for(int iii=0;iii<atoi(data[tmp][0].c_str());iii++)
					Graph_.incrementCounter(i,ii);
			}
		}
	}
}

bool copyFile(
	string SRC,
	string DEST)
{
    ifstream src (SRC,  ios::binary);
    ofstream dest(DEST, ios::binary);
    dest << src.rdbuf();
    return src && dest;
}

bool directoryCheck(
	string path_ )
{
    if (path_.empty())
    {
    	return false;
    }
    DIR *dir;
    bool exist = false;
    dir = opendir(path_.c_str());
    if (dir != NULL)
    {
    	exist = true; closedir(dir);
    }
    if (!exist)
    {
    	mkdir(path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    return exist;
}




