/*
 * Graph.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#include "Graph.h"

Graph::Graph(
	string scene_,
	string object_)
{
	scene 	= scene_;
	object 	= object_;
	node 	= {};
	edge 	= {};

	movLabel.clear();
	nodes.clear();
	edges.clear();
	surface.clear();
}

void Graph::addSurface(
	vector<vector<double> > surface_)
{
	surface = surface_;
}

void Graph::updateMovLabel(
	vector<string> movLabel_)
{
	movLabel.clear();
	movLabel = movLabel_;
}

void Graph::updateSectorPara(
	sector_para_t sector_para_)
{
	sector_para = {};
	sector_para = sector_para_;
}

//=============================================================================
// NODES
//=============================================================================
void Graph::addNode(
	string 			name_,
	unsigned int 	index_,
	int 			category_, //not used
	point_t 		location_,
	double 			boundary_,
	int 			surface_num_,
	double			surface_boundary_,
	vector<data_t> 	data_)
{
	node = {};

	node.name     			= name_;
	node.index    			= index_;
	node.category 			= category_;
	node.location			= location_;
	node.boundary 			= boundary_;
	node.surface			= surface_num_;
	node.surface_boundary	= surface_boundary_;
	node.data    			= data_;

	if (nodes.size() < index_+1)
		nodes.resize(index_+1);

	nodes[index_] = node;
}

void Graph::extendNode(
	unsigned int 	node_num_,
	vector<data_t> 	data_)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data used to extend node is empty." << endl;
	else if(data_.size()==1)
		nodes[node_num_].data.push_back(data_[0]);
	else
		nodes[node_num_].data.insert(nodes[node_num_].data.end(),
				                     data_.begin(), data_.end());
}

bool Graph::checkNode(
	unsigned int n1_)
{
	bool check_flag = false;
	if(nodes.size() >= n1_+1)
		check_flag = true;
	return check_flag;
}

vector<string> Graph::getNodeName()
{
	vector<string> names(nodes.size());
	for(int i=0;i<nodes.size();i++)
		names[i] = nodes[i].name;
	return names;
}

void Graph::updateNodeName(vector<string> names_)
{
	if(names_.size()!=nodes.size())
		printf("[WARNING] : Number of names is not the same as the number of nodes in graph...");
	else
		for(int i=0;i<nodes.size();i++)
			nodes[i].name = names_[i];
}

//=============================================================================
// EDGES
//=============================================================================
void Graph::initEdge(
	int loc_int,
	int sec_int)
{
	edges.clear();
	sector_zero.clear();
	tangent_zero.clear();
	normal_zero.clear();
	loc_start_zero.clear();
	loc_mid_zero.clear();
	loc_end_zero.clear();
	counter.clear();

	edges.resize(Sqr(nodes.size()));
	sector_zero.resize(loc_int*sec_int);
	tangent_zero.resize(loc_int);
	normal_zero.resize(loc_int);
	loc_start_zero.resize(loc_int);
	loc_mid_zero.resize(loc_int);
	loc_end_zero.resize(loc_int);
	counter.resize(Sqr(nodes.size()));

	for(int i=0;i<Sqr(nodes.size());i++)
	{
		counter[i].resize(1);
		edges[i].resize(1);
		edges[i][0].sector_map   = sector_zero;
		edges[i][0].sector_const = sector_zero;
		edges[i][0].nor 		 = normal_zero;
		edges[i][0].tan	 		 = tangent_zero;
		edges[i][0].loc_start 	 = loc_start_zero;
		edges[i][0].loc_mid 	 = loc_mid_zero;
		edges[i][0].loc_end 	 = loc_end_zero;
		edges[i][0].total_len 	 = 0.0;
	}

	sector_para.loc_int = loc_int;
	sector_para.sec_int = sec_int;

	sector_para.dir.clear();
	sector_para.dir_n.clear();
	sector_para.dist.clear();

	sector_para.dir.  resize(Sqr(nodes.size()));
	sector_para.dir_n.resize(Sqr(nodes.size()));
	sector_para.dist. resize(Sqr(nodes.size()));
}

void Graph::addEdge(
	vector<data_t> data_,
	vector<double> sector_map_,
	unsigned int   n1_,
	unsigned int   n2_,
	unsigned int   edge_num_)
{

	edge = {};

	edge.idx1		= n1_;
	edge.idx2		= n2_;
	edge.data		= data_;
	edge.sector_map	= sector_map_;

	if (edges[n1_*nodes.size()+n2_].size()>edge_num_)
	{
		edge.sector_const =
				edges[n1_*nodes.size()+n2_][edge_num_].sector_const;
		edges[n1_*nodes.size()+n2_][edge_num_] = edge;
	}
	else
	{
		edges[n1_*nodes.size()+n2_].resize(edge_num_);
		edge.sector_const = sector_zero;
		edges[n1_*nodes.size()+n2_][edge_num_] = edge;
	}
}

void Graph::updateEdgeData(
	vector<data_t> 	 data_,
	unsigned int 	 n1_,
	unsigned int 	 n2_,
	unsigned int 	 edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].data = data_;
}

void Graph::updateEdgeConst(
	vector<double> 	sector_map_,
	unsigned int  	n1_,
	unsigned int  	n2_,
	unsigned int 	edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].sector_const = sector_map_;
}

void Graph::updateEdgeSector(
	vector<double>	sector_map_,
	unsigned int  	n1_,
	unsigned int  	n2_,
	unsigned int 	edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].sector_map = sector_map_;
}

void Graph::updateEdgeNormal(
	vector<point_t> 	normal_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].nor = normal_;
}

void Graph::updateEdgeTangent(
	vector<point_t> 	tangent_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].tan = tangent_;
}

void Graph::updateEdgeLocStartMidEnd(
	vector<point_t> 	start_,
	vector<point_t> 	mid_,
	vector<point_t> 	end_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].loc_start = start_;
	edges[n1_*nodes.size()+n2_][edge_num_].loc_mid = mid_;
	edges[n1_*nodes.size()+n2_][edge_num_].loc_end = end_;
}

void Graph::updateEdgeLocDist(
	double				total_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].total_len = total_;
}

void Graph::extendEdge(
	vector<data_t> data_,
	unsigned int n1_,
	unsigned int n2_,
	unsigned int edge_num_)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data to extend edge is empty." << endl;
	else if(data_.size()==1)
		edges[n1_*nodes.size()+n2_][edge_num_].data.push_back(data_[0]);
	else
		edges[n1_*nodes.size()+n2_][edge_num_].data.insert(
				edges[n1_*nodes.size()+n2_][edge_num_].data.end(),
				data_.begin(), data_.end());
}

bool Graph::checkEdge(
	unsigned int n1_,
	unsigned int n2_)
{
	bool check_flag = true;
	if (edges[n1_*nodes.size()+n2_].empty())
	{
		check_flag = false;
	}
	return check_flag;
}

vector<vector<double> > Graph::getEdgeDataLabel(
	bool pos_,
	bool vel_,
	bool acc_)
{
	vector<vector<double> > output;
	vector<vector<edge_tt> > list = getEdgeList();
	vector<data_t> data;
	// list of nodes with edges
	for(int i=0; i<list.size();i++)
	{
		// list of edges for each node
		for(int ii=0; ii<list[i].size();ii++)
		{
			data = list[i][ii].data;
			// list of data in each edge
			for(int iii=0; iii<data.size();iii++)
			{
				vector<double> tmp_out;
				tmp_out.push_back(i);
				if(pos_)
				{
					point_t tmp_pos = data[iii].pos;
					tmp_out.push_back(tmp_pos.x);
					tmp_out.push_back(tmp_pos.y);
					tmp_out.push_back(tmp_pos.z);
				}
				if(vel_)
				{
					point_t tmp_vel = data[iii].vel;
					tmp_out.push_back(tmp_vel.x);
					tmp_out.push_back(tmp_vel.y);
					tmp_out.push_back(tmp_vel.z);
				}
				if(acc_)
				{
					point_t tmp_acc = data[iii].acc;
					tmp_out.push_back(tmp_acc.x);
					tmp_out.push_back(tmp_acc.y);
					tmp_out.push_back(tmp_acc.z);
				}
				output.push_back(tmp_out);
			}
		}
	}
	return output;
}

vector<vector<double> > Graph::getNodeDataLabel(
	bool pos_,
	bool vel_,
	bool acc_)
{
	vector<vector<double> > output; // length * XYZId
	vector<node_tt> list = getNodeList();
	vector<data_t> data;
	// list of nodes
	for(int i=0; i<list.size();i++)
	{
		//printf("Category %02d %02d\n", i, list[i].category);
		data = list[i].data;
		// list of node data
		for(int ii=0; ii<data.size();ii++)
		{
			vector<double> tmp_out;
			if(pos_)
			{
				point_t tmp_pos = data[ii].pos;
				tmp_out.push_back(tmp_pos.x);
				tmp_out.push_back(tmp_pos.y);
				tmp_out.push_back(tmp_pos.z);
				tmp_out.push_back(i);
			}
			if(vel_)
			{
				point_t tmp_vel = data[ii].vel;
				tmp_out.push_back(tmp_vel.x);
				tmp_out.push_back(tmp_vel.y);
				tmp_out.push_back(tmp_vel.z);
				tmp_out.push_back(i);
			}
			if(acc_)
			{
				point_t tmp_acc = data[ii].acc;
				tmp_out.push_back(tmp_acc.x);
				tmp_out.push_back(tmp_acc.y);
				tmp_out.push_back(tmp_acc.z);
				tmp_out.push_back(i);
			}
			output.push_back(tmp_out);
		}
	}
	return output;
}


