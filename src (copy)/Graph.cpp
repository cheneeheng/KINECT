/*
 * Graph.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#include "Graph.h"

//=============================================================================
// NODES
//=============================================================================
void Graph::addNode(
	string name_,
	unsigned int category_, //moving???
	unsigned int surface_num_,
	double boundary_,
	vector<data_t> data_)
{
	node = {};

	node.name     		= name_;
	node.index    		= nodes.size();
	node.category 		= category_;
	node.surface_num	= surface_num_;
	node.boundary 		= boundary_,
	node.data    	 	= data_;
	nodes.push_back(node);
	edges.push_back(emptyEdgeList());
}

void Graph::extendNode(
	vector<data_t> data_,
	unsigned int node_num_)
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
	unsigned int node_index1_)
{
	bool check_flag = false;
	if(nodes.size() > 0)
		if(nodes.size()-1 >= node_index1_)
			check_flag = true;
	return check_flag;
}

//=============================================================================
// EDGES
//=============================================================================
void Graph::addEdge(
	vector<data_t> data_,
	unsigned int node_index1_,
	unsigned int node_index2_,
	unsigned int num_location_intervals_,
	unsigned int num_sector_intervals_,
	vector<vector<sector_t> > sector_map_)
{
	edge = {};

	edge.begin_index 			= node_index1_;
	edge.end_index   			= node_index2_;
	edge.data 					= data_;
	edge.num_location_intervals = num_location_intervals_;
	edge.num_sector_intervals 	= num_sector_intervals_;
	edge.sector_map 			= sector_map_;

	checkEdgeList(node_index1_);
	edges[node_index1_].push_back(edge);
}

void Graph::extendEdge(
	vector<data_t> data_,
	unsigned int node_index1_,
	unsigned int node_index2_)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data to extend edge is empty." << endl;
	else if(data_.size()==1)
		edges[node_index1_][node_index2_].data.push_back(data_[0]);
	else
		edges[node_index1_][node_index2_].data.insert(
				edges[node_index1_][node_index2_].data.end(),
				data_.begin(), data_.end());
}

void Graph::checkEdgeList(
	unsigned int node_index1_)
{
	if(node_index1_ > edges.size())
	{
		cout << "[WARNING] : Edge does not exists in list. Resizing list." << endl;
		edges.resize(node_index1_);
	}
}

bool Graph::checkEdge(
	unsigned int node_index1_,
	unsigned int node_index2_,
	unsigned int &edge_num_ )
{
	bool check_flag = false;
	if(edges[node_index1_].size()>0)
		for(int i=0;i<edges[node_index1_].size();i++)
			if(edges[node_index1_][i].end_index == node_index2_)
			{
				check_flag = true;
				edge_num_ = (unsigned int)i;
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
