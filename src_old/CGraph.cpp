/*******************************************************************************
 * CGraph.cpp
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: A graph structure with edges and nodes.
 *     			Node list contains the nodes.
 *     			Edge list contains the edges in the form of a list.
 *     			([node1][node2][number of edges])
 ******************************************************************************/

#include "CGraph.h"

CGraph::CGraph( ) :	OBJECT(""),
					LOC_INT(-1),
					SEC_INT(-1)
{
}
CGraph::CGraph(
		const std::string &object_,
		const int &loc_int_,
		const int &sec_int_)
		:	OBJECT(object_),
			LOC_INT(loc_int_),
			SEC_INT(sec_int_)
{
}
CGraph::~CGraph() { }

/*******************************************************************************
 * Nodes
 ******************************************************************************/

std::vector<Eigen::Vector4d> CGraph::GetCentroidList() const
{
	std::vector<Eigen::Vector4d> centroid_list;
	for(auto i:node_list) { centroid_list.push_back(i.centroid); }
	return centroid_list;
}

std::vector<int> CGraph::GetSurfaceFlagList() const
{
	std::vector<int> surface_flag_list;
	for(auto i:node_list) { surface_flag_list.push_back(i.surface_flag); }
	return surface_flag_list;
}

/*******************************************************************************
 * Edges
 ******************************************************************************/

void CGraph::addEmptyEdgeForNewNode(
	int idx_)
{
	if(edge_list.size()<idx_+1)
	{
		edge_list.resize(idx_+1);

		for(auto &i:edge_list)
		{
			if(i.size()<=GetNumberOfNodes())
			{
				i.resize(GetNumberOfNodes());
				for(auto &ii:i)
				{
					if (ii.size()==0) { ii.push_back({}); }
					ii[0].tan.resize(LOC_INT);
					ii[0].nor.resize(LOC_INT);
					ii[0].loc_mid.resize(LOC_INT);
					ii[0].loc_len.resize(LOC_INT);
					ii[0].sector_map.resize(LOC_INT*SEC_INT);
					ii[0].mov_const.resize(2);
				}
			}
		}
	}
}

