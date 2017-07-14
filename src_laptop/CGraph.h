/*******************************************************************************
 * CGraph.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: A graph structure with edges and nodes.
 *     			Node list contains the nodes.
 *     			Edge list contains the edges in the form of a list.
 *     			([node1][node2][number of edges])
 ******************************************************************************/

#ifndef CGRAPH_H_
#define CGRAPH_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>

class CGraph
{

public:
	struct node_t
	{
		std::string name;
		int index; // used to check for new nodes
		int contact;
		Eigen::Vector4d centroid; // Sphere
		int surface_flag; // Surface
		Eigen::Vector3d cuboid_max;
		Eigen::Vector3d cuboid_min;
	};
	struct edge_t
	{
		std::string name;
		unsigned int index1; //start node
		unsigned int index2; // end node
		std::vector<double> sector_map; // locations int * sectors int
		std::vector<Eigen::Vector3d> tan; // locations int
		std::vector<Eigen::Vector3d> nor; // locations int
		std::vector<Eigen::Vector4d> loc_mid; // locations int
		std::vector<double> loc_len; // locations int
		double total_len;
		int counter;
		std::vector<int> mov_const; // 0/1 activation of the mov_const labels
		std::vector<double> loc_mem; // to calculate d2(loc)
		std::vector<double> sec_mem; // to calculate d2(sec)
		std::vector<double> err_mem; // to calculate d2(err)
	};

	CGraph();
	CGraph(
			const std::string &object_,
			const int &loc_int_,
			const int &sec_int_);
	virtual ~CGraph();

	std::string GetObject()
	{
		return OBJECT;
	}
	void SetObject(
			std::string obj_)
	{
		OBJECT = obj_;
	}
	int GetLocInt()
	{
		return LOC_INT;
	}
	void SetLocInt(
			int loc_)
	{
		LOC_INT = loc_;
	}
	int GetSecInt()
	{
		return SEC_INT;
	}
	void SetSecInt(
			int sec_)
	{
		SEC_INT = sec_;
	}

	/*******************************************************************************
	 * Nodes
	 *******************************************************************************/

	virtual node_t GetNode(
			int idx_) const
	{
		if (idx_ < node_list.size())
		{
			return node_list[idx_];
		}
		else
		{
			return
			{};
		}
	}

	virtual int SetNode(
			node_t node_)
	{
		if (node_list.size() < node_.index + 1)
		{
			node_list.resize(node_.index + 1);
		}
		node_list[node_.index] = node_;
		return EXIT_SUCCESS;
	}

	virtual int GetNumberOfNodes() const
	{
		return node_list.size();
	}
	virtual std::vector<node_t> GetNodeList() const
	{
		return node_list;
	}

	virtual std::vector<Eigen::Vector4d> GetCentroidList() const;
	virtual std::vector<int> GetSurfaceFlagList() const;

	/*******************************************************************************
	 * Edges
	 *******************************************************************************/

	virtual void addEmptyEdgeForNewNode(
			int idx_);

	// edge_list = [#loc1] [#loc2] [#edges] [#sec*#loc]
	virtual std::vector<std::vector<std::vector<edge_t> > > GetListOfEdges()
	{
		return edge_list;
	}

	virtual edge_t GetEdge(
			int n1_,
			int n2_,
			int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_];
	}

	virtual void SetEdge(
			int n1_,
			int n2_,
			int edge_num_,
			edge_t edge_)
	{
		edge_list[n1_][n2_][edge_num_] = edge_;
	}

	virtual int GetEdgeCounter(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_].counter;
	}

	virtual void SetEdgeCounter(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			unsigned int x_)
	{
		edge_list[n1_][n2_][edge_num_].counter += x_;
	}

	void GetEdgeMovementConstraint(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<int> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].mov_const;
	}

	void SetEdgeMovementConstraint(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<int> x_)
	{
		edge_list[n1_][n2_][edge_num_].mov_const = x_;
	}

	void GetEdgeLocMem(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].loc_mem;
	}

	void SetEdgeLocMem(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].loc_mem = x_;
	}

	void GetEdgeSecMem(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].sec_mem;
	}

	void SetEdgeSecMem(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].sec_mem = x_;
	}

	std::vector<double> GetEdgeSectorMap(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_].sector_map;
	}

	void SetEdgeSectorMap(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].sector_map = x_;
	}

	void GetEdGetan(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<Eigen::Vector3d> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].tan;
	}

	void SetEdGetan(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<Eigen::Vector3d> x_)
	{
		edge_list[n1_][n2_][edge_num_].tan = x_;
	}

	void GetEdgeNor(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<Eigen::Vector3d> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].nor;
	}

	void SetEdgeNor(
			unsigned int n1_,
			unsigned int n2_,
			unsigned int edge_num_,
			std::vector<Eigen::Vector3d> x_)
	{
		edge_list[n1_][n2_][edge_num_].nor = x_;
	}

private:
	int LOC_INT;
	int SEC_INT;
	std::string OBJECT; // what kind of action Set is being evaluated

	std::vector<node_t> node_list;
	std::vector<std::vector<std::vector<edge_t> > > edge_list;

};

#endif /* CGRAPH_H_ */
