/*
 * Graph.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "dataDeclaration.h"

class Graph
{
public:
	Graph(
		string scene_,
		string object_);

	void addSurface(
		vector<vector<double> > surface_);

	void updateMovLabel(
		vector<string> movLabel_);

	void addNode(
		string 			name_,
		unsigned int 	index_,
		int 			category_, // not used
		point_t 		location,
		double 			boundary,
		int 			surface_num_,
		double			surface_boundary_,
		vector<data_t> 	data_);

	void extendNode(
		unsigned int 	node_num_,
		vector<data_t> 	data_);

	bool checkNode(
		unsigned int node_index_);

	void updateNodeName(
		vector<string> names_);

	void initEdge(
		int loc_int,
		int sec_int);

	void addEdge(
		vector<data_t>	data_,
		vector<double>	sector_map_,
		unsigned int	n1_,
		unsigned int	n2_,
		unsigned int	edge_num_);

	void updateEdgeData(
		vector<data_t>	data_,
		unsigned int 	n1_,
		unsigned int 	n2_,
		unsigned int 	edge_num_);

	void updateEdgeConst(
		vector<double> 	sector_map_,
		unsigned int  	n1_,
		unsigned int  	n2_,
		unsigned int 	edge_num_);

	void updateEdgeSector(
		vector<double>	sector_map_,
		unsigned int  	n1_,
		unsigned int  	n2_,
		unsigned int 	edge_num_);

	void updateEdgeNormal(
		vector<point_t>  normal_,
		unsigned int  	 n1_,
		unsigned int  	 n2_,
		unsigned int 	 edge_num_);

	void updateEdgeTangent(
		vector<point_t> 	tangent_,
		unsigned int  		n1_,
		unsigned int  		n2_,
		unsigned int 		edge_num_);

	void updateEdgeLocStartMidEnd(
		vector<point_t> 	start_,
		vector<point_t> 	mid_,
		vector<point_t> 	end_,
		unsigned int  		n1_,
		unsigned int  		n2_,
		unsigned int 		edge_num_);

	void updateEdgeLocDist(
		double 				total_,
		unsigned int  		n1_,
		unsigned int  		n2_,
		unsigned int 		edge_num_);

	void extendEdge(
		vector<data_t> 	data_,
		unsigned int 	node_index1_,
		unsigned int 	node_index2_,
		unsigned int 	edge_num_);

	bool checkEdge(
		unsigned int node_index1_,
		unsigned int node_index2_);

	void updateSectorPara(
		sector_para_t sector_para_);

	void incrementCounter(
		int edge_,
		int edge_num_)
	{counter[edge_][edge_num_]++;}

	node_tt getNode(
		unsigned int node_index)
	{return nodes[node_index];}

	vector<node_tt> getNodeList()
	{return nodes;}

	vector<string> getNodeName();

	vector<double> getInitSector()
	{return sector_zero;}

	vector<double> getInitSectorConst()
	{return sector_zero;}

	sector_para_t getSectorPara()
	{return sector_para;}

	vector<vector<edge_tt> > getEdgeList()
	{return edges;}

	vector<vector<double> > getNodeDataLabel(
		bool pos_=false,
		bool vel_=false,
		bool acc_=false);

	vector<vector<double> > getEdgeDataLabel(
		bool pos_=false,
		bool vel_=false,
		bool acc_=false);

	vector<vector<double> > getSurface()
	{return surface;}

	string getScene()
	{return scene;}

	string getObject()
	{return object;}

	vector<string> getMovLabel()
	{return movLabel;}

	int getCounter(
		int edge_,
		int edge_num_)
	{return counter[edge_][edge_num_];}

	virtual ~Graph(){}

private:
	vector<vector<int>>		    counter;

	string 						scene;
	string 						object;
	vector<string>				movLabel;
	vector<double> 				sector_zero;
	sector_para_t 				sector_para;
	vector<point_t>				tangent_zero;
	vector<point_t>				normal_zero;
	vector<point_t>				loc_start_zero;
	vector<point_t>				loc_mid_zero;
	vector<point_t>				loc_end_zero;

	node_tt 					node;
	edge_tt 					edge;
	vector<node_tt> 			nodes;
	vector<vector<edge_tt> > 	edges;
	vector<vector<double> > 	surface;
};


#endif /* GRAPH_H_ */
