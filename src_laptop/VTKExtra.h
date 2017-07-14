/*******************************************************************************
 * VTKExtra.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 *      Detail: For viewing purposes.
 ******************************************************************************/

#ifndef VTKEXTRA_H_
#define VTKEXTRA_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSphereSource.h>
#include <vtkPointPicker.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkLineSource.h>
#include <vtkDoubleArray.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>
#include <vtkSmartPointer.h>
#include <vtkTubeFilter.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolygon.h>
#include <vtkRect.h>
#include <vtkAxis.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkGL2PSExporter.h>
#include <vtkObjectFactory.h>

#include "CGraph.h"
#include "algo.h"

#define CLICK_EMPTY		0
#define CLICK_LABEL		1
#define CLICK_DELETE 	2

#define WIN_HEIGHT		800
#define WIN_WIDTH 		1280
#define FONT_SIZE 		20

#define VTKCRED "\x1B[31m"
#define VTKCGRN "\x1B[32m"
#define VTKCYEL "\x1B[33m"
#define VTKCBLU "\x1B[34m"
#define VTKCMAG "\x1B[35m"
#define VTKCCYN "\x1B[36m"
#define VTKCWHT "\x1B[37m"
#define VTKCNOR "\x1B[0m"

class VTKExtra
{

private:

	int LOC_INT;
	int SEC_INT;

	void arrayTovector(
			unsigned char *A,
			int size,
			std::vector<unsigned char> &B)
	{
		//reshapeVector(B, std::extent<decltype(A)>::value);
		B.clear();
		for (int i = 0; i < size; i++)
		{
			B.push_back(A[i]);
		}
	}

public:

	VTKExtra(
			int loc_int_,
			int sec_int_);
	virtual ~VTKExtra();

	void ColorCode(
			std::vector<std::vector<unsigned char> > &container_);

	void ShowData(
			std::vector<Eigen::Vector4d> points_,
			std::vector<std::string> &labels_,
			std::vector<std::string> labels_ref_,
			std::vector<int> &loc_idx_,
			std::vector<std::vector<unsigned char> > color_,
			bool cluster_,
			bool labeling_,
			bool deleting_);

	void ShowConnectionOnly(
			CGraph Graph_,
			std::vector<std::vector<unsigned char> > color_);

	void ShowConnection(
			CGraph *Graph_,
			std::vector<Eigen::Vector4d> points_,
			std::vector<std::string> &labels_,
			std::vector<std::vector<unsigned char> > color_,
			bool show_points);

	void ShowConnectionTest(
			CGraph *Graph_,
			std::vector<Eigen::Vector4d> points_,
			std::vector<std::string> &labels_,
			std::vector<std::vector<unsigned char> > color_,
			bool show_points);

	void PlotData(
			std::vector<double> x,
			std::vector<double> y);

	void PlotData(
			std::vector<double> x,
			std::vector<double> y,
			std::vector<double> x2,
			std::vector<double> y2);

	void PlotDatas(
			std::vector<std::string> title,
			std::vector<double> x,
			std::vector<std::vector<std::vector<double> > > y);

	void PlotDatasGeo(
			std::vector<std::string> title,
			std::vector<double> x,
			std::vector<std::vector<std::vector<double> > > y);

	vtkSmartPointer<vtkPolyDataMapper> DataPoints(
			std::vector<Eigen::Vector4d> points_,
			int num_locations_,
			std::vector<std::vector<unsigned char> > color_,
			bool cluster_);
};

#endif /* VTKEXTRA_H_ */
