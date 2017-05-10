/*
 * dataDeclaration.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef DATADECLARATION_H_
#define DATADECLARATION_H_

#define PC

#ifdef PC
	// For backward compatibility with new VTK generic data arrays
	#define InsertNextTypedTuple InsertNextTupleValue
#endif

#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <list>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stack>
#include <semaphore.h>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <iterator>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkLookupTable.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointSource.h>
#include <vtkPointData.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPropPicker.h>
#include <vtkSphereSource.h>
#include <vtkPointPicker.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTubeFilter.h>
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
#include <vtkVersion.h>
#include <vtkParametricFunctionSource.h>
#include <vtkTupleInterpolator.h>
#include <vtkTubeFilter.h>
#include <vtkParametricSpline.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkAssembly.h>
#include <vtkPropAssembly.h>
#include <vtkRegularPolygonSource.h>
#include <vtkPolygon.h>
#include <vtkCurvatures.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkLight.h>

#include <gsl/gsl_integration.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>

using namespace std;

#ifdef PC
	#include <opencv2/opencv.hpp>
	using namespace cv;
#endif

#define Sqr(x) ((x)*(x))
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define Calloc(type,n) (type *)calloc( n, sizeof(type))

//0 : all
//1 : motion
//2 : location
//3 : label only
#define VERBOSE 0

#define FILTER_WIN 15

#define BOUNDARY_VAR 0.01

// number of fit coefficients
// nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4
#define NCOEFFS	12
#define NBREAK 	(NCOEFFS - 2)
#define DEGREE 5 //k+1

#define OUT_OF_BOUND 2
#define OUT_OF_RANGE 1
#define	WITHIN_RANGE 0
#define EXCEED_RANGE -1

#define WIN_HEIGHT	800
#define WIN_WIDTH 	1200
#define FONT_SIZE 	10

//******************** TAKEN FROM .....
#define UNCLASSIFIED 	-1
#define NOISE 			-2

#define CORE_POINT 		 1
#define NOT_CORE_POINT 	 0

#define SUCCESS 		 0
#define FAILURE 		-3

typedef struct point_s point_t;
struct point_s {
    double x, y, z;
    int cluster_id;
};

typedef struct node_s node_t;
struct node_s {
    unsigned int index;
    node_t *next;
};

typedef struct epsilon_neighbours_s epsilon_neighbours_t;
struct epsilon_neighbours_s {
    unsigned int num_members;
    node_t *head, *tail;
};

//********************

typedef struct sector_s sector_t;
struct sector_s
{
	double max;
	double min;
};

typedef struct sector_para_s sector_para_t;
struct sector_para_s
{
	vector<point_t> dir;
	vector<point_t> dir_n;
	vector<double>  dist;
	int loc_int;
	int sec_int;
};

typedef struct data_s data_t;
struct data_s
{
	point_t pos;
	point_t vel;
	point_t acc;
};

typedef struct node_ss node_tt;
struct node_ss
{
	string 			name;
	unsigned int 	index;
	int 			category; //moving???
	point_t 		location;
	double 			boundary;
	int				surface;
	double 			surface_boundary;
	vector<data_t> 	data;
};

typedef struct edge_ss edge_tt;
struct edge_ss
{
	string 			name;
	unsigned int 	idx1;
	unsigned int 	idx2;
	vector<data_t> 	data;
	vector<double> 	sector_map; // locations int * sectors int
	vector<double> 	sector_const;
	vector<point_t> tan; // locations int
	vector<point_t> nor; // locations int
	vector<point_t> loc_start; // locations int
	vector<point_t> loc_mid; // locations int
	vector<point_t> loc_end; // locations int
	double 			total_len;
};

typedef struct label_s label_t;
struct label_s
{
	int 		mov;
	vector<int> loc;
	vector<int> sur; // surface
};

typedef struct pred_s pred_t;
struct pred_s
{
	vector<int> 	pred; // in or outside
	vector<double> 	pred_in; // prob shared between multiple predictions of inside
	vector<double> 	pred_in_last; // prob shared between multiple predictions of inside
	vector<double> 	pred_err; // prediction error = diff from the sectormap
};

typedef struct msg_s msg_t;
struct msg_s
{
	int 	msg; // msg option
	int 	idx;
	int 	loc_idx;
	int 	num_loc; // location
	int 	num_sur; // surface
	label_t label;
	pred_t 	pred;
};

typedef struct limit_s limit_t;
struct limit_s
{
	double vel;
	double sur_dis;
	double sur_ang;
};

#endif /* DATADECLARATION_H_ */
