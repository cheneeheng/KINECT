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
#include <fstream>
#include <sys/stat.h>
#include <stack>
#include <ctime>

#include <opencv2/opencv.hpp>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
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
#include <vtkParametricFunctionSource.h>
#include <vtkTupleInterpolator.h>
#include <vtkTubeFilter.h>
#include <vtkParametricSpline.h>
#include <vtkDoubleArray.h>
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
using namespace cv;

#define Sqr(x) ((x)*(x))
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define Calloc(type,n) (type *)calloc( n, sizeof(type))

//0 : all
//1 : motion
//2 : location
//3 : label only
#define VERBOSE 0

// number of fit coefficients
// nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4
#define NCOEFFS	12
#define NBREAK 	(NCOEFFS - 2)
#define DEGREE 5 //k+1

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
	vector<data_t> 	data;
};

typedef struct edge_ss edge_tt;
struct edge_ss
{
	string 			name;
	unsigned int 	begin_index;
	unsigned int 	end_index;
	vector<data_t> 	data;
	vector<double> 	sector_map; // locations int * sectors int
	vector<double> 	sector_const;
	vector<point_t> tangent; // locations int
	vector<point_t> normal; // locations int
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
	vector<int> surface;
};

#endif /* DATADECLARATION_H_ */
