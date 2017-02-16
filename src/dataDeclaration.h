/*
 * dataDeclaration.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef DATADECLARATION_H_
#define DATADECLARATION_H_

#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <list>
#include <math.h>
#include <algorithm>
#include <vector>
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
#include <iostream>

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

#include <XVImageSeq.h>
#include <XVMpeg.h>
#include <XVImageIO.h>
#include <XVColorSeg.h>
#include <XVBlobFeature.h>
#include <XVTracker.h>
#include <XVWindowX.h>

#include "ippi.h"
#include "ippcv.h"
#include "ippcc.h"
/*
#include "../aruco/markerdetector.cpp"
#include "../aruco/marker.cpp"
#include "../aruco/ar_omp.cpp"
#include "../aruco/subpixelcorner.cpp"
#include "../aruco/cameraparameters.cpp"
#include "../aruco/arucofidmarkers.cpp"
*/
using namespace std;
using namespace cv;


//******************** TAKEN FROM .....
#define UNCLASSIFIED -1
#define NOISE -2

#define CORE_POINT 1
#define NOT_CORE_POINT 0

#define SUCCESS 0
#define FAILURE -3

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
	string name;
	unsigned int index;
	unsigned int category;
	int surface_num;
	vector<data_t> data;
};

typedef struct edge_ss edge_tt;
struct edge_ss
{
	unsigned int begin_index;
	unsigned int end_index;
	double cost;
	vector<data_t> data;
};

#endif /* DATADECLARATION_H_ */
