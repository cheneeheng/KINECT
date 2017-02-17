/*
 * algo.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef ALGO_H_
#define ALGO_H_

#include "dataDeclaration.h"

//=============================================================================
// functions

double l2Norm(vector<double> A);

double l2Norm(point_t A);

double pdfExp(
	double var,
	double mu,
	double x);

double normalPdf(
	double var,
	double mu,
	double x);

vector<double> addVector(
	vector<double> A,
	vector<double> B);

vector<double> minusVector(
	vector<double> A,
	vector<double> B);

point_t minusPoint(
	point_t A,
	point_t B);

point_t addPoint(
	point_t A,
	point_t B);

vector<double> crossProduct(
	vector<double> A,
	vector<double> B);

double dotProduct(
	vector<double> A,
	vector<double> B);

void normalization(vector<double> &data_);

double average(vector<double> &A);

double movingAverage(
	double a,
	vector<double> &A);

void averagePoint(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg);

void averagePointIncrement(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg);

void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_);

//=============================================================================
// inline

static inline int min (int x,int y) { return (x<y)?x:y; }

static inline int max (int x,int y) { return (x>y)?x:y; }

static inline double min (double x,double y) { return (x<y)?x:y; }

static inline double max (double x,double y) { return (x>y)?x:y; }

static inline bool min_ (double x,double y) { return (x<y)?true:false; }

static inline bool max_ (double x,double y) { return (x>y)?true:false; }

static inline point_t min (point_t x, point_t y)
{
	return (l2Norm(x)<l2Norm(y)) ? x:y;
}

static inline vector<double> point2vector(point_t A)
{
	vector<double> B(3);
	B[0]=A.x;
	B[1]=A.y;
	B[2]=A.z;
	return B;
}

static inline point_t vector2point(vector<double> A)
{
	point_t B;
	B.x=A[0];
	B.y=A[1];
	B.z=A[2];
	B.z=UNCLASSIFIED;
	return B;
}

template<typename T> void vector2array(vector<T> A, T *B)
{
	for(int i=0;i<A.size();i++) B[i] = A[i];
}

template<typename T> void array2vector(T *A, int size, vector<T> B)
{
	for(int i=0;i<size;i++) B[i] = A[i];
}

#endif /* ALGO_H_ */
