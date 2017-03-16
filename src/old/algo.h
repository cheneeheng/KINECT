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

point_t minusPoint(
	point_t A,
	point_t B);

point_t addPoint(
	point_t A,
	point_t B);

point_t multiPoint(
	point_t A,
	double B);

vector<double> crossProduct(
	vector<double> A,
	vector<double> B);

double dotProduct(
	vector<double> A,
	vector<double> B);

point_t movingAverage(
	point_t a,
	vector<point_t> &A);

point_t averagePoint(vector<point_t> A);

point_t averagePointIncrement(
	point_t A,
	vector< point_t > &A_mem);

void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_);

point_t rodriguezVec(
	double angle_,
	point_t axis_,
	point_t vec_);

vector<double> rodriguezRot(
	point_t vec_1,
	point_t vec_2);

vector<double> transInv(
		vector<double> A);

void cal_tangent_normal(
	double t_mid_,
	point_t &p_tan_,
	point_t &p_nor_,
	vector<point_t> coeff,
	int dim,
	bool normal);

//=============================================================================
// inline

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
	B.cluster_id=UNCLASSIFIED;
	return B;
}

template<typename T>
static inline bool min_ (T x,T y) { return (x<y)?true:false; }

template<typename T>
static inline bool max_ (T x,T y) { return (x>y)?true:false; }

//=============================================================================
// template

template<typename T>
void vector2array(vector<T> A, T *B)
{
	for(int i=0;i<A.size();i++) B[i] = A[i];
}

template<typename T>
void array2vector(T *A, int size, vector<T> &B)
{
	for(int i=0;i<size;i++) B[i] = A[i];
}

template<typename T>
void reshapeVector(vector<T> &A, int size)
{
	A.clear();
	A.resize(size);
}

template<typename T>
T average(vector<T> A)
{
	T avg = accumulate( A.begin(), A.end(), 0.0)/A.size();
	return avg;
}

template<typename T>
vector<T> addVector(vector<T> A, vector<T> B)
{
	vector<T> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]+B[i]);
	return C;
}

template<typename T>
vector<T> minusVector(vector<T> A, vector<T> B)
{
	vector<T> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]-B[i]);
	return C;
}

// DATA IS ASSUMED TO BE POSITIVE ONLY
template<typename T>
void normalizeData(vector<T> &data_)
{
	T tmp = 0;
	for(int i=0;i<data_.size();i++)
		tmp += data_[i];
	if (tmp>0)
		for(int i=0;i<data_.size();i++)
			data_[i]/=tmp;
//	else
//		printf("[WARNING] : Data is empty.\n");
}

#endif /* ALGO_H_ */
