/*
 * algo.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "algo.h"

double l2Norm(vector<double> A)
{
    double a=0.0;
    for (unsigned int i=0;i<A.size();i++)
        a+=Sqr(A[i]);
    return sqrt(a);
}

double l2Norm(point_t A)
{
    return sqrt(Sqr(A.x)+Sqr(A.y)+Sqr(A.z));
}

double normalPdf(
	double var,
	double mu,
	double x)
{
	return (1/sqrt(2*var*M_PI)) * exp( - Sqr(x-mu)/(2*var) );
}

double pdfExp(
	double var,
	double mu,
	double x)
{
	return exp( - Sqr(x-mu)/(2*var) );
}

vector<double> addVector(
	vector<double> A,
	vector<double> B)
{
	vector<double> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]+B[i]);
	return C;
}

vector<double> minusVector(
	vector<double> A,
	vector<double> B)
{
	vector<double> C;
	for(int i=0;i<A.size();i++)
		C.push_back(A[i]-B[i]);
	return C;
}

point_t addPoint(
	point_t A,
	point_t B)
{
	point_t C;
	C.x = A.x + B.x;
	C.y = A.y + B.y;
	C.z = A.z + B.z;
	C.cluster_id = UNCLASSIFIED;
	return C;
}

point_t minusPoint(
	point_t A,
	point_t B)
{
	point_t C;
	C.x = A.x - B.x;
	C.y = A.y - B.y;
	C.z = A.z - B.z;
	C.cluster_id = UNCLASSIFIED;
	return C;
}

vector<double> crossProduct(
	vector<double> A,
	vector<double> B)
{
	vector<double> C(3);
	C[0] = A[1]*B[2] - A[2]*B[1];
	C[1] = A[2]*B[0] - A[0]*B[2];
	C[2] = A[0]*B[1] - A[1]*B[0];
	if(C[0]*C[0]+C[1]*C[1]+C[2]*C[2] == 0){ // prevent degenerate case
		printf("[WARNING] : CROSS PRODUCT VECTORS ARE COLLINEAR !!!\n");
		C[0]=0; C[1]=0; C[2]=0;
	}
	if(A[0] == 0 && A[1] == 0 && A[2] == 0)
		printf("[WARNING] : CROSS PRODUCT VECTOR 1 IS A ZERO VECTOR !!!\n");
	if(B[0] == 0 && B[1] == 0 && B[2] == 0)
		printf("[WARNING] : CROSS PRODUCT VECTOR 2 IS A ZERO VECTOR !!!\n");
	return C;
}

double dotProduct(
	vector<double> A,
	vector<double> B)
{
	double ans;
	vector<double> C(3);
	C[0] = A[0]*B[0];
	C[1] = A[1]*B[1];
	C[2] = A[2]*B[2];
	ans = C[0]+C[1]+C[2];
	if(A[0] == 0 && A[1] == 0 && A[2] == 0)
		printf("[WARNING] : DOT PRODUCT VECTOR 1 IS A ZERO VECTOR !!!\n");
	if(B[0] == 0 && B[1] == 0 && B[2] == 0)
		printf("[WARNING] : DOT PRODUCT VECTOR 2 IS A ZERO VECTOR !!!\n");
	return ans;
}

void normalization(vector<double> &data_)
{
	double tmp = 0.0;
	for(int i=0;i<data_.size();i++)
		tmp += data_[i];
	for(int i=0;i<data_.size();i++)
		data_[i]/=tmp;
}

double average(vector<double> &A)
{
	double avg = 0.0;
	for (int i=0;i<A.size();i++)
		avg += A[i];
	avg = avg / A.size();
	return avg;
}

double movingAverage(
	double a,
	vector<double> &A)
{
	for(int i=0;i<A.size()-1;i++)
		A[i] = A[i+1];
	A[A.size()-1] = a;
	double avg = average(A);
	return avg;
}

void averagePoint(
	point_t X,
	vector<vector<double> > &X_tmp, // xyz, win length
	point_t &Xavg)
{
	Xavg.x =  movingAverage(X.x, X_tmp[0]);
	Xavg.y =  movingAverage(X.y, X_tmp[1]);
	Xavg.z =  movingAverage(X.z, X_tmp[2]);
}

void averagePointIncrement(
	point_t X,
	vector<vector<double> > &X_tmp,
	point_t &Xavg)
{
	if(X_tmp[0].empty())
	{
		X_tmp[0].push_back(X.x);
		X_tmp[1].push_back(X.y);
		X_tmp[2].push_back(X.z);
		Xavg.x =  X.x;
		Xavg.y =  X.y;
		Xavg.z =  X.z;
	}
	else
	{
		X_tmp[0].push_back(X.x);
		X_tmp[1].push_back(X.y);
		X_tmp[2].push_back(X.z);
		Xavg.x =  average(X_tmp[0]);
		Xavg.y =  average(X_tmp[1]);
		Xavg.z =  average(X_tmp[2]);
	}
}


void gaussKernel(
	vector<vector<double> > &kernel_,
	int numx_,
	int numy_,
	double var_)
{
    double sum = 0.0;
    for (int x = -(numx_/2); x <= (numx_/2); x++)
    {
        for(int y = -(numy_/2); y <= (numy_/2); y++)
        {
        	kernel_[x+(numx_/2)][y+(numy_/2)] =
        			(1/(2*M_PI*var_)) *
        			exp(-(Sqr(sqrt(Sqr(x) + Sqr(y)))/(2*var_)));
            sum += kernel_[x+(numx_/2)][y+(numy_/2)];
        }
    }
    for(int i = 0; i < numx_; ++i)
        for(int j = 0; j < numy_; ++j)
        	kernel_[i][j] /= sum;
}

