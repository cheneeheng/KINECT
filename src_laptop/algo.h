/*
 * algo.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef ALGO_H_
#define ALGO_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Eigen>

#include <gsl/gsl_integration.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_linalg.h>

template<typename T>
static inline T Sqr(
		const T &A)
{
	return A * A;
}

static inline Eigen::Vector4d V3d4d(
		const Eigen::Vector3d &A)
{
	Eigen::Vector4d B;
	B(0) = A(0);
	B(1) = A(1);
	B(2) = A(2);
	B(3) = 0.0;
	return B;
}

static inline Eigen::Vector3d V4d3d(
		const Eigen::Vector4d &A)
{
	Eigen::Vector3d B;
	B(0) = A(0);
	B(1) = A(1);
	B(2) = A(2);
	return B;
}

template<typename T> static inline bool min_(
		T x,
		T y)
{
	return (x < y) ? true : false;
}
template<typename T> static inline bool max_(
		T x,
		T y)
{
	return (x > y) ? true : false;
}

template<typename T>
void reshapeVector(
		std::vector<T> &A,
		const int &size)
{
	A.clear();
	A.resize(size);
}

template<typename T>
void vectorToArray(
		const std::vector<T> &A,
		T *B)
{
	int c = 0;
	for (auto i : A)
	{
		B[c] = i;
		c++;
	}
}

template<typename T>
void arrayTovector(
		T *A,
		std::vector<T> &B)
{
	//reshapeVector(B, std::extent<decltype(A)>::value);
	B.clear();
	for (auto i : A)
	{
		B.push_back(i);
	}
}

//template<typename T>
//std::vector<T> addvector(
//		std::vector<T> A,
//		std::vector<T> B)
//{
//	std::vector<T> C;
//	for(int i=0;i<A.size();i++) { C.push_back(A[i]+B[i]); }
//	return C;
//}
//
//template<typename T>
//std::vector<T> minusvector(
//		std::vector<T> A,
//		std::vector<T> B)
//{
//	std::vector<T> C;
//	for(int i=0;i<A.size();i++) { C.push_back(A[i]-B[i]); }
//	return C;
//}

template<typename T>
T l2Norm(
		std::vector<T> A)
{
	double a = 0.0;
	for (auto i : A)
	{
		a += Sqr(i);
	}
	return sqrt(a);
}

double pdfExp(
		double var,
		double mu,
		double x);

double normalPdf(
		double var,
		double mu,
		double x);

template<typename T>
T addFunction(
		const T &x,
		const T &y)
{
	return fabs(x) + fabs(y);
}

template<typename T>
T average(
		const std::vector<T> &A)
{
	if (accumulate(A.begin(), A.end(), 0.0, addFunction<T>) == 0.0)
		return 0.0;
	else
		return accumulate(A.begin(), A.end(), 0.0, addFunction<T>) / A.size();
}

template<typename T>
T movingAverage(
		const T &a,
		std::vector<T> &A)
{
	A.erase(A.begin());
	A.push_back(a);
	return average(A);
}

template<typename T>
T movingAverageIncrement(
		const T &a,
		std::vector<T> &A)
{
	A.push_back(a);
	return average(A);
}

Eigen::Vector4d average(
		const std::vector<Eigen::Vector4d> &A);

Eigen::Vector4d movingAverage(
		const Eigen::Vector4d &a,
		std::vector<Eigen::Vector4d> &A);

Eigen::Vector4d movingAverageIncrement(
		const Eigen::Vector4d &a,
		std::vector<Eigen::Vector4d> &A);

void gaussKernel(
		std::vector<std::vector<double> > &kernel_,
		const int &numx_,
		const int &numy_,
		const double &var_);

Eigen::Vector3d rodriguezVec(
		const Eigen::AngleAxisd &aa_,
		const Eigen::Vector3d &vec_);

Eigen::Matrix3d rodriguezRot(
		const Eigen::Vector3d &vec_1,
		const Eigen::Vector3d &vec_2);

void cal_tangent_normal(
		Eigen::Vector3d &p_tan_,
		Eigen::Vector3d &p_nor_,
		const double &t_mid_,
		const std::vector<Eigen::Vector3d> &coeff,
		bool normal);

// ============================================================================
// B-spline
// ============================================================================

double curveIntegral(
		double x,
		void *params);

void polyCurveFit(
		const std::vector<double> &points_,
		std::vector<double> &coeff_,
		std::vector<double> &cov_,
		int DEGREE_);

void polyCurveFitPoint(
		const std::vector<Eigen::Vector4d> &points_,
		std::vector<Eigen::Vector4d> &points_est_,
		std::vector<Eigen::Vector3d> &coeffs_,
		std::vector<Eigen::Vector3d> &covs_,
		int DEGREE_,
		bool flag_est_);

void polyCurveFitEst(
		std::vector<double> &points_,
		int num_points_,
		const std::vector<double> &coeffs_,
		const std::vector<double> &covs_,
		int DEGREE_);

void polyCurveLength(
		double &length_,
		double a_,
		double b_,
		const std::vector<Eigen::Vector3d> &coeffs_,
		int DEGREE_);

#endif /* ALGO_H_ */
