/*
 * algo.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "algo.h"

//#define BSPLINE

//=============================================================================
// functions
//=============================================================================
/*
 std::vector<double> transposeInv(
 std::vector<double> A)
 {
 double det =	+ A[0]*(A[4]*A[8]-A[7]*A[5])
 - A[1]*(A[3]*A[8]-A[5]*A[6])
 + A[2]*(A[3]*A[7]-A[4]*A[6]);
 double invdet = 1/det;
 std::vector<double> B; B.resize(9);
 B[0] =  (A[4]*A[8]-A[7]*A[5])*invdet;
 B[3] = -(A[1]*A[8]-A[2]*A[7])*invdet;
 B[6] =  (A[1]*A[5]-A[2]*A[4])*invdet;
 B[1] = -(A[3]*A[8]-A[5]*A[6])*invdet;
 B[4] =  (A[0]*A[8]-A[2]*A[6])*invdet;
 B[7] = -(A[0]*A[5]-A[3]*A[2])*invdet;
 B[2] =  (A[3]*A[7]-A[6]*A[4])*invdet;
 B[5] = -(A[0]*A[7]-A[6]*A[1])*invdet;
 B[8] =  (A[0]*A[4]-A[3]*A[1])*invdet;
 return B;
 }*/

double pdfExp(
		double var,
		double mu,
		double x)
{
	return exp(-Sqr(x - mu) / (2 * var));
}

double normalPdf(
		double var,
		double mu,
		double x)
{
	return (1 / sqrt(2 * var * M_PI)) * exp(-Sqr(x - mu) / (2 * var));
}

Eigen::Vector4d average(
		const std::vector<Eigen::Vector4d> &A)
{
	Eigen::Vector4d result = Eigen::Vector4d::Zero();
	for (auto i : A)
	{
		result += i;
	}
	if (result.isZero())
		return result;
	else
		return (result / A.size());
}

Eigen::Vector4d movingAverage(
		const Eigen::Vector4d &a,
		std::vector<Eigen::Vector4d> &A)
{
	A.erase(A.begin());
	A.push_back(a);
	return average(A);
}

Eigen::Vector4d movingAverageIncrement(
		const Eigen::Vector4d &a,
		std::vector<Eigen::Vector4d> &A)
{
	A.push_back(a);
	return average(A);
}

Eigen::Vector3d rodriguezVec(
		const Eigen::AngleAxisd &aa_,
		const Eigen::Vector3d &vec_)
{
	return (vec_ * cos(aa_.angle()))
			+ ((aa_.axis()).cross(vec_) * sin(aa_.angle()))
			+ (aa_.axis() * ((aa_.axis()).dot(vec_)) * (1 - cos(aa_.angle())));
}

Eigen::Matrix3d rodriguezRot(
		const Eigen::Vector3d &vec_1,
		const Eigen::Vector3d &vec_2)
{
	double angle = acos(vec_1.dot(vec_2) / (vec_1.norm() * vec_2.norm()));
	Eigen::Vector3d axis = vec_1.cross(vec_2).normalized();
	Eigen::Matrix3d A;
	A(0, 0) = 0.0;
	A(0, 1) = -axis(2);
	A(0, 2) = axis(1);
	A(1, 0) = axis(2);
	A(1, 1) = 0.0;
	A(1, 2) = -axis(0);
	A(2, 0) = -axis(1);
	A(2, 1) = axis(0);
	A(2, 2) = 0.0;
	return Eigen::Matrix3d::Identity(3, 3) + (A * sin(angle))
			+ ((A * A) * (1 - cos(angle)));
}

void gaussKernel(
		std::vector<std::vector<double> > &kernel_,
		const int &numx_,
		const int &numy_,
		const double &var_)
{
	double sum = 0.0;
	for (int x = -(numx_ / 2); x <= (numx_ / 2); x++)
	{
		for (int y = -(numy_ / 2); y <= (numy_ / 2); y++)
		{
			kernel_[x + (numx_ / 2)][y + (numy_ / 2)] = (1 / (2 * M_PI * var_))
					* exp(-(Sqr(sqrt(Sqr(x) + Sqr(y))) / (2 * var_)));
			sum += kernel_[x + (numx_ / 2)][y + (numy_ / 2)];
		}
	}
	for (auto &i : kernel_)
	{
		for (auto &j : i)
		{
			j /= sum;
		}
	}
}

void cal_tangent_normal(
		Eigen::Vector3d &p_tan_,
		Eigen::Vector3d &p_nor_,
		const double &t_mid_,
		const std::vector<Eigen::Vector3d> &coeff,
		bool normal)
{
	Eigen::Vector3d out1(Eigen::Vector3d::Zero());
	Eigen::Vector3d out2(Eigen::Vector3d::Zero());
	Eigen::Vector3d out4(Eigen::Vector3d::Zero());

	int i = 0;
	for (auto c : coeff)
	{
		out1 += (i * c * pow(t_mid_, i - 1));
		i++;
	}
	p_tan_ = out1;

	if (normal)
	{
		i = 0;
		for (auto c : coeff)
		{
			out2 += (i * (i - 1) * c * pow(t_mid_, i - 2));
			i++;
		}
		double out3N = (out1.cwiseProduct(out2)).sum() * 2;
		out4 = ((out1.norm() * out2) - (0.5 * out1 * out3N * (1 / out1.norm())))
				/ Sqr(out1.norm());
		p_nor_ = out4;
	}
	else
	{
		p_nor_ = out4;
	}
}
/*
 double determinant(
 std::vector<std::vector<double> > x)
 {
 int sign = 0;
 int * signum = &sign;
 double det = 0.0;
 gsl_permutation * p = gsl_permutation_calloc(x.size());
 gsl_matrix * A = gsl_matrix_calloc(x.size(), x.size());
 for(int j=0;j<x.size();j++)
 {
 for(int i=0;i<x.size();i++)
 {
 gsl_matrix_set(A, i, j, x[j][i]);
 }
 }
 gsl_linalg_LU_decomp(A, p, signum);
 det = gsl_linalg_LU_det(A, *signum);
 gsl_permutation_free(p);
 gsl_matrix_free(A);
 return det;
 }
 */
// ============================================================================
// B-spline
// ============================================================================
#ifdef NEVER
// example
void curveFit(
		std::vector<point_d> points_,
		std::vector<point_d> &curves_)
{
	const size_t n = points_.size();
	const size_t ncoeffs = NCOEFFS;
	const size_t nbreak = NBREAK;

	size_t i, j;
	gsl_bspline_workspace *bw;
	gsl_vector *B;
	gsl_vector *c, *w;
	gsl_vector *x, *y;
	gsl_vector *yerr;
	gsl_matrix *X, *cov;
	gsl_multifit_linear_workspace *mw;
	double chisq;

	for(int ii=0;ii<3;ii++)
	{
		/* allocate a cubic bspline workspace (k = 4) */
		bw = gsl_bspline_alloc(4, nbreak);
		B = gsl_vector_alloc(ncoeffs);

		x = gsl_vector_alloc(n);
		y = gsl_vector_alloc(n);
		X = gsl_matrix_alloc(n, ncoeffs);
		c = gsl_vector_alloc(ncoeffs);
		w = gsl_vector_alloc(n);
		cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
		mw = gsl_multifit_linear_alloc(n, ncoeffs);
		yerr = gsl_vector_alloc(n);

		/* this is the data to be fitted */
		for (i=0;i<n;++i)
		{
			gsl_vector_set(x, i, double(i));
			switch (ii)
			{
				case 0:
				gsl_vector_set(y, i, points_[i].x);
				break;
				case 1:
				gsl_vector_set(y, i, points_[i].y);
				break;
				case 2:
				gsl_vector_set(y, i, points_[i].z);
				break;
			}
		}

		/* use uniform breakpoints on [0, 15] */
		gsl_bspline_knots_uniform(0.0, (double)n, bw);

		/* construct the fit matrix X */
		for (i=0;i<n;++i)
		{
			double xi = gsl_vector_get(x, i);

			/* compute B_j(xi) for all j */
			gsl_bspline_eval(xi, B, bw);

			/* fill in row i of X */
			for (j = 0; j < ncoeffs; ++j)
			{
				double Bj = gsl_vector_get(B, j);
				gsl_matrix_set(X, i, j, Bj);
			}
		}

		/* do the fit */
		gsl_multifit_linear(X, y, c, cov, &chisq, mw);

		/* output the smoothed curve */
		for (i=0;i<n;i++)
		{
			double yerr_tmp;
			double xi = gsl_vector_get(x, i);
			gsl_bspline_eval(xi, B, bw);
			switch (ii)
			{
				case 0:
				gsl_multifit_linear_est(B, c, cov, &curves_[i].x, &yerr_tmp);
				break;
				case 1:
				gsl_multifit_linear_est(B, c, cov, &curves_[i].y, &yerr_tmp);
				break;
				case 2:
				gsl_multifit_linear_est(B, c, cov, &curves_[i].z, &yerr_tmp);
				break;
			}
			gsl_vector_set(yerr, i, yerr_tmp);
		}

		gsl_bspline_free(bw);
		gsl_vector_free(B);
		gsl_vector_free(x);
		gsl_vector_free(y);
		gsl_matrix_free(X);
		gsl_vector_free(c);
		gsl_vector_free(w);
		gsl_matrix_free(cov);
		gsl_multifit_linear_free(mw);
	}
}
#endif

void polyCurveFit(
		const std::vector<double> &points_,
		std::vector<double> &coeff_,
		std::vector<double> &cov_,
		int DEGREE_)
{

#ifdef BSPLINE

	int num_points = points_.size();
	double chisq; //residual error

	gsl_bspline_workspace *bw;
	gsl_multifit_linear_workspace *mws;
	gsl_matrix *cov, *X;
	gsl_vector *y, *c, *B;

	B = gsl_vector_alloc(NCOEFFS);
	X = gsl_matrix_alloc(num_points, NCOEFFS);
	y = gsl_vector_alloc(num_points);
	c = gsl_vector_alloc(NCOEFFS);
	cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);
	bw = gsl_bspline_alloc(4, NBREAK);
	mws = gsl_multifit_linear_alloc(num_points, NCOEFFS);

	gsl_bspline_knots_uniform(0.0, num_points, bw);

	for(int i=0;i<num_points;i++)
	{
		gsl_bspline_eval((double)i, B, bw);
		for (int j = 0; j < NCOEFFS; ++j)
		{
			double Bj = gsl_vector_get(B, j);
			gsl_matrix_set(X, i, j, Bj);
		}
		gsl_vector_set(y, i, points_[i]);
	}

	gsl_multifit_linear(X, y, c, cov, &chisq, mws);

	reshapeVector(coeff_, NCOEFFS);
	for(int i=0;i<NCOEFFS;i++)
	{	coeff_[i] = gsl_vector_get(c, i);}
	reshapeVector(cov_, Sqr(NCOEFFS));
	for(int i=0;i<Sqr(NCOEFFS);i++)
	{
		cov_[i] = gsl_matrix_get(cov, i/NCOEFFS, i%NCOEFFS);
	}

	// for visualizing the curve-fitting
	if(0)
	{
		std::vector<double> y0(num_points);
		{
			double yerr;
			for(int i=0;i<num_points;i++)
			{
				gsl_bspline_eval((double)i, B, bw);
				gsl_multifit_linear_est(B, c, cov, &y0[i], &yerr);
			}
		}
		std::vector<double> x0;
		for(int i=0;i<num_points;i++)
		{	x0.push_back(i);}
		plotData(x0, points_, x0, y0);
	}

	gsl_multifit_linear_free(mws);
	gsl_bspline_free(bw);
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(c);
	gsl_vector_free(B);

#else

	gsl_multifit_linear_workspace *mws;
	gsl_matrix *cov, *X;
	gsl_vector *y, *c;
	double chisq; //residual error
	int num_points = points_.size();
	X = gsl_matrix_alloc(num_points, DEGREE_);
	y = gsl_vector_alloc(num_points);
	c = gsl_vector_alloc(DEGREE_);
	cov = gsl_matrix_alloc(DEGREE_, DEGREE_);
	for (int i = 0; i < num_points; i++)
	{
		for (int j = 0; j < DEGREE_; j++)
		{
			gsl_matrix_set(X, i, j, pow(i, j));
		}
		gsl_vector_set(y, i, points_[i]);
	}
	mws = gsl_multifit_linear_alloc(num_points, DEGREE_);
	gsl_multifit_linear(X, y, c, cov, &chisq, mws);
	reshapeVector(coeff_, DEGREE_);
	for (int i = 0; i < DEGREE_; i++)
	{
		coeff_[i] = gsl_vector_get(c, i);
	}
	reshapeVector(cov_, Sqr(DEGREE_));
	for (int i = 0; i < Sqr(DEGREE_); i++)
	{
		cov_[i] = gsl_matrix_get(cov, i / DEGREE_, i % DEGREE_);
	}
	gsl_multifit_linear_free(mws);
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(c);

#endif

}

void polyCurveFitEst(
		std::vector<double> &points_,
		int num_points_,
		const std::vector<double> &coeffs_,
		const std::vector<double> &covs_,
		int DEGREE_)
{

#ifdef BSPLINE

	gsl_matrix *cov, *X;
	gsl_vector *c, *Xj, *B;
	gsl_bspline_workspace *bw;

	B = gsl_vector_alloc(NCOEFFS);
	bw = gsl_bspline_alloc(4, NBREAK);

	double cc[NCOEFFS];
	X = gsl_matrix_alloc(num_points_, NCOEFFS);
	Xj = gsl_vector_alloc(NCOEFFS);
	c = gsl_vector_alloc(NCOEFFS);
	cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);

	for(int i=0;i<NCOEFFS;i++)
	{	gsl_vector_set(c, i, coeffs_[i]);}

	for(int i=0;i<Sqr(NCOEFFS);i++)
	{
		gsl_matrix_set(cov, i/NCOEFFS, i%NCOEFFS, covs_[i]);
	}

	gsl_bspline_knots_uniform(0.0, points_.size(), bw);

	double yerr;
	for(int i=0;i<points_.size();i++)
	{
		gsl_bspline_eval((double)i, B, bw);
		gsl_multifit_linear_est(B, c, cov, &points_[i], &yerr);
	}

	// Vizualize
	if(0)
	{
		std::vector<double> x0;
		for(int i=0;i<points_.size();i++)
		{	x0.push_back(i);}
		plotData(x0, points_);
	}

	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(Xj);
	gsl_vector_free(c);
	gsl_vector_free(B);

#else

	gsl_matrix *cov, *X;
	gsl_vector *c, *Xj;
	double cc[DEGREE_];
	X = gsl_matrix_alloc(num_points_, DEGREE_);
	Xj = gsl_vector_alloc(DEGREE_);
	c = gsl_vector_alloc(DEGREE_);
	cov = gsl_matrix_alloc(DEGREE_, DEGREE_);
	for (int i = 0; i < DEGREE_; i++)
	{
		cc[i] = coeffs_[i];
	}
	for (int i = 0; i < points_.size(); i++)
	{
		points_[i] = gsl_poly_eval(cc, DEGREE_,
				((double) i + 1) / (points_.size() / num_points_));
	}
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(Xj);
	gsl_vector_free(c);

#endif

}

void polyCurveFitPoint(
		const std::vector<Eigen::Vector4d> &points_,
		std::vector<Eigen::Vector4d> &points_est_,
		std::vector<Eigen::Vector3d> &coeffs_,
		std::vector<Eigen::Vector3d> &covs_,
		int DEGREE_,
		bool flag_est_)
{
	int num_points = points_.size();
	std::vector<double> x;
	x.resize(num_points);
	std::vector<double> y;
	y.resize(num_points);
	std::vector<double> z;
	z.resize(num_points);
	std::vector<double> cx;
	cx.resize(DEGREE_);
	std::vector<double> cy;
	cy.resize(DEGREE_);
	std::vector<double> cz;
	cz.resize(DEGREE_);
	std::vector<double> covx;
	covx.resize(Sqr(DEGREE_));
	std::vector<double> covy;
	covy.resize(Sqr(DEGREE_));
	std::vector<double> covz;
	covz.resize(Sqr(DEGREE_));
	for (int i = 0; i < num_points; i++)
	{
		x[i] = points_[i](0);
		y[i] = points_[i](1);
		z[i] = points_[i](2);
	}
	polyCurveFit(x, cx, covx, DEGREE_);
	polyCurveFit(y, cy, covy, DEGREE_);
	polyCurveFit(z, cz, covz, DEGREE_);
	reshapeVector(coeffs_, DEGREE_);
	for (int i = 0; i < DEGREE_; i++)
	{
		coeffs_[i](0) = cx[i];
		coeffs_[i](1) = cy[i];
		coeffs_[i](2) = cz[i];
	}
	reshapeVector(covs_, Sqr(DEGREE_));
	for (int i = 0; i < Sqr(DEGREE_); i++)
	{
		covs_[i](0) = covx[i];
		covs_[i](1) = covy[i];
		covs_[i](2) = covz[i];
	}
	int num_points2 = points_est_.size();
	std::vector<double> x2;
	x2.resize(num_points2);
	std::vector<double> y2;
	y2.resize(num_points2);
	std::vector<double> z2;
	z2.resize(num_points2);
	if (flag_est_)
	{
		polyCurveFitEst(x2, num_points, cx, covx, DEGREE_);
		polyCurveFitEst(y2, num_points, cy, covy, DEGREE_);
		polyCurveFitEst(z2, num_points, cz, covz, DEGREE_);
		for (int i = 0; i < num_points2; i++)
		{
			points_est_[i](0) = x2[i];
			points_est_[i](1) = y2[i];
			points_est_[i](2) = z2[i];
			points_est_[i](3) = -1;
		}
		// ##TODO hack
		for (int i = 0; i < 4; i++)
		{
			points_est_.erase(points_est_.begin());
			points_est_.pop_back();
		}
	}
}

double curveIntegral(
		double x,
		void *params)
{
	double *cc = (double *) params;
	int DEGREE = cc[0];
	double dfx[2], dfy[2], dfz[2];
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	for (int i = 0; i < DEGREE; i++)
	{
		cx[i] = cc[i * 3 + 0 + 1];
		cy[i] = cc[i * 3 + 1 + 1];
		cz[i] = cc[i * 3 + 2 + 1];
	}
	gsl_poly_eval_derivs(cx, DEGREE, x, dfx, 2);
	gsl_poly_eval_derivs(cy, DEGREE, x, dfy, 2);
	gsl_poly_eval_derivs(cz, DEGREE, x, dfz, 2);
	double f = sqrt(Sqr(dfx[1]) + Sqr(dfy[1]) + Sqr(dfz[1]));
	return f;
}

void polyCurveLength(
		double &length_,
		double a_,
		double b_,
		const std::vector<Eigen::Vector3d> &coeffs_,
		int DEGREE_)
{
	gsl_function F;
	gsl_integration_glfixed_table *table;
	double cc[3 * DEGREE_ + 1];
	cc[0] = DEGREE_;
	table = gsl_integration_glfixed_table_alloc(DEGREE_ * 2);
//	table = gsl_integration_glfixed_table_alloc((DEGREE_+1)/2);
	for (int i = 0; i < DEGREE_; i++)
	{
		for (int ii = 0; ii < 3; ii++)
		{
			cc[i * 3 + ii + 1] = coeffs_[i](ii);
		}
	}
	F.function = &curveIntegral;
	F.params = cc;
	length_ = gsl_integration_glfixed(&F, a_, b_, table);
	gsl_integration_glfixed_table_free(table);
}
