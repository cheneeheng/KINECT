/*
 * algo.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "algo.h"

//=============================================================================
// functions
//=============================================================================

double l2Norm(
	vector<double> A)
{
    double a=0.0;
    for (unsigned int i=0;i<A.size();i++)
        a+=Sqr(A[i]);
    return sqrt(a);
}

double l2Norm(
	point_t A)
{
    return sqrt(Sqr(A.x)+Sqr(A.y)+Sqr(A.z));
}

double pdfExp(
	double var,
	double mu,
	double x)
{
	return exp( - Sqr(x-mu)/(2*var) );
}

double invPdfExp(
	double var,
	double mu,
	double x)
{
	return sqrt(-(log(x)*2*var));
}

double normalPdf(
	double var,
	double mu,
	double x)
{
	return (1/sqrt(2*var*M_PI)) * exp( - Sqr(x-mu)/(2*var) );
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

point_t multiPoint(
	point_t A,
	double  B)
{
	point_t C;
	C.x = A.x*B;
	C.y = A.y*B;
	C.z = A.z*B;
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
//		printf("[WARNING] : CROSS PRODUCT VECTORS ARE COLLINEAR !!!\n");
		C[0]=0; C[1]=0; C[2]=0;
	}
//	if(A[0] == 0 && A[1] == 0 && A[2] == 0)
//		printf("[WARNING] : CROSS PRODUCT VECTOR 1 IS A ZERO VECTOR !!!\n");
//	if(B[0] == 0 && B[1] == 0 && B[2] == 0)
//		printf("[WARNING] : CROSS PRODUCT VECTOR 2 IS A ZERO VECTOR !!!\n");
	return C;
}

double dotProduct(
	vector<double> A,
	vector<double> B)
{
	double ans = A[0]*B[0] + A[1]*B[1] + A[2]*B[2];
//	if(A[0] == 0 && A[1] == 0 && A[2] == 0)
//		printf("[WARNING] : DOT PRODUCT VECTOR 1 IS A ZERO VECTOR !!!\n");
//	if(B[0] == 0 && B[1] == 0 && B[2] == 0)
//		printf("[WARNING] : DOT PRODUCT VECTOR 2 IS A ZERO VECTOR !!!\n");
	return ans;
}

point_t movingAverage(
	point_t a,
	vector<point_t> &A)
{
	for(int i=0;i<A.size()-1;i++)
		A[i] = A[i+1];
	A[A.size()-1] = a;
	point_t avg = averagePoint(A);
	return avg;
}

point_t averagePoint(
	vector<point_t> A)
{
	point_t avg;
	avg.x = avg.y = avg.z = 0;
	for (int i=0;i<A.size();i++)
	{
		avg.x += A[i].x;
		avg.y += A[i].y;
		avg.z += A[i].z;
	}
	avg.x /= A.size();
	avg.y /= A.size();
	avg.z /= A.size();
	return avg;
}

point_t averagePointIncrement(
	point_t A,
	vector< point_t > &A_mem)
{
	vector< point_t > tmp = A_mem;
	tmp.push_back(A);
	point_t avg = averagePoint(tmp);
	A_mem.push_back(avg);
	return avg;
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

point_t rodriguezVec(
	double angle_,
	point_t axis_,
	point_t vec_)
{
	point_t out1;
	point_t N1, N2, N3;
	N1 = multiPoint(vec_,cos(angle_));
	N2 = multiPoint(vector2point(crossProduct(point2vector(axis_),
											  point2vector(vec_ ))),
					sin(angle_));
	N3 = multiPoint(multiPoint(axis_,
							   dotProduct(point2vector(axis_),
										  point2vector(vec_ ))),
					1 - cos(angle_));
	out1 = addPoint(addPoint(N1,N2),N3);
	return out1;
}

vector<double> rodriguezRot(
	point_t vec_1,
	point_t vec_2)
{
	vector<double> axis(3), A(9), A2(9), R(9); //row major
	double angle, axis_norm, angle_norm;
	axis 		 = crossProduct(point2vector(vec_1), point2vector(vec_2));
	axis_norm 	 = l2Norm(axis);
	axis[0]		/= axis_norm;
	axis[1]		/= axis_norm;
	axis[2]		/= axis_norm;
	angle_norm	 = dotProduct(point2vector(vec_1),point2vector(vec_2)) /
				   (l2Norm(vec_1) * l2Norm(vec_2));
	angle 		 = acos(angle_norm);
	A[0] = 0;
	A[1] = -axis[2];
	A[2] =  axis[1];
	A[3] =  axis[2];
	A[4] = 0;
	A[5] = -axis[0];
	A[6] = -axis[1];
	A[7] =  axis[0];
	A[8] = 0;
	for(int i=0;i<9;i++)
	{
		A2[i] = 0;
		for(int ii=0;ii<3;ii++)
			A2[i] += A[(i/3)*3 + ii] * A[(i%3) + (ii*3)];
	}
	for(int i=0;i<9;i++)
	{
		if (i==0||i==4||i==8)
			R[i] = 1 + sin(angle)*A[i] + (1-cos(angle))*A2[i];
		else
			R[i] = 0 + sin(angle)*A[i] + (1-cos(angle))*A2[i];
	}
	return R;
}

vector<double> transInv(
		vector<double> A)
{
	double det =	+ A[0]*(A[4]*A[8]-A[7]*A[5])
					- A[1]*(A[3]*A[8]-A[5]*A[6])
					+ A[2]*(A[3]*A[7]-A[4]*A[6]);
	double invdet = 1/det;
	vector<double> B(9);
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
}

void cal_tangent_normal(
	double t_mid_,
	point_t &p_tan_,
	point_t &p_nor_,
	vector<point_t> coeff,
	int dim,
	bool normal)
{
	point_t out1; out1.x=out1.y=out1.z=0;
	point_t out2; out2.x=out2.y=out2.z=0;
	point_t out3; out3.x=out3.y=out3.z=0;
	point_t out4; out4.x=out4.y=out4.z=0;
	for(int i=0;i<dim;i++)
	{
		out1.x += i* coeff[i].x * pow(t_mid_,i-1);
		out1.y += i* coeff[i].y * pow(t_mid_,i-1);
		out1.z += i* coeff[i].z * pow(t_mid_,i-1);
	}
	p_tan_ = out1;
	if(normal)
	{
		for(int i=0;i<dim;i++)
		{
			out2.x += i * (i-1) * coeff[i].x * pow(t_mid_,i-2);
			out2.y += i * (i-1) * coeff[i].y * pow(t_mid_,i-2);
			out2.z += i * (i-1) * coeff[i].z * pow(t_mid_,i-2);
		}
		out3.x = 2*out1.x*out2.x;
		out3.y = 2*out1.y*out2.y;
		out3.z = 2*out1.z*out2.z;
		double out3N = out3.x + out3.y + out3.z;
		out4.x = ((l2Norm(out1) * out2.x) - (out1.x * 0.5 * out3N * (1/l2Norm(out1)))) / Sqr(l2Norm(out1));
		out4.y = ((l2Norm(out1) * out2.y) - (out1.y * 0.5 * out3N * (1/l2Norm(out1)))) / Sqr(l2Norm(out1));
		out4.z = ((l2Norm(out1) * out2.z) - (out1.z * 0.5 * out3N * (1/l2Norm(out1)))) / Sqr(l2Norm(out1));
		p_nor_ = out4;
	}
	else
		p_nor_ = out4;
}


// ============================================================================
// B-spline
// ============================================================================

double curveIntegral (
	double x,
	void * params)
{
	double dfx[2], dfy[2], dfz[2];
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double *cc = (double *)params;
	for(int i=0;i<DEGREE;i++) cx[i] = cc[i+(DEGREE*0)];
	for(int i=0;i<DEGREE;i++) cy[i] = cc[i+(DEGREE*1)];
	for(int i=0;i<DEGREE;i++) cz[i] = cc[i+(DEGREE*2)];
	gsl_poly_eval_derivs (cx, DEGREE, x, dfx, 2);
	gsl_poly_eval_derivs (cy, DEGREE, x, dfy, 2);
	gsl_poly_eval_derivs (cz, DEGREE, x, dfz, 2);
	double f = sqrt(Sqr(dfx[1])+Sqr(dfy[1])+Sqr(dfz[1]));
	return f;
}

// example
void curveFit(
	vector<point_t> points_,
	vector<point_t> &curves_)
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
		bw 	= gsl_bspline_alloc(4, nbreak);
		B 	= gsl_vector_alloc(ncoeffs);

		x 		= gsl_vector_alloc(n);
		y 		= gsl_vector_alloc(n);
		X 		= gsl_matrix_alloc(n, ncoeffs);
		c 		= gsl_vector_alloc(ncoeffs);
		w 		= gsl_vector_alloc(n);
		cov 	= gsl_matrix_alloc(ncoeffs, ncoeffs);
		mw 		= gsl_multifit_linear_alloc(n, ncoeffs);
		yerr 	= gsl_vector_alloc(n);

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

void polyCurveFit(
	vector<double> points_,
	vector<double> &coeff_,
	vector<double> &cov_)
{
	gsl_multifit_linear_workspace *ws;
	gsl_matrix *cov, *X;
	gsl_vector *y, *c;
	double chisq; //residual error
	int num_points = points_.size();
	X = gsl_matrix_alloc(num_points, DEGREE);
	y = gsl_vector_alloc(num_points);
	c = gsl_vector_alloc(DEGREE);
	cov = gsl_matrix_alloc(DEGREE, DEGREE);
	for(int i=0;i<num_points;i++)
	{
		for(int j=0;j<DEGREE;j++)
		{
			gsl_matrix_set(X, i, j, pow(i, j));
		}
		gsl_vector_set(y, i, points_[i]);
	}
	ws = gsl_multifit_linear_alloc(num_points, DEGREE);
	gsl_multifit_linear(X, y, c, cov, &chisq, ws);
	reshapeVector(coeff_, DEGREE);
	for(int i=0;i<DEGREE;i++) { coeff_[i] = gsl_vector_get(c, i); }
	reshapeVector(cov_, Sqr(DEGREE));
	for(int i=0;i<Sqr(DEGREE);i++)
	{
		cov_[i] = gsl_matrix_get(cov, i/DEGREE, i%DEGREE);
	}
	gsl_multifit_linear_free(ws);
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(c);
}

void polyCurveFitPoint(
	vector<point_t> points_,
	vector<point_t> &points_est_,
	vector<point_t> &coeffs_,
	vector<point_t> &covs_,
	bool est_)
{
	int num_points = points_.size();
	vector<double> x(num_points);
	vector<double> y(num_points);
	vector<double> z(num_points);
	vector<double> cx(DEGREE);
	vector<double> cy(DEGREE);
	vector<double> cz(DEGREE);
	vector<double> covx(Sqr(DEGREE));
	vector<double> covy(Sqr(DEGREE));
	vector<double> covz(Sqr(DEGREE));
	for(int i=0;i<num_points;i++)
	{
		x[i] = points_[i].x;
		y[i] = points_[i].y;
		z[i] = points_[i].z;
	}
	polyCurveFit(x,cx,covx);
	polyCurveFit(y,cy,covy);
	polyCurveFit(z,cz,covz);
	reshapeVector(coeffs_,DEGREE);
	for(int i=0;i<DEGREE;i++)
	{
		coeffs_[i].x = cx[i];
		coeffs_[i].y = cy[i];
		coeffs_[i].z = cz[i];
	}
	reshapeVector(covs_,Sqr(DEGREE));
	for(int i=0;i<Sqr(DEGREE);i++)
	{
		covs_[i].x = covx[i];
		covs_[i].y = covy[i];
		covs_[i].z = covz[i];
	}
	int num_points2 = points_est_.size();
	vector<double> x2(num_points2);
	vector<double> y2(num_points2);
	vector<double> z2(num_points2);
	if(est_)
	{
		polyCurveFitEst(x2, num_points, cx, covx);
		polyCurveFitEst(y2, num_points, cy, covy);
		polyCurveFitEst(z2, num_points, cz, covz);
		for(int i=0;i<num_points2;i++)
		{
			points_est_[i].x = x2[i];
			points_est_[i].y = y2[i];
			points_est_[i].z = z2[i];
		}
	}
}

void polyCurveLength(
	double &length_,
	double a_,
	double b_,
	vector<point_t> coeffs_)
{
	gsl_function F;
	gsl_integration_glfixed_table *table;
	double cc[DEGREE*3];
	table = gsl_integration_glfixed_table_alloc ((DEGREE+1)/2);
	for(int i=0;i<DEGREE;i++)
	{
		cc[i+(DEGREE*0)] = coeffs_[i].x;
		cc[i+(DEGREE*1)] = coeffs_[i].y;
		cc[i+(DEGREE*2)] = coeffs_[i].z;
	}
	F.function = &curveIntegral;
	F.params = cc;
	length_ = gsl_integration_glfixed (&F, a_, b_, table);
	gsl_integration_glfixed_table_free (table);
}

void polyCurveFitEst(
	vector<double> &points_,
	int num_points_,
	vector<double> coeffs_,
	vector<double> covs_)
{
	gsl_matrix *cov, *X;
	gsl_vector *c, *Xj;
	double cc[DEGREE];
	X  = gsl_matrix_alloc(num_points_, DEGREE);
	Xj = gsl_vector_alloc(DEGREE);
	c  = gsl_vector_alloc(DEGREE);
	cov = gsl_matrix_alloc(DEGREE, DEGREE);
	for(int i=0;i<DEGREE;i++) { cc[i] = coeffs_[i]; }
	for(int i=0;i<points_.size();i++)
	{
		points_[i] =
				gsl_poly_eval(
						cc,
						DEGREE,
						(double)i/(points_.size()/num_points_));
	}
	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(Xj);
	gsl_vector_free(c);
}

// ============================================================================
// Surface
// ============================================================================

double surfaceRange(
	point_t pos_,
	point_t pos_surface_,
	vector<double> surface_)
{
	vector<double> surface_tmp(3);
	vector<double> surface_parallel_tmp;
	vector<double> p_tmp;
	double norm_tmp = 0.0;

	surface_tmp[0] = surface_[0];
	surface_tmp[1] = surface_[1];
	surface_tmp[2] = surface_[2];

	p_tmp = point2vector(minusPoint(pos_, pos_surface_));

	// surface_parallel_tmp can be obtained at the beginning by just taking the normalized vector between 2 points on the surface
	{
		surface_parallel_tmp = crossProduct(surface_tmp, p_tmp);

		for(int i=0;i<3;i++)
			norm_tmp += Sqr(surface_parallel_tmp[i]);
		norm_tmp = sqrt(norm_tmp);

		for(int i=0;i<3;i++)
			surface_parallel_tmp[i] /= norm_tmp;
	}

	return dotProduct(p_tmp, surface_parallel_tmp);
}

double surfaceDistance(
	point_t pos_,
	vector<double> surface_)
{
	return surface_[0]*pos_.x +
		   surface_[1]*pos_.y +
		   surface_[2]*pos_.z -
		   surface_[3];
}

double surfaceAngle(
	point_t vel_,
	vector<double> surface_)
{
	vector<double> tmp(3);
	tmp[0] = surface_[0];
	tmp[1] = surface_[1];
	tmp[2] = surface_[2];
	return 	l2Norm(crossProduct(tmp,point2vector(vel_)))/
			(l2Norm(tmp) * l2Norm(point2vector(vel_)));
}
