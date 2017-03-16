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

point_t averagePoint(vector<point_t> A)
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
