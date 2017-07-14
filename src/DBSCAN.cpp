/*******************************************************************************
 * DBSCAN.h
 *
 *  Created on: Apr 20, 2017
 *      Author: Chen, EeHeng
 * 		Detail: Implementation of DBSCAN algorithm. Original code is written in
 * 				C by Gagarine Yaikhom and modified by author.
 * 				(Copyright 2015 Gagarine Yaikhom MIT License).
 *
 ******************************************************************************/

#include "DBSCAN.h"

DBSCAN::DBSCAN()
{
}

DBSCAN::~DBSCAN()
{
}

DBSCAN::node_t *DBSCAN::create_node(
		unsigned int index)
{
	node_t *n = (node_t *) calloc(1, sizeof(node_t));
	if (n == NULL)
		perror("Failed to allocate node.");
	else
	{
		n->index = index;
		n->next = NULL;
	}
	return n;
}

int DBSCAN::append_at_end(
		unsigned int index,
		epsilon_neighbours_t *en)
{
	node_t *n = create_node(index);
	if (n == NULL)
	{
		free(en);
		return FAILURE;
	}
	if (en->head == NULL)
	{
		en->head = n;
		en->tail = n;
	}
	else
	{
		en->tail->next = n;
		en->tail = n;
	}
	++(en->num_members);
	return SUCCESS;
}

DBSCAN::epsilon_neighbours_t *DBSCAN::get_epsilon_neighbours(
		unsigned int index,
		point_d *points,
		unsigned int num_points,
		double epsilon)
{
	epsilon_neighbours_t *en = (epsilon_neighbours_t *) calloc(1,
			sizeof(epsilon_neighbours_t));
	if (en == NULL)
	{
		perror("Failed to allocate epsilon neighbours.");
		return en;
	}
	for (unsigned int i = 0; i < num_points; i++)
	{
		if (i == index)
			continue;
		if (this->euclidean_dist(&points[index], &points[i]) > epsilon)
			continue;
		else
		{
			if (append_at_end(i, en) == FAILURE)
			{
				destroy_epsilon_neighbours(en);
				en = NULL;
				break;
			}
		}
	}
	return en;
}

void DBSCAN::destroy_epsilon_neighbours(
		epsilon_neighbours_t *en)
{
	if (en)
	{
		node_t *t, *h = en->head;
		while (h)
		{
			t = h->next;
			free(h);
			h = t;
		}
		free(en);
	}
}

int DBSCAN::expand(
		unsigned int index,
		unsigned int l,
		point_d *points,
		unsigned int num_points,
		double epsilon,
		unsigned int minpts)
{
	int return_value = NOT_CORE_POINT;
	epsilon_neighbours_t *seeds = get_epsilon_neighbours(index, points,
			num_points, epsilon);
	if (seeds == NULL)
		return FAILURE;

	if (seeds->num_members < minpts)
		points[index].l = NOISE;
	else
	{
		points[index].l = l;
		node_t *h = seeds->head;
		while (h)
		{
			points[h->index].l = l;
			h = h->next;
		}

		h = seeds->head;
		while (h)
		{
			spread(h->index, seeds, l, points, num_points, epsilon, minpts);
			h = h->next;
		}

		return_value = CORE_POINT;
	}
	destroy_epsilon_neighbours(seeds);
	return return_value;
}

int DBSCAN::spread(
		unsigned int index,
		epsilon_neighbours_t *seeds,
		unsigned int l,
		point_d *points,
		unsigned int num_points,
		double epsilon,
		unsigned int minpts)
{
	epsilon_neighbours_t *spread = get_epsilon_neighbours(index, points,
			num_points, epsilon);
	if (spread == NULL)
		return FAILURE;
	if (spread->num_members >= minpts)
	{
		node_t *n = spread->head;
		point_d *d;
		while (n)
		{
			d = &points[n->index];
			if (d->l == NOISE || d->l == UNCLASSIFIED)
			{
				if (d->l == UNCLASSIFIED)
				{
					if (append_at_end(n->index, seeds) == FAILURE)
					{
						destroy_epsilon_neighbours(spread);
						return FAILURE;
					}
				}
				d->l = l;
			}
			n = n->next;
		}
	}

	destroy_epsilon_neighbours(spread);
	return SUCCESS;
}

double DBSCAN::euclidean_dist(
		point_d *a,
		point_d *b)
{
	return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));
}

void DBSCAN::dbscan(
		point_d *points,
		unsigned int num_points,
		double epsilon,
		unsigned int minpts)
{
	unsigned int i, l = 0;
	for (i = 0; i < num_points; ++i)
	{
		if (points[i].l == UNCLASSIFIED)
		{
			if (expand(i, l, points, num_points, epsilon, minpts) == CORE_POINT)
				++l;
		}
	}
}

DBSCAN::point_d DBSCAN::AddPoint(
		point_d A,
		point_d B)
{
	point_d C;
	C.x = A.x + B.x;
	C.y = A.y + B.y;
	C.z = A.z + B.z;
	C.l = A.l;
	return C;
}

DBSCAN::point_d DBSCAN::MinusPoint(
		point_d A,
		point_d B)
{
	point_d C;
	C.x = A.x - B.x;
	C.y = A.y - B.y;
	C.z = A.z - B.z;
	C.l = A.l;
	return C;
}

DBSCAN::point_d DBSCAN::MultiPoint(
		point_d A,
		double B)
{
	point_d C;
	C.x = A.x * B;
	C.y = A.y * B;
	C.z = A.z * B;
	C.l = A.l;
	return C;
}

double DBSCAN::l2Norm(
		point_d A)
{
	return sqrt(A.x * A.x + A.y * A.y + A.z * A.z);
}

void DBSCAN::vectorToArray(
		std::vector<point_d> A,
		point_d *B)
{
	for (int i = 0; i < A.size(); i++)
	{
		B[i] = A[i];
	}
}

void DBSCAN::ArrayTovector(
		point_d *A,
		int size,
		std::vector<point_d> &B)
{
	B.clear();
	B.resize(size);
	for (int i = 0; i < size; i++)
	{
		B[i] = A[i];
	}
}

Eigen::Vector4d DBSCAN::PointToVector4d(
		point_d A)
{
	Eigen::Vector4d B;
	B(0) = A.x;
	B(1) = A.y;
	B(2) = A.z;
	B(3) = A.l;
	return B;
}
DBSCAN::point_d DBSCAN::Vector4dToPoint(
		Eigen::Vector4d A)
{
	point_d B;
	B.x = A(0);
	B.y = A(1);
	B.z = A(2);
	B.l = A(3);
	return B;
}

void DBSCAN::DBSCANCluster(
		double epsilon,
		unsigned int minpts,
		unsigned int num_points,
		point_d *p)
{
	if (num_points)
	{
		this->dbscan(p, num_points, epsilon, minpts);
	}
	else
	{
		std::cerr << "NO POINTS FOR CLUSTERING!!!";
	}
}

void DBSCAN::Clustering(
		std::shared_ptr<std::vector<Eigen::Vector4d> > points_,
		std::shared_ptr<std::vector<Eigen::Vector4d> > centroids_,
		std::shared_ptr<std::vector<int> > locations_flag_,
		std::shared_ptr<std::vector<int> > contact_flag_,
		const double &epsilon,
		const unsigned int &minpts)
{
	std::vector<point_d> pos_tmp;
	std::vector<point_d> loc_tmp;

	for (auto p : *points_)
	{
		pos_tmp.push_back(Vector4dToPoint(p));
	}
	for (auto c : *centroids_)
	{
		loc_tmp.push_back(Vector4dToPoint(c));
	}

	auto num_points = points_->size();
	point_d *points_array = new point_d[num_points];
	this->vectorToArray(pos_tmp, points_array);

	this->DBSCANCluster(epsilon, minpts, num_points, points_array);

	this->ArrayTovector(points_array, num_points, pos_tmp);
	delete[] points_array;

	this->CombineNearCluster(pos_tmp, loc_tmp, *locations_flag_,
			*contact_flag_);

	points_->clear();
	centroids_->clear();

	for (auto p : pos_tmp)
	{
		points_->push_back(PointToVector4d(p));
	}
	for (auto l : loc_tmp)
	{
		centroids_->push_back(PointToVector4d(l));
	}
}

void DBSCAN::CombineNearCluster(
		std::vector<point_d> &points_,
		std::vector<point_d> &locations_,
		std::vector<int> &locations_flag_,
		const std::vector<int> &contact_)
{
	int num_points = points_.size();
	int num_locations = locations_.size();
	int num_locations2 = 0;

	if (num_locations < 1)
	{
		for (auto i : points_)
			num_locations = std::max((int) i.l, num_locations);
		num_locations += 1;
		num_locations2 = num_locations;
	}
	else
	{
		for (auto i : points_)
			num_locations2 = std::max((int) i.l, num_locations2);
		num_locations2 += 1;
	}

	// calculating the centroid of cluster
	std::vector<point_d> p_tmp(num_locations2,
	{ 0.0, 0.0, 0.0, 0.0 });
	for (auto i : points_)
	{
		if (i.l >= 0)
		{
			p_tmp[(int) i.l] = this->AddPoint(p_tmp[(int) i.l], i);
			p_tmp[(int) i.l].l += 1;
		}
	}

	for (auto &i : p_tmp)
	{
		i = this->MultiPoint(i, 1 / i.l);
		i.l = UNCLASSIFIED;
	}

	// combine cluster if it is less than 0.1m
	bool limit = false;
	for (int i = 0; i < num_locations2; i++)
	{
		for (int j = 0; j < num_locations2; j++)
		{
			if (j <= i)
				continue;

			for (int ii = 0; ii < num_points; ii++)
				if (points_[ii].l == i && !limit)
					for (int jj = 0; jj < num_points; jj++)
						if (points_[jj].l == j)
							if (this->l2Norm(
									this->MinusPoint(points_[ii],
											points_[jj])) < CLUSTER_LIMIT)
								limit = true;

			if (limit)
			{
				limit = false;

				if (p_tmp[i].l >= 0 && p_tmp[j].l >= 0)
				{
					int big = std::max(p_tmp[i].l, p_tmp[j].l);
					int small = std::min(p_tmp[i].l, p_tmp[j].l);
					for (int ii = 0; ii < num_locations2; ii++)
					{
						if (p_tmp[ii].l == big)
						{
							p_tmp[ii].l = small;
						}
					}
				}
				else if (p_tmp[i].l >= 0)
				{
					p_tmp[j].l = p_tmp[i].l;
				}
				else if (p_tmp[j].l >= 0)
				{
					p_tmp[i].l = p_tmp[j].l;
				}
				else
				{
					if (i < j)
					{
						p_tmp[i].l = i;
						p_tmp[j].l = i;
					}
					else
					{
						p_tmp[i].l = j;
						p_tmp[j].l = j;
					}
				}
			}
			else
			{
				if (p_tmp[i].l != (int) i && p_tmp[i].l < 0)
				{
					p_tmp[i].l = i;
				}
				if (p_tmp[j].l != (int) j && p_tmp[j].l < 0)
				{
					p_tmp[j].l = j;
				}
			}
		}
		//printf("Location %02d: %02f\n", i, p_tmp[i].l);
	}

	// removing the missing cluster labels
	int c = 1;
	for (int i = 1; i < num_locations2; i++)
	{
		if (p_tmp[i].l > p_tmp[i - 1].l && p_tmp[i].l == i)
		{
			p_tmp[i].l = c;
			for (int ii = i + 1; ii < num_locations2; ii++)
			{
				if (p_tmp[ii].l == i)
				{
					p_tmp[ii].l = c;
				}
			}
			c++;
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].l );
	}

	// updating cluster label
	for (auto &i : points_)
	{
		if (i.l >= 0)
		{
			i.l = p_tmp[(int) i.l].l;
		}
		//printf("Location %02d: %02f\n", i, points_[i].l );
	}

	if (!contact_.empty())
	{
		// checking contact constraint
		num_locations2 = 0;
		for (int i = 0; i < num_points; i++)
		{
			num_locations2 = std::max((int) points_[i].l, num_locations2);
		}
		num_locations2 += 1;
		locations_flag_.clear();
		locations_flag_.resize(num_locations2);
		std::vector<double> c1(num_locations2, 0.0);
		std::vector<double> c2(num_locations2, 0.0);
		for (int i = 0; i < num_points; i++)
		{
			if (points_[i].l < 0)
			{
				continue;
			}
			if (contact_[i] == 1)
			{
				c1[points_[i].l] += 1;
			}
			c2[points_[i].l] += 1;
		}
		for (auto i : points_)
		{
			if (i.l < 0)
			{
				continue;
			}
			if (c1[i.l] / c2[i.l] < CONTACT_TRIGGER_RATIO)
			{
				continue;
			}
			locations_flag_[i.l] = 1;
		}
	}

	// calculate the centroid of combined clusters
	p_tmp.clear();
	p_tmp.resize(num_locations2);

	for (auto i : points_)
	{
		if (i.l >= 0)
		{
			p_tmp[(int) i.l] = this->AddPoint(p_tmp[(int) i.l], i);
			p_tmp[(int) i.l].l += 1;
		}
		//printf("Location %02d: %02d %02d\n", i, points_[i].l, p_center[points_[i].l].l );
	}

	for (auto &i : p_tmp)
	{
		i = this->MultiPoint(i, 1 / i.l);
		i.l = 1.0; // boundary starts with 1.0 as no error and goes to zero for large distance boundary
		//printf("Location %02d: %+.4f %+.4f %+.4f %d\n", i, p_center[i].x, p_center[i].y, p_center[i].z, p_center[i].l);
	}

//	for(int i=0;i<p_tmp.size();i++)
//	{
//		p_tmp[i]	= this->MultiPoint(p_tmp[i],1/count[i]);
//		p_tmp[i].l 	= 1.0; // boundary starts with 1.0 as no error and goes to zero for large distance boundary
//		count[i]	= UNCLASSIFIED;
//		//printf("Location %02d: %+.4f %+.4f %+.4f %d\n", i, p_center[i].x, p_center[i].y, p_center[i].z, p_center[i].l);
//	}
	locations_ = p_tmp;
}
