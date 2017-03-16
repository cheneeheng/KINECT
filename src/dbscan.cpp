/* Copyright 2015 Gagarine Yaikhom (MIT License) */
/* modified by Chen */

#include "dbscan.h"

node_t *create_node(unsigned int index)
{
    node_t *n = (node_t *) calloc(1, sizeof(node_t));
    if (n == NULL)
        perror("Failed to allocate node.");
    else {
        n->index = index;
        n->next = NULL;
    }
    return n;
}

int append_at_end(
     unsigned int index,
     epsilon_neighbours_t *en)
{
    node_t *n = create_node(index);
    if (n == NULL) {
        free(en);
        return FAILURE;
    }
    if (en->head == NULL) {
        en->head = n;
        en->tail = n;
    } else {
        en->tail->next = n;
        en->tail = n;
    }
    ++(en->num_members);
    return SUCCESS;
}

epsilon_neighbours_t *get_epsilon_neighbours(
    unsigned int index,
    point_t *points,
    unsigned int num_points,
    double epsilon,
    double (*dist)(point_t *a, point_t *b))
{
    epsilon_neighbours_t *en = (epsilon_neighbours_t *)
        calloc(1, sizeof(epsilon_neighbours_t));
    if (en == NULL) {
        perror("Failed to allocate epsilon neighbours.");
        return en;
    }
    for (unsigned int i = 0; i < num_points; i++) {
        if (i == index)
            continue;
        if (dist(&points[index], &points[i]) > epsilon)
            continue;
        else {
            if (append_at_end(i, en) == FAILURE) {
                destroy_epsilon_neighbours(en);
                en = NULL;
                break;
            }
        }
    }
    return en;
}

void destroy_epsilon_neighbours(epsilon_neighbours_t *en)
{
    if (en) {
        node_t *t, *h = en->head;
        while (h) {
            t = h->next;
            free(h);
            h = t;
        }
        free(en);
    }
}

void dbscan(
    point_t *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts,
    double (*dist)(point_t *a, point_t *b))
{
    unsigned int i, cluster_id = 0;
    for (i = 0; i < num_points; ++i) {
        if (points[i].cluster_id == UNCLASSIFIED) {
            if (expand(i, cluster_id, points,
                       num_points, epsilon, minpts,
                       dist) == CORE_POINT)
                ++cluster_id;
        }
    }
}

int expand(
    unsigned int index,
    unsigned int cluster_id,
    point_t *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts,
    double (*dist)(point_t *a, point_t *b))
{
    int return_value = NOT_CORE_POINT;
    epsilon_neighbours_t *seeds =
        get_epsilon_neighbours(index, points,
                               num_points, epsilon,
                               dist);
    if (seeds == NULL)
        return FAILURE;

    if (seeds->num_members < minpts)
        points[index].cluster_id = NOISE;
    else {
        points[index].cluster_id = cluster_id;
        node_t *h = seeds->head;
        while (h) {
            points[h->index].cluster_id = cluster_id;
            h = h->next;
        }

        h = seeds->head;
        while (h) {
            spread(h->index, seeds, cluster_id, points,
                   num_points, epsilon, minpts, dist);
            h = h->next;
        }

        return_value = CORE_POINT;
    }
    destroy_epsilon_neighbours(seeds);
    return return_value;
}

int spread(
    unsigned int index,
    epsilon_neighbours_t *seeds,
    unsigned int cluster_id,
    point_t *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts,
    double (*dist)(point_t *a, point_t *b))
{
    epsilon_neighbours_t *spread =
        get_epsilon_neighbours(index, points,
                       num_points, epsilon,
                       dist);
    if (spread == NULL)
        return FAILURE;
    if (spread->num_members >= minpts) {
        node_t *n = spread->head;
        point_t *d;
        while (n) {
            d = &points[n->index];
            if (d->cluster_id == NOISE ||
                d->cluster_id == UNCLASSIFIED) {
                if (d->cluster_id == UNCLASSIFIED) {
                    if (append_at_end(n->index, seeds)
                        == FAILURE) {
                        destroy_epsilon_neighbours(spread);
                        return FAILURE;
                    }
                }
                d->cluster_id = cluster_id;
            }
            n = n->next;
        }
    }

    destroy_epsilon_neighbours(spread);
    return SUCCESS;
}

double euclidean_dist(point_t *a, point_t *b)
{
    return sqrt(pow(a->x - b->x, 2) +
            pow(a->y - b->y, 2) +
            pow(a->z - b->z, 2));
}

//unsigned int parse_input(
//    FILE *file,
//    point_t **points,
//    double *epsilon,
//    unsigned int *minpts)
//{
//    unsigned int num_points, i = 0;
//    fscanf(file, "%lf %u %u\n",
//           epsilon, minpts, &num_points);
//    point_t *p = (point_t *)
//        calloc(num_points, sizeof(point_t));
//    if (p == NULL) {
//        perror("Failed to allocate points array");
//        return 0;
//    }
//    while (i < num_points) {
//          fscanf(file, "%lf %lf %lf\n",
//                 &(p[i].x), &(p[i].y), &(p[i].z));
//          p[i].cluster_id = UNCLASSIFIED;
//          ++i;
//    }
//    *points = p;
//    return num_points;
//}
//
//void print_points(
//    point_t *points,
//    unsigned int num_points)
//{
//    unsigned int i = 0;
//    printf("Number of points: %u\n"
//        " x     y     z     cluster_id\n"
//        "-----------------------------\n"
//        , num_points);
//    while (i < num_points) {
//          printf("%5.2lf %5.2lf %5.2lf: %d\n",
//                 points[i].x,
//                 points[i].y, points[i].z,
//                 points[i].cluster_id);
//          ++i;
//    }
//}
//
//void print_epsilon_neighbours(
//    point_t *points,
//    epsilon_neighbours_t *en)
//{
//    if (en) {
//        node_t *h = en->head;
//        while (h) {
//            printf("(%lfm, %lf, %lf)\n",
//                   points[h->index].x,
//                   points[h->index].y,
//                   points[h->index].z);
//            h = h->next;
//        }
//    }
//}
//
//int main(void) {
//    point_t *points;
//    double epsilon;
//    unsigned int minpts;
//    unsigned int num_points =
//        parse_input(stdin, &points, &epsilon, &minpts);
//    if (num_points) {
//        dbscan(points, num_points, epsilon,
//               minpts, euclidean_dist);
//        printf("Epsilon: %lf\n", epsilon);
//        printf("Minimum points: %u\n", minpts);
//        print_points(points, num_points);
//    }
//    free(points);
//    return 0;
//}


// ============================================================================
// dbscan
// ============================================================================

void dbscanCluster(
	double epsilon,
	unsigned int minpts,
	unsigned int num_points,
	point_t *p)
{
	if(num_points)
        dbscan(p, num_points, epsilon, minpts, euclidean_dist);
	else
		cerr << "NO POINTS FOR CLUSTERING!!!";
}

void combineNearCluster(
	vector<point_t> 		&points_,
	vector<point_t> 		&locations_)
{
	int num_points 		= points_.size();
	int num_locations 	= locations_.size();

	if (num_locations < 1)
	{
		for(int i=0;i<num_points;i++)
			num_locations = max(points_[i].cluster_id,num_locations);
		num_locations += 1;
	}

	// calculating the centroid of cluster
	vector<point_t> p_tmp0(num_locations);
	vector<point_t> p_tmp1(num_locations);
	for(int i=0;i<num_points;i++)
	{
		if(points_[i].cluster_id >= 0)
		{
			p_tmp1[points_[i].cluster_id].cluster_id += 1;
			p_tmp0[points_[i].cluster_id].x += points_[i].x;
			p_tmp0[points_[i].cluster_id].y += points_[i].y;
			p_tmp0[points_[i].cluster_id].z += points_[i].z;
		}
	}

	for(int i=0;i<num_locations;i++)
	{
		p_tmp1[i].x = p_tmp0[i].x/p_tmp1[i].cluster_id;
		p_tmp1[i].y = p_tmp0[i].y/p_tmp1[i].cluster_id;
		p_tmp1[i].z = p_tmp0[i].z/p_tmp1[i].cluster_id;
		p_tmp1[i].cluster_id = UNCLASSIFIED;
	}

	// combine cluster if it is less than 0.1m
	bool limit = false;
	for(int i=0;i<num_locations;i++)
	{
		for(int j=0;j<num_locations;j++)
		{
			if(j<=i) continue;

			for(int ii=0;ii<num_points;ii++)
				if(points_[ii].cluster_id == i && !limit)
					for(int jj=0;jj<num_points;jj++)
						if(points_[jj].cluster_id == j)
							if(l2Norm(minusPoint(points_[ii],points_[jj]))<0.1)
								limit = true;

			if(limit)
			{
				limit = false;

				if(p_tmp1[i].cluster_id>=0 && p_tmp1[j].cluster_id>=0)
				{
					int big   = max(p_tmp1[i].cluster_id,
									p_tmp1[j].cluster_id);
					int small = min(p_tmp1[i].cluster_id,
									p_tmp1[j].cluster_id);
					for(int ii=0;ii<num_locations;ii++)
					{
						if(p_tmp1[ii].cluster_id == big)
						   p_tmp1[ii].cluster_id = small;
					}
				}
				else if(p_tmp1[i].cluster_id>=0)
						p_tmp1[j].cluster_id = p_tmp1[i].cluster_id;
				else if(p_tmp1[j].cluster_id>=0)
						p_tmp1[i].cluster_id = p_tmp1[j].cluster_id;
				else
				{
					if(i<j)
					{
						p_tmp1[i].cluster_id = i;
						p_tmp1[j].cluster_id = i;
					}
					else
					{
						p_tmp1[i].cluster_id = j;
						p_tmp1[j].cluster_id = j;
					}
				}
			}
			else
			{
				if(p_tmp1[i].cluster_id!=(int)i && p_tmp1[i].cluster_id<0)
				   p_tmp1[i].cluster_id = i;
				if(p_tmp1[j].cluster_id!=(int)j && p_tmp1[j].cluster_id<0)
				   p_tmp1[j].cluster_id = j;
			}
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].cluster_id);
	}

	// removing the missing cluster labels
	int c = 1;
	for(int i=1;i<num_locations;i++)
	{
		if(p_tmp1[i].cluster_id > p_tmp1[i-1].cluster_id &&
		   p_tmp1[i].cluster_id == i)
		{
			p_tmp1[i].cluster_id = c;
			for(int ii=i+1;ii<num_locations;ii++)
				if(p_tmp1[ii].cluster_id == i)
				   p_tmp1[ii].cluster_id = c;
			c++;
		}
		//printf("Location %02d: %02d\n", i, p_tmp1[i].cluster_id );
	}

	// updating cluster label
	for(int i=0;i<num_points;i++)
	{
		if (points_[i].cluster_id >= 0)
			points_[i].cluster_id = p_tmp1[points_[i].cluster_id].cluster_id;
		//printf("Location %02d: %02d\n", i, points_[i].cluster_id );
	}

	// calculate the centroid of combined clusters
	vector<point_t> p_tmp2  (c);
	vector<point_t> p_center(c);

	for(int i=0;i<num_points;i++)
	{
		if(points_[i].cluster_id >= 0)
		{
			p_center[points_[i].cluster_id].cluster_id += 1;
			p_tmp2  [points_[i].cluster_id].x += points_[i].x;
			p_tmp2  [points_[i].cluster_id].y += points_[i].y;
			p_tmp2  [points_[i].cluster_id].z += points_[i].z;
		}
		//printf("Location %02d: %02d %02d\n", i, points_[i].cluster_id, p_center  [p[i].cluster_id].cluster_id );
	}

	for(int i=0;i<c;i++)
	{
		p_center[i].x = p_tmp2[i].x/p_center[i].cluster_id;
		p_center[i].y = p_tmp2[i].y/p_center[i].cluster_id;
		p_center[i].z = p_tmp2[i].z/p_center[i].cluster_id;
		p_center[i].cluster_id = UNCLASSIFIED;
		//printf("Location %02d: %+.4f %+.4f %+.4f\n", i, p_center[i].x, p_center[i].y, p_center[i].z );
	}

	//cout << num_locations << c << endl;

	locations_.clear(); locations_ = p_center;
}
