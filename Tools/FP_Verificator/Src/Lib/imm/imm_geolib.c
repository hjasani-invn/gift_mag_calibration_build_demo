#include "imm_geolib.h"
#include "imm_math.h"
#include "imm_clipper.h"
#include "imm_memory.h"

#include <math.h>


static float det(const float Pxi, const float Pyi, const float Pxi1, const float Pyi1, const float Rx, const float Ry);


static float
det(const float Pxi, const float Pyi, const float Pxi1, const float Pyi1, const float Rx, const float Ry)
{
	return (Pxi - Rx)*(Pyi1 - Ry) - (Pxi1 - Rx)*(Pyi - Ry);
}


void
angle_correct(float *angle)
{
	float temp = *angle;
	while (ABS(temp) > 180.0f) {
		temp = (temp > 180.0f) ? temp - 360.0f : (temp < -180.0f) ? temp + 360.0f : temp;
	}

	*angle = temp;
}


float 
angle_diff(float angle1, float angle2)
{
	float diff = angle1 - angle2;
	angle_correct(&diff);
	return diff;
}


float
vector2f_slope(const vector2f_t *p1, const vector2f_t *p2)
{
	if (NEAR_EQUAL(p1->X, p2->X)) {
		return (p2->Y > p1->Y) ? 0.0f : 180.0f;
	}
	else if (NEAR_EQUAL(p1->Y, p2->Y)) {
		return (p2->X > p1->X) ? 90.0f : -90.0f;
	}
	else {
		float slope = (float)(90.0 - atan2((p2->Y - p1->Y), (p2->X - p1->X)) * RAD2DEG);
		angle_correct(&slope);
		return slope;
	}
}


bool_t
vector2f_intersect(vector2f_t *inter_pt, 
                   const vector2f_t *p1,
                   const vector2f_t *p2, 
				   const vector2f_t *q1, 
				   const vector2f_t *q2)
{

	float par, tp, tq;

	par = (p2->X - p1->X) * (q2->Y - q1->Y) - (p2->Y - p1->Y) * (q2->X - q1->X);

	if (NEAR_ZERO(par))
		return FALSE;

	tp = ((q1->X - p1->X) * (q2->Y - q1->Y) - (q1->Y - p1->Y) * (q2->X - q1->X)) / par;
	tq = ((p2->Y - p1->Y) * (q1->X - p1->X) - (p2->X - p1->X) * (q1->Y - p1->Y)) / par;

	if (tp<0 || tp>1 || tq<0 || tq>1)
		return FALSE;

	inter_pt->X = p1->X + tp * (p2->X - p1->X);
	inter_pt->Y = p1->Y + tp * (p2->Y - p1->Y);

	return TRUE;
}


float 
vector2f_dot(const vector2f_t *p1, const vector2f_t *p2)
{
	return p1->X * p2->X + p1->Y *p2->Y;
}


vector2f_t 
vector2f_edge_projection(const vector2f_t *p, const edge_t *edge)
{
	vector2f_t v1, v2, ret, norm_v2;
	float proj, v2_dist;

	v1.X = p->X - edge->start.X;
	v1.Y = p->Y - edge->start.Y;
	v2.X = edge->end.X - edge->start.X;
	v2.Y = edge->end.Y - edge->start.Y;

	v2_dist = vector2f_distance(&edge->start, &edge->end);
	norm_v2.X = v2.X / v2_dist;
	norm_v2.Y = v2.Y / v2_dist;

	proj = vector2f_dot(&v1, &v2) / v2_dist;

	norm_v2.X *= proj;
	norm_v2.Y *= proj;

	ret.X = edge->start.X + norm_v2.X;
	ret.Y = edge->start.Y + norm_v2.Y;

	return ret;
}



bool_t 
edge_intersect(vector2f_t *inter_pt, const edge_t *edge1, const edge_t *edge2)
{
	return vector2f_intersect(inter_pt, 
		&edge1->start, &edge1->end, &edge2->start, &edge2->end);
}


int8_t 
vector2f_side_of_edge(const vector2f_t *point,
	                         const edge_t *edge)
{
	// Cross product. vector1 of edge->start ----> edge->end, vector2 of edge->start ----> p, 
	float test = (edge->end.X - edge->start.X) * (point->Y - edge->start.Y) - 
		(edge->end.Y - edge->start.Y) * (point->X - edge->start.X);

	if (test > 0)
		return 1;
	else if (test < 0)
		return -1;
	else
		return 0;
}


bool_t
point_on_line(const vector2f_t *pt, const vector2f_t *start_pt, const vector2f_t *end_pt, float tolerance)
{
	float cross_prod = (pt->Y - start_pt->Y) * (end_pt->X - start_pt->X) -
		(pt->X - start_pt->X) * (end_pt->Y - start_pt->Y);

	float dot_prod, squared_lengthba;

	return_val_if_fail(ABS(cross_prod) <= tolerance, FALSE);

	dot_prod = (pt->X - start_pt->X) * (end_pt->X - start_pt->X) +
		(pt->Y - start_pt->Y) * (end_pt->Y - start_pt->Y);

	return_val_if_fail(dot_prod >= 0.0f, FALSE);

	squared_lengthba = (end_pt->X - start_pt->X)*(end_pt->X - start_pt->X) +
		(end_pt->Y - start_pt->Y)*(end_pt->Y - start_pt->Y);

	return_val_if_fail((dot_prod <= squared_lengthba), FALSE);

	return	TRUE;
}


bool_t
point_in_polygon(const vector2f_t *R, const polygon_t *polygon)
{
	bool_t status = FALSE;
	int32_t w = 0;
	vector2f_t *Pi, *Pi1;
	float d;
	edgelist_t *edge = list_entry(polygon->edge_list.next, edgelist_t, list);

	if (vector2f_equal(&edge->edge.start, R)) {
		return status;
	}

	list_for_each_entry(edge, edgelist_t, &polygon->edge_list, list) {
		Pi = &edge->edge.start;
		Pi1 = &edge->edge.end;

		if (NEAR_EQUAL(Pi1->Y, R->Y)){
			if (NEAR_EQUAL(Pi1->X, R->X)){
				return status;
			}
			else {
				if (NEAR_EQUAL(Pi->Y, R->Y) && ((Pi1->X > R->X) == (Pi->X < R->X))) {
					return status;
				}
			}
		}

		if ((Pi->Y<R->Y) != (Pi1->Y < R->Y)) {
			if (Pi->X >= R->X) {
				if (Pi1->X > R->X) {
					w = w + 2 * (Pi1->Y>Pi->Y) - 1;
				}
				else {
					d = det(Pi->X, Pi->Y, Pi1->X, Pi1->Y, R->X, R->Y);
					if (NEAR_ZERO(d)) {
						return status;
					}
					else{
						if ((d > 0.0f) == (Pi1->Y > Pi->Y)) {
							w = w + 2 * (Pi1->Y > Pi->Y) - 1;
						}
					}
				}
			}
			else {
				if (Pi1->X > R->X) {
					d = det(Pi->X, Pi->Y, Pi1->X, Pi1->Y, R->X, R->Y);

					if (NEAR_ZERO(d)) {
						return status;
					}
					else {
						if ((d > 0.0f) == (Pi1->Y > Pi->Y)) {
							w = w + 2 * (Pi1->Y > Pi->Y) - 1;
						}
					}
				}
			}
		}
	}

	if (w != 0)
		status = TRUE;

	return status;
}


void
polygon_geometry(const polygon_t *poly, float *area, vector2f_t *center)
{
	float xm, ym, Axc, Ayc;
	float *x, *y, *dx, *dy;
	uint32_t num = poly->num, i;
	edgelist_t *edge;

	*area = 0.0f;
	xm = 0.0f;
	ym = 0.0f;
	Axc = 0.0f;
	Ayc = 0.0f;

	x = (float *)mem_allocate(num * sizeof (float));
	y = (float *)mem_allocate(num * sizeof (float));
	dx = (float *)mem_allocate(num * sizeof (float));
	dy = (float *)mem_allocate(num * sizeof (float));

	i = 0;
	list_for_each_entry(edge, edgelist_t, &poly->edge_list, list) {
		x[i] = edge->edge.start.X;
		y[i] = edge->edge.start.Y;

		xm += x[i] / (float)num;
		ym += y[i] / (float)num;
		i++;
	}

	for (i = 0; i < num; i++) {
		x[i] -= xm;
		y[i] -= ym;
	}

	for (i = 1; i < num; i++) {
		dx[i - 1] = (x[i] - x[i - 1]);
		dy[i - 1] = (y[i] - y[i - 1]);
	}
	dx[num - 1] = (x[0] - x[num - 1]);
	dy[num - 1] = (y[0] - y[num - 1]);

	for (i = 0; i < num; i++) {
		*area += (y[i] * dx[i] - x[i] * dy[i]);
		Axc += (6.0f*x[i] * y[i] * dx[i] - 3.0f*x[i] * x[i] * dy[i] + 3.0f*y[i] * dx[i] * dx[i] + dx[i] * dx[i] * dy[i]);
		Ayc += (3.0f*y[i] * y[i] * dx[i] - 6.0f*x[i] * y[i] * dy[i] - 3.0f*x[i] * dy[i] * dy[i] - dx[i] * dy[i] * dy[i]);
	}

	*area /= 2.0f;
	Axc /= 12.0f;
	Ayc /= 12.0f;

	if (*area < 0.0) {
		*area = -(*area);
		Axc = -Axc;
		Ayc = -Ayc;
	}

	center->X = Axc / (*area) + xm;
	center->Y = Ayc / (*area) + ym;

	mem_free(x);
	mem_free(y);
	mem_free(dx);
	mem_free(dy);
}


void
polygon_offset(polygon_t *poly, float offset, JoinType_t join_type, uint32_t scale)
{

	ptr_vector_t *subject, *solution;
	uint32_t i;
	double clipper_offset;
	edgelist_t *curr;

	subject = ptr_vector_new();
	solution = ptr_vector_new();

	int_polygons_append_element(subject, int_polygon_sized_new(poly->num));

	i = 0;
	list_for_each_entry(curr, edgelist_t, &poly->edge_list, list) {
		int_polygons_vertex(subject, 0, i).X =
			(int64_t)(curr->edge.start.X * (float)scale);
		int_polygons_vertex(subject, 0, i).Y =
			(int64_t)(curr->edge.start.Y * (float)scale);
		i++;
	}

	clipper_offset = offset * (double)scale;

	clipper_offset_polygons(subject, solution, clipper_offset, join_type, 2.0);

	if (solution->len > 1) {
		i = 0;
		list_for_each_entry_prev(curr, edgelist_t, &poly->edge_list, list) {
			int_polygons_vertex(subject, 0, i).X =
				(int64_t)(curr->edge.start.X * (float)scale);
			int_polygons_vertex(subject, 0, i).Y =
				(int64_t)(curr->edge.start.X * (float)scale);
			i++;
		}

		clipper_offset_polygons(subject, solution, clipper_offset, join_type, 2.0);
	}

	polygon_free(poly);
	

	if (solution->len > 0) {
		polygon_init(poly);

		for (i = 0; i < int_polygons_at(solution, 0)->len; i++) {
			vector2f_t point;
			point.X = (float)int_polygons_vertex(solution, 0, i).X / (float)scale;
			point.Y = (float)int_polygons_vertex(solution, 0, i).Y / (float)scale;
			polygon_add_vertex(poly, &point);
		}

		polygon_close_path(poly);
	}

	int_polygons_clear(subject, TRUE);
	int_polygons_free(subject, TRUE);

	int_polygons_clear(solution, TRUE);
	int_polygons_free(solution, TRUE);
}


void
polygon_clip(ptr_vector_t *subject_in, const polygon_t *clip_in,  ClipType_t clip_type,  uint32_t scale)
{
	int_polygons_t *subj, *clip, *sol;
	uint32_t i, j;
	edgelist_t *edge;
	clipper_t *clipper;

	sol = int_polygons_new();
	subj = int_polygons_new();

	for (j = 0; j < subject_in->len; j++) {
		polygon_t *sub_j = (polygon_t *)ptr_vector_at(subject_in, j);
		int_polygons_append_element(subj, int_polygon_sized_new(sub_j->num));

		i = 0;
		list_for_each_entry(edge, edgelist_t, &sub_j->edge_list, list) {
			int_polygons_vertex(subj, j, i).X = (int64_t)(edge->edge.start.X * (float)scale);
			int_polygons_vertex(subj, j, i).Y = (int64_t)(edge->edge.start.Y * (float)scale);
			i++;
		}
	}

	clip = int_polygons_new();

	int_polygons_append_element(clip, int_polygon_sized_new(clip_in->num));

	i = 0;
	list_for_each_entry(edge, edgelist_t, &clip_in->edge_list, list) {
		int_polygons_vertex(clip, 0, i).X = (int64_t)(edge->edge.start.X * (float)scale);
		int_polygons_vertex(clip, 0, i).Y = (int64_t)(edge->edge.start.Y * (float)scale);
		i++;
	}

	clipper = clipper_init();
	clipper_add_polygons(clipper, subj, pt_subject);
	clipper_add_polygons(clipper, clip, pt_clip);
	clipper_execute(clipper, clip_type, sol, pf_even_odd, pf_even_odd);

	polygons_clear(subject_in);
	for (i = 0; i < sol->len; i++) {
		polygon_t *tmp_poly = (polygon_t *)mem_allocate(sizeof(polygon_t));
		polygon_init(tmp_poly);
		for (j = 0; j < int_polygons_at(sol, i)->len; j++) {
			vector2f_t vertex;
			vertex.X = (float)int_polygons_vertex(sol, i, j).X / (float)scale;
			vertex.Y = (float)int_polygons_vertex(sol, i, j).Y / (float)scale;
			polygon_add_vertex(tmp_poly, &vertex);
		}
		polygon_close_path(tmp_poly);

		ptr_vector_append_element(subject_in, tmp_poly);
	}

	clipper_free(clipper);
	int_polygons_clear(subj, TRUE);
	int_polygons_free(subj, TRUE);

	int_polygons_clear(clip, TRUE);
	int_polygons_free(clip, TRUE);

	int_polygons_clear(sol, TRUE);
	int_polygons_free(sol, TRUE);

}


float
polygons_overlap_area_percentage(const polygon_t *poly1, 
                                 const polygon_t *poly2, 
								 const ClipType_t ct, 
								 vector2f_t *center)
{
	ptr_vector_t *subj = ptr_vector_new();
	float area_percent, overlap_area, ellipse_area;
	vector2f_t ellipse_pt;
	polygon_t *new_sub = (polygon_t *)mem_allocate(sizeof(polygon_t));

	polygon_init(new_sub);
	polygon_deep_cpy(new_sub, poly1);
	ptr_vector_append_element(subj, new_sub);

	polygon_clip(subj, poly2, ct, 1000);

	if (subj->len != 0) {
		polygon_geometry((polygon_t *)ptr_vector_at(subj, 0), &overlap_area, center);
		polygon_geometry(poly1, &ellipse_area, &ellipse_pt);

		area_percent = overlap_area / ellipse_area * 100.0f;
	}
	else {
		area_percent = 0.0f;
		center->X = 0.0f;
		center->Y = 0.0f;
	}

    polygons_free(subj);

	return area_percent;
}


void calc_MN (double *M, double *N, double lat) 
{

	double a               = 6378137;
	double e2              = 0.00669437999019758;
	double W  = sqrt(1.0 - e2 * (sin(lat)*sin(lat)));

	*N  = a / W;
	*M  = (a * (1.0 - e2)) / pow(W ,3);

}


void
error_ellipse_init(polygon_t *ellipse, const double *cov, const vector2f_t *point)
{
	double A, B, phi, chisquare_val, x_r, y_r;
	double eigen_val[2], eigen_vec1[2], eigen_vec2[2], R[4];
	const double incr = TWOPI / 16.0;
	uint32_t i;

	R[0] = cov[0]; R[2] = cov[4];
	R[1] = cov[1]; R[3] = cov[5];

	eigenvector_2x2(eigen_val, eigen_vec1, eigen_vec2, R);

	phi = atan2(eigen_vec1[1], eigen_vec1[0]);

	chisquare_val = 1.5137;

	A = chisquare_val * sqrt(MAX(eigen_val[0], eigen_val[1]));
	B = chisquare_val * sqrt(MIN(eigen_val[0], eigen_val[1]));

	R[0] = cos(phi);  R[2] = sin(phi);
	R[1] = -sin(phi);  R[3] = cos(phi);

	polygon_init(ellipse);

	for (i = 0; i < 16; i++) {
		vector2f_t tmp;
		x_r = A * cos(incr * i);
		y_r = B * sin(incr * i);
		tmp.X = (float)(x_r * R[0] + y_r * R[1]);
		tmp.Y = (float)(x_r * R[2] + y_r * R[3]);

		tmp.X += point->X;
		tmp.Y += point->Y;
		polygon_add_vertex(ellipse, &tmp);
	}

	polygon_close_path(ellipse);
}


void
error_ellipse_update(polygon_t *ellipse, const double *cov, const vector2f_t *point)
{
	double A, B, phi, chisquare_val, x_r, y_r;
	double eigen_val[2], eigen_vec1[2], eigen_vec2[2], R[4];
	const double incr = TWOPI / 16.0;
	uint32_t i = 0;
	edgelist_t *edgelist;

	R[0] = cov[0]; R[2] = cov[4];
	R[1] = cov[1]; R[3] = cov[5];

	eigenvector_2x2(eigen_val, eigen_vec1, eigen_vec2, R);

	phi = atan2(eigen_vec1[1], eigen_vec1[0]);

	chisquare_val = 1.5137;

	A = chisquare_val * sqrt(MAX(eigen_val[0], eigen_val[1]));
	B = chisquare_val * sqrt(MIN(eigen_val[0], eigen_val[1]));

	R[0] =  cos(phi);  R[2] = sin(phi);
	R[1] = -sin(phi);  R[3] = cos(phi);

	list_for_each_entry(edgelist, edgelist_t, &ellipse->edge_list, list) {
		vector2f_t tmp;
		x_r = A * cos(incr * i);
		y_r = B * sin(incr * i);
		tmp.X = (float)(x_r * R[0] + y_r * R[1]);
		tmp.Y = (float)(x_r * R[2] + y_r * R[3]);

		tmp.X += point->X;
		tmp.Y += point->Y;

		edgelist->edge.start = tmp;
		i++;
	}

	polygon_close_path(ellipse);
}










