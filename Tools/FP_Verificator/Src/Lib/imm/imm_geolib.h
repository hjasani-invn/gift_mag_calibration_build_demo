#ifndef _IMM_GEOLIB_H_
#define _IMM_GEOLIB_H_


#include "imm_types.h"
#include "imm_geoobject.h"

IMM_BEGIN_DECLS

void angle_correct(float *angle);


float angle_diff(float angle1, float angle2);


bool_t
vector2f_intersect(vector2f_t *inter_pt,
                   const vector2f_t *p1,
                   const vector2f_t *p2,
                   const vector2f_t *q1,
                   const vector2f_t *q2);

float vector2f_dot(const vector2f_t *p1, 
	               const vector2f_t *p2);

float vector2f_slope(const vector2f_t *p1, const vector2f_t *p2);

vector2f_t vector2f_edge_projection(const vector2f_t *p, 
	                                const edge_t *edge);

bool_t edge_intersect(vector2f_t *inter_pt,
	                  const edge_t *edge1, 
                      const edge_t *edge2);

int8_t vector2f_side_of_edge(const vector2f_t *point,
	                         const edge_t *edge);

bool_t point_on_line(const vector2f_t *point, 
                     const vector2f_t *start_pt, 
					 const vector2f_t *end_pt, 
					 float tolerance);


bool_t point_in_polygon(const vector2f_t* point, 
	                    const polygon_t *polygon);


void calc_MN (double *M, 
	          double *N, 
			  double lat);


void error_ellipse_init(polygon_t *ellipse, 
	                    const double *cov, 
				        const vector2f_t *point);


void error_ellipse_update(polygon_t *ellipse,
	                      const double *cov,
	                      const vector2f_t *point);

void polygon_offset(polygon_t *poly, 
	                float offset, 
					JoinType_t join_type, 
					uint32_t scale);


void polygon_clip(ptr_vector_t *subject_in,
	              const polygon_t *clip_in,
	              ClipType_t clip_type,
	              uint32_t scale);

void polygon_geometry(const polygon_t *poly, 
	                  float *area, 
					  vector2f_t *center);

float polygons_overlap_area_percentage(const polygon_t *poly1,
	                                   const polygon_t *poly2,
	                                   const ClipType_t ct,
	                                   vector2f_t *center);

IMM_END_DECLS


#endif