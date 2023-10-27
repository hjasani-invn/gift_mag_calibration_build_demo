#ifndef _IMM_GEOOBJECT_H_
#define _IMM_GEOOBJECT_H_


#include "imm_types.h"
#include "imm_vector.h"
#include "imm_list.h"


IMM_BEGIN_DECLS

typedef struct  {
	int64_t X;
	int64_t Y;
} vector2d_t;


typedef struct {
	float X;
	float Y;
} vector2f_t;


typedef struct {
	vector2f_t start;
	vector2f_t end;
	float      slope;
} edge_t;


typedef struct {
	int       id;
	edge_t    edge;
	struct    list_head list;
} edgelist_t;


typedef struct {
	int       num;
	struct    list_head edge_list;
}polygon_t;


typedef enum { ct_intersection, ct_union, ct_difference, ct_xor } ClipType_t;
typedef enum { pt_subject, pt_clip } PolyType_t;
typedef enum { pf_even_odd, pf_non_zero, pf_positive, pf_negative } PolyFillType_t;
typedef enum { jt_square, jt_round, jt_miter }JoinType_t;

#ifndef PI
#define PI        (3.1415926535897932384626433832795)    //!< better value
#endif

#ifndef TWOPI
#define TWOPI     (6.283185307179586476925286766559)     //!< 2.0*PI
#endif

#ifndef DEG2RAD
#define DEG2RAD   (0.017453292519943295769236907684886)  //!< PI/180.0
#endif

#ifndef RAD2DEG
#define RAD2DEG   (57.295779513082320876798154814105)    //!< 180.0/PI
#endif


EXTERN_SYMBOL bool vector2f_equal(const vector2f_t *point1, const vector2f_t *point2);

void vector2f_swap(vector2f_t *p1, vector2f_t *p2);

float vector2f_distance(const vector2f_t *p1, const vector2f_t *p2);

EXTERN_SYMBOL void correct_slope(float *slope);

bool edge_equal(const edge_t *edge1, const edge_t *edge2);

float edge_slope(const edge_t *edge);

edge_t *edge_prev(const polygon_t *polygon, const edgelist_t* edgelist);

edge_t *edge_next(const polygon_t *polygon, const edgelist_t* edgelist);

void polygon_init(polygon_t *polygon);

void polygon_add_vertex(polygon_t *polygon, const vector2f_t *vertex);

void polygon_close_path(polygon_t *polygon);

void polygon_get_bbox(const polygon_t *polygon, float *bbox);

void polygon_deep_cpy(polygon_t *dst, const polygon_t *src);

bool polygon_free(polygon_t *polygon);

void polygon_printf(const polygon_t *polygon);

//void plot_polygon(const polygon_t *polygon);

void polygons_free(ptr_vector_t *polygons);

void polygons_clear(ptr_vector_t *polygons);


IMM_END_DECLS


#endif