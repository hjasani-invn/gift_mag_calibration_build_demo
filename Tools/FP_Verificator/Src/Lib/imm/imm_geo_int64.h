#ifndef _IMM_GEO_INT64_H_
#define _IMM_GEO_INT64_H_

#include "imm_geoobject.h"

IMM_BEGIN_DECLS

typedef vector_t     int_polygon_t;
typedef ptr_vector_t int_polygons_t;


#define int_polygon_at(a,i)              (((vector2d_t*) (void *) (a)->data) [(i)])
#define int_polygons_at(a,i)             ((int_polygon_t *)((a)->pdata)[i]) 
#define int_polygons_vertex(a, i, j)      int_polygon_at (int_polygons_at (a,i), j)

bool_t int_vertex_equal(const vector2d_t *pt1, const vector2d_t *pt2);

void int_vertex_copy(vector2d_t *dest, const vector2d_t *src);

int_polygon_t* int_polygon_new();

int_polygon_t* int_polygon_sized_new(const uint32_t num_vertices);

void int_polygon_free(int_polygon_t *polygon);

uint32_t int_polygon_size(const int_polygon_t *polygon);

void int_polygon_resize(int_polygon_t *polygon, uint32_t size);

void int_polygon_append_vertice(int_polygon_t *polygon, vector2d_t int_vertex);

void int_polygon_remove_vertice(int_polygon_t *polygon, uint32_t index);

void int_polygon_reverse(int_polygon_t* polygon);

void int_polygon_copy_vertex(int_polygon_t *poly1, uint32_t index1, const int_polygon_t *poly2, uint32_t index2);

void int_polygon_printf(int_polygon_t *poly);

int_polygons_t* int_polygons_new();

int_polygons_t* int_polygons_sized_new(const uint32_t polygon_num);

void int_polygons_clear(int_polygons_t* polygons, bool_t free_segment);

void int_polygons_free(int_polygons_t *polygons, bool_t free_segment);

void int_polygons_resize(int_polygons_t *polys, uint32_t size);

void int_polygons_append_element(int_polygons_t *polygons, int_polygon_t *data);

int_polygons_t* int_polygons_remove_element(int_polygons_t *polygons, uint32_t index_, bool_t free_segment);

void int_polygons_copy(int_polygons_t* dest, int_polygons_t *src, bool_t free_segment);

void int_polygons_reverse(int_polygons_t* polygons);

void int_polygons_printf(int_polygons_t *polys);

IMM_END_DECLS

#endif