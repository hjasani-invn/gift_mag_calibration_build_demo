#include "imm_geo_int64.h"
#include <string.h>


bool_t
int_vertex_equal(const vector2d_t *pt1, const vector2d_t *pt2)
{
	return (pt1->X == pt2->X && pt1->Y == pt2->Y);
}

void
int_vertex_copy(vector2d_t *dest, const vector2d_t *src)
{
	memcpy(dest, src, sizeof (*src));
}

int_polygon_t*
int_polygon_new()
{
	int_polygon_t* polygon;
	polygon = vector_new(sizeof (vector2d_t));

	return polygon;
}

int_polygon_t*
int_polygon_sized_new(const uint32_t num_vertices)
{
	int_polygon_t* polygon;
	polygon = vector_new(sizeof (vector2d_t));
	vector_resize(polygon, num_vertices);

	return polygon;
}

void
int_polygon_free(int_polygon_t *polygon)
{
	vector_free(polygon);
}

uint32_t
int_polygon_size(const int_polygon_t *polygon)
{
	return polygon->len;
}

void
int_polygon_resize(int_polygon_t *polygon, uint32_t size)
{
	vector_resize(polygon, size);
}

void
int_polygon_append_vertice(int_polygon_t *polygon, vector2d_t int_vertex)
{
	vector_append_element(polygon, int_vertex);
}

void
int_polygon_remove_vertice(int_polygon_t *polygon, uint32_t index)
{
	vector_remove_element(polygon, index);
}

void
int_polygon_reverse(int_polygon_t* polygon)
{
	uint32_t i, j;
	vector2d_t temp;

	for (i = 0, j = polygon->len - 1; i < j; i++, j--) {
		temp = int_polygon_at(polygon, i);
		int_polygon_at(polygon, i) = int_polygon_at(polygon, j);
		int_polygon_at(polygon, j) = temp;
	}
}

void
int_polygon_copy_vertex(int_polygon_t *poly1, uint32_t index1,
	const int_polygon_t *poly2, uint32_t index2)
{
	int_polygon_at(poly1, index1).X = int_polygon_at(poly2, index2).X;
	int_polygon_at(poly1, index1).Y = int_polygon_at(poly2, index2).Y;
}

void
int_polygon_printf(int_polygon_t *poly)
{
	uint32_t i;

	printf("\tRAM Address: %x\n", poly);
	printf("\tNumber of vertices: %d\n", poly->len);
	for (i = 0; i < poly->len; i++) {
		printf("\t\tX = %lld, Y = %lld \n", int_polygon_at(poly, i).X, int_polygon_at(poly, i).Y);
	}
}

int_polygons_t*
int_polygons_new()
{
	int_polygons_t* polygons;
	polygons = ptr_vector_new();
	return polygons;
}

int_polygons_t*
int_polygons_sized_new(const uint32_t polygon_num)
{
	int_polygons_t* polygons;
	polygons = ptr_vector_new();
	ptr_vector_resize(polygons, polygon_num);
	return polygons;
}

void
int_polygons_clear(int_polygons_t* polygons, bool_t free_segment)
{
	ptr_vector_clear(polygons, free_segment);
}


void
int_polygons_free(int_polygons_t *polygons, bool_t free_segment)
{
	ptr_vector_free(polygons, free_segment);
}


void
int_polygons_resize(int_polygons_t *polys, uint32_t size)
{
	ptr_vector_resize(polys, size);
}


void
int_polygons_append_element(int_polygons_t *polygons, int_polygon_t *data)
{
	ptr_vector_append_element(polygons, data);
}


int_polygons_t*
int_polygons_remove_element(int_polygons_t *polygons, uint32_t index_, bool_t free_segment)
{
	if (free_segment) {
		int_polygon_free(int_polygons_at(polygons, index_));
	}

	return (int_polygons_t *)ptr_vector_remove_element(polygons, index_);
}


void
int_polygons_copy(int_polygons_t* dest, int_polygons_t *src, bool_t free_segment)
{
	uint32_t i, j;

	int_polygons_clear(dest, free_segment);

	ptr_vector_resize(dest, src->len);

	for (i = 0; i < src->len; i++) {
		if (free_segment) {
			(dest->pdata)[i] = int_polygon_new();
		}

		for (j = 0; j < int_polygons_at(src, i)->len; j++) {
			int_polygon_append_vertice(int_polygons_at(dest, i), int_polygons_vertex(src, i, j));
		}
	}
}


void
int_polygons_reverse(int_polygons_t* polygons)
{
	uint32_t i;

	for (i = 0; i < polygons->len; i++) {
		int_polygon_reverse(int_polygons_at(polygons, i));
	}

}


void
int_polygons_printf(int_polygons_t *polys)
{
	uint32_t i;
	printf("RAM Address: %x\n", polys);
	printf("Number of vertices: %d\n", polys->len);

	for (i = 0; i < polys->len; i++) {
		int_polygon_printf(int_polygons_at(polys, i));
	}
}
