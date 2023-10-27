#include "imm_geoobject.h"
#include "imm_memory.h"
#include "imm_list.h"

#include <math.h>
#include <string.h>

bool_t
vector2f_equal(const vector2f_t *pt1, const vector2f_t *pt2)
{
	return (NEAR_EQUAL(pt1->X, pt2->X) && NEAR_EQUAL(pt1->Y, pt2->Y));
}


float
vector2f_distance(const vector2f_t *p1, const vector2f_t *p2)
{
	return sqrtf((p1->X - p2->X) * (p1->X - p2->X) + (p1->Y - p2->Y) * (p1->Y - p2->Y));
}


void
vector2f_swap(vector2f_t *p1, vector2f_t *p2)
{
	vector2f_t tmp = *p1;
	*p1 = *p2;
	*p2 = tmp;

	return;
}


void 
correct_slope(float *slope)
{
	float temp;

	temp = (*slope > 180.0f) ? *slope - 360.0f : (*slope < -180.0f) ? *slope + 360.0f : *slope;

	*slope = temp;

	return;
}


bool_t 
edge_equal(const edge_t *edge1, const edge_t *edge2)
{
	return vector2f_equal(&edge1->start, &edge2->start)
		&& vector2f_equal(&edge1->end, &edge2->end);
}


float edge_slope(const edge_t *edge)
{
	const vector2f_t *p1 = &edge->start;
	const vector2f_t *p2 = &edge->end;

	if (NEAR_EQUAL(p1->X, p2->X)) {
		return (p2->Y > p1->Y) ? 0.0f : 180.0f;
	}
	else if (NEAR_EQUAL(p1->Y, p2->Y)) {
		return (p2->X > p1->X) ? 90.0f : -90.0f;
	}
	else {
		float slope = (float)(90.0 - atan2((p2->Y - p1->Y), (p2->X - p1->X)) * RAD2DEG);
		correct_slope(&slope);
		return slope;
	}
}


edge_t * 
edge_next(const polygon_t *polygon, const edgelist_t* edgelist)
{
	edgelist_t *next;

	next = list_entry(edgelist->list.next, edgelist_t, list);

	if (&next->list == &polygon->edge_list) {
		next = list_entry(polygon->edge_list.next, edgelist_t, list);
	}

	return &next->edge;
}


edge_t * edge_prev(const polygon_t *polygon, const edgelist_t* edgelist)
{
	edgelist_t *prev;

	prev = list_entry(edgelist->list.prev, edgelist_t, list);

	if (&prev->list == &polygon->edge_list) {
		prev = list_entry(polygon->edge_list.prev, edgelist_t, list);
	}

	return &prev->edge;
}


void 
polygon_init(polygon_t *polygon)
{
	INIT_LIST_HEAD(&polygon->edge_list);
	polygon->num = 0;
}


void 
polygon_add_vertex(polygon_t *polygon, const vector2f_t *vertex)
{
	edgelist_t *tmp;

	tmp = (edgelist_t *)mem_allocate(sizeof(edgelist_t));
	memset(tmp, 0, sizeof(edgelist_t));
	tmp->id = polygon->num;
	tmp->edge.start = *vertex;

	list_add_tail(&(tmp->list), &polygon->edge_list);

	polygon->num++;
}


void 
polygon_close_path(polygon_t *polygon)
{
	edgelist_t *tmp_edge, *next_edge;
	polygon->num = 0;

	list_for_each_entry(tmp_edge, edgelist_t, &polygon->edge_list, list) {
		if (tmp_edge->list.next != (&polygon->edge_list)) {
			next_edge = list_entry(tmp_edge->list.next, edgelist_t, list);
		}
		else {
			next_edge = list_entry(polygon->edge_list.next, edgelist_t, list);
		}
		
		tmp_edge->edge.end = next_edge->edge.start;
		tmp_edge->edge.slope = edge_slope(&tmp_edge->edge);

		polygon->num++;
	}
}


void 
polygon_deep_cpy(polygon_t *dst, const polygon_t *src)
{
	edgelist_t *edge, *tmp;

	dst->num = src->num;

	list_for_each_entry(edge, edgelist_t, &src->edge_list, list){
		tmp = (edgelist_t *)mem_allocate(sizeof(edgelist_t));
		memcpy(tmp, edge, sizeof(edgelist_t));
		list_add_tail(&tmp->list, &dst->edge_list);
	}

}


bool_t 
polygon_free(polygon_t *polygon)
{
	edgelist_t *tmp;
	struct list_head *pos, *q;

	list_for_each_safe(pos, q, &polygon->edge_list) {
		tmp = list_entry(pos, edgelist_t, list);
		list_del(pos);
		mem_free(tmp);
	}
	return TRUE;
}


void 
polygon_printf(const polygon_t *polygon)
{
	edgelist_t *edgelist;
	printf("Vertex Num: %d\n", polygon->num);
	list_for_each_entry(edgelist, edgelist_t, &polygon->edge_list, list) {
		printf("\tID: %d\n\t%f %f\t%f %f\t%f\n", edgelist->id, 
			edgelist->edge.start.X, edgelist->edge.start.Y,
			edgelist->edge.end.X, edgelist->edge.end.Y,
			edgelist->edge.slope);
	}
}


//void
//plot_polygon(const polygon_t *polygon)
//{
//	FILE *fid = fopen("c:\\test_data\\map_debug_shape.txt", "wt");
//	edgelist_t *edgelist;
//	fprintf(fid, "%d ", polygon->num);
//	list_for_each_entry(edgelist, edgelist_t, &polygon->edge_list, list) {
//		fprintf(fid, "%f %f ", edgelist->edge.start.X, edgelist->edge.start.Y);
//	}
//
//	fclose(fid);
//}


void 
polygons_clear(ptr_vector_t *polygons)
{
	uint32_t i;
	for (i = 0; i < polygons->len; i++) {
		polygon_t *polygon = (polygon_t *)ptr_vector_at(polygons, i);
		polygon_free(polygon);
	}

	polygons->len = 0;
}


void 
polygons_free(ptr_vector_t *polygons)
{
	uint32_t i;
	for (i = 0; i < polygons->len; i++) {
		polygon_t *polygon = (polygon_t *)ptr_vector_at(polygons, i);
		polygon_free(polygon);
	}

	ptr_vector_free(polygons, TRUE);
}





