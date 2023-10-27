#include <string.h>
#include <math.h>

#include "imm_rtree.h"
#include "imm_math.h"
#include "imm_geolib.h"
#include "imm_memory.h"

#define MAX_RtreeEntries  5
#define MIN_RtreeEntries  ceil(MAX_RtreeEntries*0.4)


typedef struct {
	bool_t is_leaf;
	float bbox[4];
	vector_t *child;
	ptr_vector_t *data_items;
	uint32_t height;
}rtree_node_t;


typedef struct {
	int32_t lvl;
	rtree_node_t node;
	vector_t *items;
}lvl_node_t;


struct MapTree_Tag{
	vector_t *entity_rtree;
	vector_t *grid_rtree;
};


typedef struct {
	void *item_ptr;
	float bbox[4];
}rtree_item_t;

enum rtree_type { entity_rtree, grid_rtree };

static rtree_node_t inner_build(ptr_vector_t *item, const uint32_t level, uint32_t height);
static int32_t sort_by_x(const void *o1, const void *o2);
static int32_t sort_by_y(const void *o1, const void *o2);
static void item_sort(ptr_vector_t *item, uint32_t mode);
static void extend (float *bbox, const float *item_box);
static void dist_bbox(rtree_node_t *node, const uint32_t start_id, const uint32_t end_id);
static void all_in (ptr_vector_t *results, rtree_node_t *node);
static ptr_vector_t* bbox_search (const float *bbox, rtree_node_t *root);
static bool_t is_contain (const float* a, const float* b);
static bool_t is_intersect (const float *a, const float *b);
static void internal_nodes_printf (const rtree_node_t* node);
static void rtrees_nodes_free (rtree_node_t* node);



static rtree_node_t 
inner_build(ptr_vector_t *item, const uint32_t level, uint32_t height)
{
	uint32_t N = item->len, M = MAX_RtreeEntries, N1, N2, cmp_mode;
	uint32_t i, j, k;
	rtree_node_t node;
	rtree_item_t *item_p;
	ptr_vector_t *slice, *mslice;
	rtree_node_t child_node;

	if (N < M) {
		node.data_items = ptr_vector_new();
		for (i = 0; i < N; i++) {
			item_p = (rtree_item_t *)ptr_vector_at(item, i);
			ptr_vector_append_element(node.data_items, item_p);
		}

		node.is_leaf = TRUE;
		node.height = 1;
		dist_bbox(&node, 0, node.data_items->len);

		return node;
	}

	if (!level) {
		height = (uint32_t)ceil(log((double)N) / log((double)M));
		M = (uint32_t)ceil((double)N / pow((double)M, (int)height - 1));
		item_sort(item, 0);
	}

	node.child = vector_new(sizeof (rtree_node_t));
	node.height = height;
	node.is_leaf = FALSE;

	N1 = (uint32_t)(ceil((double)N / (double)M) * ceil(sqrt((double)M)));
	N2 = (uint32_t)ceil((double)N / (double)M);
	cmp_mode = 1 - (level % 2);

	for (i = 0; i < N; i += N1) {
		slice = ptr_vector_new();

		for (j = i; j < MIN(N, i + N1); j++) {
			item_p = (rtree_item_t *)ptr_vector_at(item, j);
			ptr_vector_append_element(slice, item_p);
		}

		item_sort(slice, cmp_mode);

		for (j = 0; j < slice->len; j += N2)  {
			mslice = ptr_vector_new();
			for (k = j; k< MIN(slice->len, j + N2); k++) {
				item_p = (rtree_item_t *)ptr_vector_at(slice, k);
				ptr_vector_append_element(mslice, item_p);
			}

			child_node = inner_build(mslice, level + 1, height - 1);
			vector_append_element(node.child, child_node);
			ptr_vector_free(mslice, TRUE);
		}

		ptr_vector_free(slice, TRUE);
	}

	dist_bbox(&node, 0, node.child->len);

	return node;

}


static int32_t
sort_by_x(const void *o1, const void *o2)
{
	float a, b;

	rtree_item_t* om1 = (rtree_item_t*)(*(void **)o1);
	rtree_item_t* om2 = (rtree_item_t*)(*(void **)o2);
	a = (om1->bbox[0] + om1->bbox[2]);
	b = (om2->bbox[0] + om2->bbox[2]);

	if (a < b)
		return -1;
	else if (a == b)
		return 0;
	else
		return 1;
}


static int32_t
sort_by_y(const void *o1, const void *o2)
{
	float a, b;

	rtree_item_t* om1 = (rtree_item_t*)(*(void **)o1);
	rtree_item_t* om2 = (rtree_item_t*)(*(void **)o2);
	a = (om1->bbox[1] + om1->bbox[3]);
	b = (om2->bbox[1] + om2->bbox[3]);

	if (a < b)
		return -1;
	else if (a == b)
		return 0;
	else
		return 1;
}


static void
item_sort(ptr_vector_t *item, uint32_t mode)
{

	if (mode == 0)
		ptr_vector_sort(item, sort_by_x);
	else
		ptr_vector_sort(item, sort_by_y);

}


static void
extend(float *bbox, const float *item_box)
{
	float tmp[4];
	uint32_t i;

	for (i = 0; i < 4; i++) {
		tmp[i] = bbox[i];
	}

	bbox[0] = 0.0f;
	bbox[1] = 0.0f;
	bbox[2] = 0.0f;
	bbox[3] = 0.0f;

	bbox[0] = MIN(tmp[0], item_box[0]);
	bbox[1] = MIN(tmp[1], item_box[1]);
	bbox[2] = MAX(tmp[2], item_box[2]);
	bbox[3] = MAX(tmp[3], item_box[3]);
}


static void
dist_bbox(rtree_node_t *node, const uint32_t start_id, const uint32_t end_id)
{
	float itembox[4];
	uint32_t i;
	rtree_item_t *data_i;

	node->bbox[0] = 1.0e10;
	node->bbox[1] = 1.0e10;
	node->bbox[2] = -1.0e10;
	node->bbox[3] = -1.0e10;

	for (i = start_id; i < end_id; i++) {

		if (node->is_leaf) {
			data_i = (rtree_item_t *) ptr_vector_at(node->data_items, i);
			itembox[0] = data_i->bbox[0];
			itembox[1] = data_i->bbox[1];
			itembox[2] = data_i->bbox[2];
			itembox[3] = data_i->bbox[3];
			
			extend (node->bbox, itembox);
		} else {
			extend(node->bbox, vector_at (node->child, rtree_node_t, i).bbox);
		}

	}
}


MapTree_t* rtrees_init(void)
{
	MapTree_t *p;
	p = (MapTree_t *)mem_allocate(sizeof(MapTree_t));
	memset(p, 0, sizeof(MapTree_t));
	return p;
}


bool_t
entity_rtrees_build(vector_t **entity_rtree, MapLvl_t *map_lvl)
{
	rtree_node_t node;
	vector_t *map_items;
	MapEntity_t *mapentity;
	uint32_t i, geo_num;
	lvl_node_t lvl_node;
	ptr_vector_t *items;

	if (*entity_rtree == NULL)
		*entity_rtree = vector_new(sizeof (lvl_node));

	node.is_leaf = FALSE;
	node.child = NULL;
	node.bbox[0] = 1.0e10;
	node.bbox[1] = 1.0e10;
	node.bbox[2] = -1.0e10;
	node.bbox[3] = -1.0e10;

	geo_num = map_lvl->mapentity->len;
	map_items = vector_new(sizeof(rtree_item_t));
	items = ptr_vector_new();

	vector_resize(map_items, geo_num);
	ptr_vector_resize(items, geo_num);

	for (i = 0; i < geo_num; i++) {
		mapentity = &vector_at(map_lvl->mapentity, MapEntity_t, i);
		rtree_item_t *item_tmp = &vector_at(map_items, rtree_item_t, i);
		item_tmp->item_ptr = mapentity;
		item_tmp->bbox[0] = mapentity->min_max[0];
		item_tmp->bbox[1] = mapentity->min_max[1];
		item_tmp->bbox[2] = mapentity->min_max[2];
		item_tmp->bbox[3] = mapentity->min_max[3];

		ptr_vector_at(items, i) = item_tmp;
	}

	node = inner_build(items, 0, 0);

	lvl_node.node = node;
	lvl_node.lvl = map_lvl->lvl;
	lvl_node.items = map_items;

	vector_append_element(*entity_rtree, lvl_node);
	ptr_vector_free(items, TRUE);

	return TRUE;
}


bool_t
grid_rtrees_build(vector_t **grid_rtree, MapLvl_t *map_lvl)
{
	rtree_node_t node;
	vector_t *grid_items;
	MapNode_t *mapnode;
	uint32_t i, grid_num;
	lvl_node_t lvl_node;
	ptr_vector_t *items;

	if (*grid_rtree == NULL)
		*grid_rtree = vector_new(sizeof (lvl_node));

	node.is_leaf = FALSE;
	node.child = NULL;
	node.bbox[0] = 1.0e10;
	node.bbox[1] = 1.0e10;
	node.bbox[2] = -1.0e10;
	node.bbox[3] = -1.0e10;

	grid_num = map_lvl->mapnodes->len;
	grid_items = vector_new(sizeof(rtree_item_t));
	items = ptr_vector_new();

	vector_resize(grid_items, grid_num);
	ptr_vector_resize(items, grid_num);

	for (i = 0; i < grid_num; i++) {
		mapnode = &vector_at(map_lvl->mapnodes, MapNode_t, i);
		rtree_item_t *item_tmp = &vector_at(grid_items, rtree_item_t, i);

		item_tmp->item_ptr = mapnode;
		item_tmp->bbox[0] = mapnode->map_pos.X - 1.0f;
		item_tmp->bbox[1] = mapnode->map_pos.Y - 1.0f;
		item_tmp->bbox[2] = mapnode->map_pos.X + 1.0f;
		item_tmp->bbox[3] = mapnode->map_pos.Y + 1.0f;

		ptr_vector_at(items, i) = item_tmp;
	}

	node = inner_build(items, 0, 0);

	lvl_node.node = node;
	lvl_node.lvl = map_lvl->lvl;
	lvl_node.items = grid_items;

	vector_append_element(*grid_rtree, lvl_node);

	ptr_vector_free(items, TRUE);

	return TRUE;
}


bool_t
rtrees_build(MapTree_t *map_tree, MapLvl_t *map_lvl)
{
	entity_rtrees_build(&map_tree->entity_rtree, map_lvl);

	if (map_lvl->mapnodes->len > 0) {
		grid_rtrees_build(&map_tree->grid_rtree, map_lvl);
	}

	return TRUE;
}


static bool_t
is_intersect(const float *a, const float *b)
{
	return  (b[0] <= a[2]) & (b[1] <= a[3]) & (b[2] >= a[0]) & (b[3] >= a[1]);
}


static bool_t
is_contain(const float* a, const float* b)
{
	return  (a[0] <= b[0]) & (a[1] <= b[1]) & (b[2] <= a[2]) & (b[3] <= a[3]);
}


static void
all_in(ptr_vector_t *results, rtree_node_t *node)
{
	uint32_t i, j;
	ptr_vector_t *nodes_to_search = ptr_vector_new();

	while (node) {
		if (node->is_leaf) {
			for (j = 0; j < node->data_items->len; j++) {
				rtree_item_t *map_entity = (rtree_item_t *)ptr_vector_at(node->data_items, j);
				ptr_vector_append_element (results, map_entity);
			}	
		} else {
			for (i = 0; i < node->child->len; i++) { 
				rtree_node_t* child_i = & vector_at (node->child, rtree_node_t, i);
				ptr_vector_append_element(nodes_to_search, child_i);
			}
		}
		if (!ptr_vector_empty(nodes_to_search)) {
			node = (rtree_node_t *) ptr_vector_at (nodes_to_search, nodes_to_search->len-1);
			ptr_vector_remove_element (nodes_to_search, nodes_to_search->len-1);
		} else {
			node = NULL;
		}

	}

	ptr_vector_free(nodes_to_search, TRUE);
}


static ptr_vector_t*
bbox_search(const float* bbox, rtree_node_t *root)
{
	rtree_node_t *node, *child_i;
	rtree_item_t *rtree_item;
	ptr_vector_t *nodes_to_search = ptr_vector_new();
	ptr_vector_t *results = ptr_vector_new();
	uint32_t i, j;

	if (!is_intersect(bbox, root->bbox)) {
		ptr_vector_free(nodes_to_search, TRUE);
		return results;
	}

	node = root;

	while (node != NULL) {
		if (node->is_leaf == FALSE) {
			for (i = 0; i < node->child->len; i++) {
				child_i = &vector_at(node->child, rtree_node_t, i);
				if (is_intersect(bbox, child_i->bbox)) {
					if (child_i->is_leaf) {
						for (j = 0; j < child_i->data_items->len; j++) {
							rtree_item = (rtree_item_t *)ptr_vector_at(child_i->data_items, j);
							if (is_intersect(bbox, rtree_item->bbox)) {
								ptr_vector_append_element(results, rtree_item);
							}
						}					
					} else if (is_contain(bbox, child_i->bbox)) {
						all_in (results, child_i);
					} else {
						ptr_vector_append_element(nodes_to_search, child_i);
					}
				}
			}
		} else {
			for (j = 0; j < node->data_items->len; j++) {
				rtree_item = (rtree_item_t *)ptr_vector_at(node->data_items, j);
				if (is_intersect(bbox, rtree_item->bbox)) {
					ptr_vector_append_element(results, rtree_item);
				}
			}
		}

		if (!ptr_vector_empty(nodes_to_search)) {
			node = (rtree_node_t *)ptr_vector_at(nodes_to_search, nodes_to_search->len - 1);
			ptr_vector_remove_element(nodes_to_search, nodes_to_search->len - 1);
		}
		else {
			node = NULL;
		}
	}

	ptr_vector_free(nodes_to_search, TRUE);
	return results;
}


static rtree_node_t*
get_node(const vector_t *map_tree, int32_t lvl)
{
	lvl_node_t *lvl_node;
	uint32_t i;

	if (map_tree) {
		for (i = 0; i < map_tree->len; i++) {
			lvl_node = &vector_at(map_tree, lvl_node_t, i);
			if (lvl_node->lvl == lvl)
				return &lvl_node->node;
		}
	}

	return NULL;
}


MapErr_t 
point_search(const MapTree_t *map_tree, 
	         const vector2f_t *point, 
			 const int32_t lvl, 
			 MapEntity_t **mapentity)
{
	rtree_node_t *node = get_node(map_tree->entity_rtree, lvl);
	ptr_vector_t *results;
	uint32_t i;
	rtree_item_t *rtree_item_i = NULL;
	MapEntity_t *map_entity_i = NULL, *ptr_road = NULL, *ptr_back = NULL;
	float bbox[4];

	*mapentity = NULL;
	return_val_if_fail(node != NULL, imm_invalid_lvl);

	bbox[0] = point->X;
	bbox[1] = point->Y;
	bbox[2] = point->X;
	bbox[3] = point->Y;

	results = bbox_search(bbox, node);

	for (i = 0; i < results->len; i++) {
		rtree_item_i = (rtree_item_t *)ptr_vector_at(results, i);
		map_entity_i = (MapEntity_t *)rtree_item_i->item_ptr;

		if (map_entity_i->type == imm_road) {
			ptr_road = map_entity_i;
			continue;
		} else if (map_entity_i->is_ignored) {
			continue;
		} else if (strcmp(map_entity_i->name,  "Customer Services") == 0) {
			continue;
		} else if (strcmp(map_entity_i->name,  "Stair Vest") == 0) {
			continue;
		} else if (map_entity_i->type == imm_background) {
			ptr_back = map_entity_i;
			continue;
		} else if (map_entity_i->type == imm_lvl_change) {
			continue;
		} else if (map_entity_i->type == imm_entrance) {
			continue;
		} else {
			if (point_in_polygon(point, &map_entity_i->shape)) {
				ptr_vector_free(results, TRUE);
				*mapentity = map_entity_i;
				return imm_success;
			}
		}
	}

	if (ptr_back != NULL) {
		if (point_in_polygon(point, &ptr_back->shape)) {
			ptr_vector_free(results, TRUE);
			*mapentity =  ptr_back;
			return imm_success;
		} else {
			if (ptr_road != NULL) {
				ptr_vector_free(results, TRUE);
				*mapentity = ptr_road;
				return imm_success;
			}
		}
	}

	ptr_vector_free(results, TRUE);

	*mapentity = NULL;
	return imm_invalid_pos;
}


MapErr_t
point_range_name_search(const MapTree_t *map_tree, 
                        const vector2f_t *point, 
						float range, 
						const int32_t lvl, 
						char *name,
						ptr_vector_t **mapentities)
{
	rtree_node_t *node = get_node(map_tree->entity_rtree, lvl);
	ptr_vector_t *results;
	ptr_vector_t *ret; 
	rtree_item_t *rtree_item_i = NULL;
	MapEntity_t *map_entity_i;
	uint32_t i;
	float bbox[4];

	*mapentities = NULL;
	return_val_if_fail(node != NULL, imm_invalid_lvl);
	return_val_if_fail(range > 0.0f, imm_invalid_range);

	ret = ptr_vector_new();

	bbox[0] = point->X - range;
	bbox[1] = point->Y - range;
	bbox[2] = point->X + range;
	bbox[3] = point->Y + range;

	results = bbox_search(bbox, node);

	for (i = 0; i < results->len; i++) {
		rtree_item_i = (rtree_item_t *)ptr_vector_at(results, i);
		map_entity_i = (MapEntity_t *)rtree_item_i->item_ptr;

		if (strcmp(map_entity_i->name, name) == 0)
			ptr_vector_append_element(ret, map_entity_i);
	}

	ptr_vector_free(results, TRUE);

	if (ret->len > 0) {
		*mapentities = ret;
		return imm_success;
	}
	else  {
		*mapentities = NULL;
		return imm_invalid_pos;
	}
}


MapErr_t node_range_search(const MapTree_t *map_tree,
	const vector2f_t *point,
	float range,
	const int32_t lvl,
	ptr_vector_t **mapnodes)
{
	rtree_node_t *node = get_node(map_tree->grid_rtree, lvl);
	ptr_vector_t *results;
	ptr_vector_t *ret; 
	rtree_item_t *rtree_item_i = NULL;
	MapNode_t *map_node_i;
	uint32_t i;
	float bbox[4];

	*mapnodes = NULL;
	return_val_if_fail(node != NULL, imm_invalid_lvl);
	return_val_if_fail(range > 0.0f, imm_invalid_range);

	ret = ptr_vector_new();

	bbox[0] = point->X - range;
	bbox[1] = point->Y - range;
	bbox[2] = point->X + range;
	bbox[3] = point->Y + range;

	results = bbox_search(bbox, node);

	for (i = 0; i < results->len; i++) {
		rtree_item_i = (rtree_item_t *)ptr_vector_at(results, i);
		map_node_i = (MapNode_t *)rtree_item_i->item_ptr;

		ptr_vector_append_element(ret, map_node_i);
	}

	ptr_vector_free(results, TRUE);

	if (ret->len > 0) {
		*mapnodes = ret;
		return imm_success;
	}
	else  {
		*mapnodes = NULL;
		return imm_invalid_pos;
	}
}


static void
rtrees_nodes_free(rtree_node_t* node)
{
	uint32_t i;
	rtree_node_t* child;

	if (node->is_leaf) {
		ptr_vector_free(node->data_items, TRUE);
		return;
	}

	for (i = 0; i < node->child->len; i++) {
		child = &vector_at(node->child, rtree_node_t, i);
		rtrees_nodes_free(child);
	}

	vector_free(node->child);
}


void
rtrees_free(MapTree_t *map_tree)
{
	uint32_t i;

	if (map_tree->entity_rtree) {
		for (i = 0; i < map_tree->entity_rtree->len; i++) {
			lvl_node_t *lvl_node = &vector_at(map_tree->entity_rtree, lvl_node_t, i);
			rtrees_nodes_free(&lvl_node->node);
			vector_free(lvl_node->items);
		}
		vector_free(map_tree->entity_rtree);
		map_tree->entity_rtree = NULL;
	}

	if (map_tree->grid_rtree) {
		for (i = 0; i < map_tree->grid_rtree->len; i++) {
			lvl_node_t *lvl_node = &vector_at(map_tree->grid_rtree, lvl_node_t, i);
			rtrees_nodes_free(&lvl_node->node);
			vector_free(lvl_node->items);
		}
		vector_free(map_tree->grid_rtree);
		map_tree->grid_rtree = NULL;
	}
	
	mem_free(map_tree);	
}


static void
internal_nodes_printf(const rtree_node_t* node)
{
	uint32_t i;
	rtree_node_t* child;
	rtree_item_t* item;
	MapEntity_t *map_entity;

	if (node->is_leaf) {
		for (i = 0; i < node->data_items->len; i++) {
			item = (rtree_item_t *)ptr_vector_at(node->data_items, i);
			map_entity = (MapEntity_t *)item->item_ptr;
			printf("%d \n", map_entity->id);
		}
		printf("\n");
		return;
	}

	for (i = 0; i < node->child->len; i++) {
		child = &vector_at(node->child, rtree_node_t, i);
		printf("ID = %d, Height = %d\n", i, child->height);
		internal_nodes_printf(child);
	}
	printf("\n");

}


void
rtrees_printf(const MapTree_t *map_tree)
{
	uint32_t i;

	return_if_fail(map_tree->entity_rtree);

	for (i = 0; i < map_tree->entity_rtree->len; i++) {
		printf("ID = %d, Height = %d\n", i, vector_at(map_tree->entity_rtree, rtree_node_t, i).height);
		internal_nodes_printf(&vector_at(map_tree->entity_rtree, lvl_node_t, i).node);
	}

}