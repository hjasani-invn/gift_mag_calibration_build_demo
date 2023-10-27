#include "imm_math.h"
#include "imm_memory.h"
#include "imm_mapobject.h"
#include "imm_geolib.h"
#include "imm_rtree.h"

#include <string.h>
#include <math.h>


static uint32_t binary_read (void *dest, uint32_t size, uint32_t num, const uint8_t **src);
static void load_shape_from_bin_file(polygon_t *shape, const uint8_t **src);
static bool_t load_binary_mapfile(MapObject_t* map_object, const uint8_t *binary_array, uint32_t size);


MapObject_t* 
mapobject_init(const uint8_t *data_array, uint32_t size)
{
	MapObject_t *mapobject = (MapObject_t *)mem_allocate(sizeof(MapObject_t));

	return_val_if_fail(mapobject != NULL, NULL);

	memset(mapobject, 0, sizeof(MapObject_t));

	if (load_binary_mapfile(mapobject, data_array, size))
		return mapobject;
	else
		return NULL;
}


bool_t
mapobject_process(MapObject_t *map_object)
{
	uint32_t i, j, k, m;
	MapDrawing_t *mapdrawing;
	MapLvl_t *maplvl;

	map_object->map_tree = rtrees_init();

	for (i = 0; i < map_object->mapdrawing->len; i++) {
		mapdrawing = &vector_at(map_object->mapdrawing, MapDrawing_t, i);

		for (j = 0; j < mapdrawing->maplvl->len; j++) {
			maplvl = &vector_at(mapdrawing->maplvl, MapLvl_t, j);
			rtrees_build(map_object->map_tree, maplvl);

			// Init Node ptr_vector
			for (k = 0; k < maplvl->mapnodes->len; k++) {
				MapNode_t *node = &vector_at(maplvl->mapnodes, MapNode_t, k);
				node->link_node = ptr_vector_new();

				for (m = 0; m < node->link_node_id->len; m++) {
					MapNode_t *child_j;
					uint32_t child_id = vector_at(node->link_node_id, uint32_t, m);
					mapobject_get_mapnode_id(map_object, child_id, maplvl->lvl, &child_j);
					ptr_vector_append_element(node->link_node, child_j);
				}
			}
		}
	}

	//vector2f_t pos;
	//pos.X = 64.0f; pos.Y = -46.0f;
	//MapEntity_t *entity;
	//MapNode_t *node;
	//ptr_vector_t *results;
	//MapErr_t val = mapobject_get_mapentity(map_object, &pos, 1, &entity);
	//val = mapobject_get_mapnode_range(map_object, &pos, 1.5f, 1,  &results);

	//if (val == imm_success){
	//	for (i = 0; i < results->len; i++) {
	//		node = (MapNode_t *)ptr_vector_at(results, i);
	//	}
	//}

	//ptr_vector_free(results, TRUE);
	return TRUE;
}


bool_t 
mapobject_free (MapObject_t *map_object)
{
	uint32_t i, j, k;

	if (map_object != NULL && map_object->map_tree != NULL) {
		rtrees_free(map_object->map_tree);
		map_object->map_tree = NULL;
	}
		
	if (map_object != NULL) {
		for (i = 0; i < map_object->mapdrawing->len; i++) {
			MapDrawing_t *mapdrawing = &vector_at(map_object->mapdrawing, MapDrawing_t, i);

			for (j = 0; j < mapdrawing->maplvl->len; j++) {
				MapLvl_t *maplvl = &vector_at(mapdrawing->maplvl, MapLvl_t, j);

				for (k = 0; k < maplvl->mapnodes->len; k++) {
					MapNode_t *mapnode = &vector_at(maplvl->mapnodes, MapNode_t, k);
					vector_free(mapnode->link_node_id);
					ptr_vector_free(mapnode->link_node, TRUE);
				}

				vector_free(maplvl->mapnodes);

				for (k = 0; k < maplvl->mapentity->len; k++) {
					MapEntity_t *mapentity = &vector_at(maplvl->mapentity, MapEntity_t, k);

					if (mapentity->shape.num > 0)
						polygon_free(&mapentity->shape);
					
					vector_free (mapentity->entrances);
					vector_free (mapentity->exits);
				}

				vector_free (maplvl->mapentity);
			}

			vector_free (mapdrawing->maplvl);
		}
		vector_free (map_object->mapdrawing);

		mem_free(map_object);
	}
	
	return TRUE;
}

uint32_t
mapobject_get_num_lvl(MapObject_t* map_object)
{
	MapDrawing_t *mapdraw = &vector_at(map_object->mapdrawing, MapDrawing_t, 0);
	
	return mapdraw->maplvl->len;
}


bool_t 
mapobject_validate_floornum(MapObject_t* map_object, int32_t floor_num)
{
	MapDrawing_t *mapdraw = &vector_at(map_object->mapdrawing, MapDrawing_t, 0);
	uint32_t i;
	MapLvl_t *maplvl;

	for (i = 0; i < mapdraw->maplvl->len; i++) {
		maplvl = &vector_at(mapdraw->maplvl, MapLvl_t, i);

		if (floor_num == maplvl->lvl)
			return TRUE;
	}

	return FALSE;

}


vector2f_t
mapobject_get_mxy(const MapObject_t *map_object, double lat, double lon)
{
	vector2f_t ret_val;
	double g_vec[3];
	double m_vec[3];
	const double *Rg2m = map_object->Rg2m;

	g_vec[0] = lat; g_vec[1] = lon; g_vec[2] = 1.0;

	matmul("NN", 3, 1, 3, 1.0, Rg2m, g_vec, 0.0, m_vec);

	ret_val.X = (float)m_vec[0];
	ret_val.Y = (float)m_vec[1];

	return ret_val;
}


void
mapobject_get_latlon(const MapObject_t *map_object, double *lat, double *lon, const vector2f_t *mxy)
{
	double g_vec[3];
	double m_vec[3];
	const double *Rm2g = map_object->Rm2g;

	m_vec[0] = mxy->X;
	m_vec[1] = mxy->Y;
	m_vec[2] = 1.0;

	matmul("NN", 3, 1, 3, 1.0, Rm2g, m_vec, 0.0, g_vec);

	*lat = g_vec[0];
	*lon = g_vec[1];
}


MapErr_t 
mapobject_get_mapnode_id(const MapObject_t *map_object, uint32_t id, const int32_t lvl, MapNode_t **mapnode)
{
	uint32_t i, j;
	MapDrawing_t *mapdraw = &vector_at(map_object->mapdrawing, MapDrawing_t, 0);
	MapLvl_t *maplvl;
	MapNode_t *node_i;
	*mapnode = NULL;

	for (i = 0; i < mapdraw->maplvl->len; i++) {
		maplvl = &vector_at(mapdraw->maplvl, MapLvl_t, i);

		if (lvl == maplvl->lvl) {
			for (j = 0; j < maplvl->mapnodes->len; j++) {
				node_i = &vector_at(maplvl->mapnodes, MapNode_t, j);
				if (node_i->id == id) {
					*mapnode = node_i;
					return imm_success;
				}				
			}
		}
	}

	return imm_invalid_node_id;
}


MapErr_t
mapobject_get_mapentity(const MapObject_t *map_obj,
const vector2f_t *point,
const int32_t lvl,
MapEntity_t **mapenitity)
{
	return point_search(map_obj->map_tree, point, lvl, mapenitity);
}


MapErr_t
mapobject_get_mapentities_range_name(const MapObject_t *map_obj,
const vector2f_t *point,
float range,
const int32_t lvl,
char *name,
ptr_vector_t **mapentities)
{
	return point_range_name_search(map_obj->map_tree, point, range, lvl, name, mapentities);
}


MapErr_t 
mapobject_get_mapnode_range(const MapObject_t *map_obj,
	const vector2f_t *point,
	float range,
	const int32_t lvl,
	ptr_vector_t **mapnodes)
{
	return node_range_search(map_obj->map_tree, point, range, lvl, mapnodes);
}


MapEntity_t*
mapobject_get_nearest_mapentity(ptr_vector_t *in, const vector2f_t *pos)
{
	uint32_t i, min_idx = 0;
	float min_distance = 1000.0f, area;
	vector2f_t center;
	MapEntity_t *map_entity_i;

	for (i = 0; i < in->len; i++) {
		float distance_i;
		map_entity_i = (MapEntity_t *)ptr_vector_at(in, i);
		polygon_geometry(&map_entity_i->shape, &area, &center);
		distance_i = vector2f_distance(&center, pos);
		if (distance_i <= min_distance) {
			min_distance = distance_i;
			min_idx = i;
		}
	}

	map_entity_i = (MapEntity_t *)ptr_vector_at(in, min_idx);
	return map_entity_i;
}


MapNode_t*
mapobject_get_nearest_mapnode(ptr_vector_t *in, const vector2f_t *pos)
{
	uint32_t i, min_idx = 0;
	float min_distance = 1000.0f;
	MapNode_t *map_node_i = NULL;

	for (i = 0; i < in->len; i++) {
		float distance_i;
		map_node_i = (MapNode_t *)ptr_vector_at(in, i);
		distance_i = vector2f_distance(&map_node_i->map_pos, pos);
		if (distance_i <= min_distance) {
			min_distance = distance_i;
			min_idx = i;
		}
	}

	map_node_i = (MapNode_t *)ptr_vector_at(in, min_idx);
	return map_node_i;
}


uint32_t
mapobject_get_background_id(const MapObject_t *map_object, int32_t lvl)
{
	MapDrawing_t *mapdraw = &vector_at(map_object->mapdrawing, MapDrawing_t, 0);
	uint32_t i;
	MapLvl_t *maplvl;

	for (i = 0; i < mapdraw->maplvl->len; i++) {
		maplvl = &vector_at(mapdraw->maplvl, MapLvl_t, i);

		if (lvl == maplvl->lvl)
			return maplvl->bg_id;
	}

	return 0;
}


MapEntity_t*
mapobject_get_background_mapentity(const MapObject_t* map_object, int32_t lvl)
{
	MapDrawing_t *mapdraw = &vector_at(map_object->mapdrawing, MapDrawing_t, 0);
	MapLvl_t *maplvl;
	uint32_t i, j;
	MapEntity_t *map_entity = NULL;


	for (i = 0; i < mapdraw->maplvl->len; i++) {
		maplvl = &vector_at(mapdraw->maplvl, MapLvl_t, i);

		if (lvl == maplvl->lvl) {
			for (j = 0; j < maplvl->mapentity->len; j++) {
				map_entity = &vector_at(maplvl->mapentity, MapEntity_t, j);
				if (map_entity->type == imm_background)
					break;
			}
		}
	}

	return map_entity;
}


float  
mapobject_get_orientation(const MapObject_t* map_object)
{
	MapDrawing_t *draw = &vector_at(map_object->mapdrawing, MapDrawing_t, 0);
	return draw->angle;
}


bool_t
mapobject_is_edge_aligned(const MapObject_t* map_object, edge_t *edge)
{
	float slope = edge->slope;
	float map_angle = mapobject_get_orientation(map_object);

	float diff = angle_diff(slope, map_angle);

	if ((ABS(diff) <= 10.0f) || ((ABS(diff) >= 80.0f) && ABS(diff) <= 100.0f) || ABS(diff) >= 170.0f)
		return TRUE;
	else
		return FALSE;
}


static uint32_t 
binary_read (void *dest, uint32_t size, uint32_t num, const uint8_t **src)
{
	uint32_t read_bytes = size * num;
	memcpy (dest, *src, read_bytes);
	*src += read_bytes;
	return read_bytes;
}


static void
load_shape_from_bin_file (polygon_t *shape, const uint8_t **src)
{
	uint32_t vertex_num, i;
	walllist_t *tmp;
	vector2f_t vertex;

	binary_read (&vertex_num, sizeof (vertex_num), 1, src);

	polygon_init(shape);

	for (i = 0; i < vertex_num; i++) {
		tmp = (walllist_t *)mem_allocate(sizeof(walllist_t));	
		tmp->is_inner = FALSE;

		binary_read(&vertex.X, sizeof (float), 1, src);
		binary_read(&vertex.Y, sizeof (float), 1, src);

		tmp->wall.id = shape->num;
		tmp->wall.edge.start = vertex;	

		list_add_tail(&(tmp->wall.list), &shape->edge_list);	
		shape->num++;
	}

	polygon_close_path(shape);

	if (shape->num != vertex_num)
		return;
	

}


void
mapobject_reset_walls_listhead(MapObject_t *map_object)
{
	uint32_t i, j, k;
	for (i = 0; i < map_object->mapdrawing->len; i++){
		MapDrawing_t *mapdraw = &vector_at(map_object->mapdrawing, MapDrawing_t, i);
		for (j = 0; j < mapdraw->maplvl->len; j++) {
			MapLvl_t *maplvl = &vector_at(mapdraw->maplvl, MapLvl_t, j);
			for (k = 0; k < maplvl->mapentity->len; k++) {
				MapEntity_t *mapentity = &vector_at(maplvl->mapentity, MapEntity_t, k);
				mapentity->shape.edge_list.next->prev = &mapentity->shape.edge_list;
				mapentity->shape.edge_list.prev->next = &mapentity->shape.edge_list;
			}
		}
	}
	
}


static bool_t 
load_binary_mapfile (MapObject_t *map_object, const uint8_t *binary_array, uint32_t size)
{
	uint32_t i, j, k;	
	uint32_t mapdrawing_num;

	return_val_if_fail (size > 0, FALSE);

	memset (map_object, 0, sizeof (*map_object));

	binary_read (&map_object->id, sizeof (map_object->id), 1, &binary_array);
	binary_read (map_object->name, sizeof (map_object->name), 1, &binary_array);
	binary_read (&map_object->version, sizeof (map_object->version), 1, &binary_array);
	binary_read (&map_object->map_type, sizeof (map_object->map_type), 1, &binary_array);
	binary_read (&map_object->doors_available, sizeof (map_object->doors_available), 1, &binary_array);
	binary_read (&map_object->grid_available, sizeof (map_object->grid_available), 1, &binary_array);
	binary_read (map_object->min_max, sizeof (map_object->min_max), 1, &binary_array);

	// Map drawing;	
	binary_read (&mapdrawing_num, sizeof (mapdrawing_num), 1, &binary_array);	
	map_object->mapdrawing =  vector_new (sizeof (MapDrawing_t));

	for (i = 0; i < mapdrawing_num; i++) {
		MapDrawing_t mapdrawing;
		uint32_t mapdlvl_num;
		double trans_mat[6];

		memset(&mapdrawing, 0, sizeof (mapdrawing));
		binary_read (&mapdrawing.id, sizeof (mapdrawing.id), 1, &binary_array);
		binary_read (mapdrawing.name, sizeof (mapdrawing.name), 1, &binary_array);
		binary_read (&mapdrawing.root, sizeof (mapdrawing.root), 1, &binary_array);
		binary_read (&mapdrawing.angle, sizeof (mapdrawing.angle), 1,&binary_array);
        if (map_object->version > 0) {
            binary_read(&mapdrawing.aisle_length, sizeof(mapdrawing.aisle_length), 1, &binary_array);
            binary_read(&mapdrawing.aisle_width, sizeof(mapdrawing.aisle_width), 1, &binary_array);
        }
        else {
            mapdrawing.aisle_length = 20.0f;
            mapdrawing.aisle_width = 3.0f;
        }
		binary_read(trans_mat, sizeof (trans_mat), 1, &binary_array);

		// Initialize projection parameters
		map_object->Rm2g[0 + 0 * 3] = trans_mat[1];
		map_object->Rm2g[0 + 1 * 3] = trans_mat[3];
		map_object->Rm2g[0 + 2 * 3] = trans_mat[5];
		map_object->Rm2g[1 + 0 * 3] = trans_mat[0];
		map_object->Rm2g[1 + 1 * 3] = trans_mat[2];
		map_object->Rm2g[1 + 2 * 3] = trans_mat[4];
		map_object->Rm2g[2 + 0 * 3] = 0.0;
		map_object->Rm2g[2 + 1 * 3] = 0.0;
		map_object->Rm2g[2 + 2 * 3] = 1.0;

		matcpy(map_object->Rg2m, map_object->Rm2g, 3, 3);
		matinv(map_object->Rg2m, 3);

		// Map level		
		binary_read (&mapdlvl_num, sizeof(mapdlvl_num),1, &binary_array);

		mapdrawing.maplvl = vector_new (sizeof (MapLvl_t));
		for (j = 0; j < mapdlvl_num; j++) {
			MapLvl_t maplvl;
			uint32_t mapentity_num;
			uint32_t mapnode_num;

			memset(&maplvl, 0, sizeof (MapLvl_t));

			binary_read (&maplvl.id, sizeof (maplvl.id), 1, &binary_array);
			binary_read (&maplvl.lvl, sizeof (maplvl.lvl), 1, &binary_array);
			binary_read (&maplvl.bg_id, sizeof (maplvl.bg_id), 1, &binary_array);
			binary_read (&mapnode_num, sizeof (mapnode_num), 1, &binary_array);
			maplvl.mapnodes = vector_new(sizeof(MapNode_t));

			for (k = 0; k < mapnode_num; k++) {
				MapNode_t mapnode;
				uint32_t id_num, id_idx, node_id;
				binary_read(&mapnode.id, sizeof (mapnode.id), 1, &binary_array);
				binary_read(&mapnode.type, sizeof (mapnode.type), 1, &binary_array);
				binary_read(&mapnode.map_pos, sizeof (mapnode.map_pos), 1, &binary_array);
				binary_read(&id_num, sizeof (id_num), 1, &binary_array);
				mapnode.link_node_id = vector_new(sizeof(uint32_t));
				for (id_idx = 0; id_idx < id_num; id_idx++) {
					binary_read(&node_id, sizeof (node_id), 1, &binary_array);
					vector_append_element(mapnode.link_node_id, node_id);
				}
				vector_append_element(maplvl.mapnodes, mapnode);
			}

			binary_read (&mapentity_num, sizeof (mapentity_num),1, &binary_array);
			maplvl.mapentity = vector_new(sizeof (MapEntity_t));
			
			for (k = 0; k < mapentity_num; k++) {
				MapEntity_t mapentity;
				uint32_t gate_idx, gate_num;	

				memset(&mapentity, 0, sizeof (mapentity));

				binary_read (&mapentity.id, sizeof (mapentity.id), 1, &binary_array);
				binary_read (mapentity.name, sizeof (mapentity.name), 1, &binary_array);
				binary_read (&mapentity.type, sizeof (mapentity.type), 1, &binary_array);
				binary_read (mapentity.min_max, sizeof (mapentity.min_max), 1, &binary_array);
				binary_read (&mapentity.is_accessible, sizeof (mapentity.is_accessible), 1, &binary_array);
				binary_read (&mapentity.is_ignored, sizeof (mapentity.is_ignored), 1, &binary_array);
				
				load_shape_from_bin_file(&mapentity.shape, &binary_array);
				
				binary_read (&gate_num, sizeof (gate_num), 1, &binary_array);
				mapentity.entrances = vector_new (sizeof (Gate_t));
				for (gate_idx = 0; gate_idx < gate_num; gate_idx++) {
					Gate_t gate;
					binary_read (&gate, sizeof (Gate_t), 1, &binary_array);
					vector_append_element (mapentity.entrances, gate);
				}

				binary_read (&gate_num, sizeof (gate_num), 1, &binary_array);
				mapentity.exits = vector_new (sizeof (Gate_t));
				for (gate_idx = 0; gate_idx < gate_num; gate_idx++) {
					Gate_t gate;
					binary_read (&gate, sizeof (Gate_t), 1, &binary_array);
					vector_append_element (mapentity.exits, gate);
				}		

				if (mapentity.shape.num == 0)
					continue;

				vector_append_element(maplvl.mapentity, mapentity);
				
			}
			vector_append_element (mapdrawing.maplvl, maplvl);

			
		}
		vector_append_element (map_object->mapdrawing, mapdrawing);
	}

	mapobject_reset_walls_listhead(map_object);

	return TRUE;
}

