#ifndef _IMM_RTREE_H_
#define _IMM_RTREE_H_

#include "imm_types.h"
#include "imm_mapobject.h"

IMM_BEGIN_DECLS

typedef struct MapTree_Tag MapTree_t;

MapTree_t* rtrees_init(void);

bool_t rtrees_build(MapTree_t *map_tree, MapLvl_t *map_lvl);

MapErr_t point_search(const MapTree_t *map_tree,
	                  const vector2f_t *point, 
					  const int32_t lvl,
					  MapEntity_t **mapentity);

MapErr_t point_range_name_search(const MapTree_t *map_tree,
	                             const vector2f_t *point, 
							     float range, 
								 const int32_t lvl, 
								 char *name,
								 ptr_vector_t **mapentities);

MapErr_t node_range_search(const MapTree_t *map_tree,
	                       const vector2f_t *point,
	                       float range,
	                       const int32_t lvl,
	                       ptr_vector_t **mapnodes);

void rtrees_free(MapTree_t *map_tree);

void rtrees_printf(const MapTree_t *map_tree);

IMM_END_DECLS

#endif