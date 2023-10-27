#ifndef _IMM_MAPOBJECT_H_
#define _IMM_MAPOBJECT_H_


#include "imm_types.h"
#include "imm_vector.h"
#include "imm_geoobject.h"

IMM_BEGIN_DECLS

#define OUTSIDE_ID  (0xFFFFFFF0)
#define NAME_STR_SIZE  (32)

typedef enum {
	imm_background,
	imm_entrance_area,
	imm_unit,
	imm_booth,
	imm_room,
	imm_lvl_change,
	imm_wall,
	imm_door,
	imm_service,
	imm_section,
	imm_parking_lot,
	imm_parking_structure,
	imm_entrance,
	imm_inaccessible,
	imm_road,
	imm_hallway,
	imm_building,
	imm_obstruction,
	imm_chair,
	imm_step,
	imm_table
}EntityType_t;


typedef enum {
	intersect_node,
	end_node,	
	escalator_node,
	elevator_node
} NodeType_t;


typedef enum {
	imm_tradeshow,
	imm_office,
	imm_shoppingmall,
	imm_retailstore
}MapType_t;


typedef enum  {
	imm_success = 0,
	imm_invalid_lvl,
	imm_invalid_pos,
	imm_invalid_range,
	imm_invalid_node_id,
	imm_invalid_node_type
}MapErr_t;


typedef struct {
	bool_t           position_available;   // is entrance information available
	vector2f_t       location;             // entrance location
	edge_t           *edge;
	bool_t           orientation_availble; // is orientation information available
	float            orientation;          // entrance orientation
}Gate_t;


typedef struct {
	edgelist_t        wall;
	bool_t            is_inner;
} walllist_t;


typedef struct {
	uint32_t          id;                   // Map entity id
	EntityType_t      type;                 // Map entity Type
	char              name[NAME_STR_SIZE];  // Map entity Name
	float             min_max[4];           // Map entity Bounds
	polygon_t         shape;                // Pointer to the map entity shape
	bool_t            is_accessible;        // Is accessible
	bool_t            is_ignored;           // Is ignored map entity 
	vector_t          *entrances;
	vector_t          *exits;
} MapEntity_t;


typedef struct {
	uint32_t id;
	NodeType_t type;
	vector2f_t map_pos;
	vector_t *link_node_id;
	ptr_vector_t *link_node;
	ptr_vector_t *extended_links;
	vector_t *incoming_angles;
	vector_t *incoming_angles_number;
	ptr_vector_t *anchor_points_link_end_nodes;
	ptr_vector_t *anchor_points_IDs;
	uint32_t extra_connected_incoming_angle;
} MapNode_t;


typedef struct {
	MapNode_t *start;
	MapNode_t *end;
	float slope;
	float distance;
	uint32_t end_node_type;
	vector_t *anchor_point_IDs;
	bool_t by_passing;
	uint32_t n_nodes_on_the_way;
} MapLink_t;


typedef struct {
	uint32_t          id;                   // Level id
	int32_t           lvl;                  // Level number
	vector_t          *mapentity;           // Pointer to map entities
	vector_t          *mapnodes;
	vector_t          *maplinks;
	uint32_t          bg_id;                // Background id
}MapLvl_t;


//typedef struct MapObject_Tag MapObject_t;
typedef struct {
	uint32_t          id;                   // Map drawing id
	char              name[NAME_STR_SIZE];  // Map drawing name
	bool_t            root;                 // Is root drawing
	float             angle;                // Map drawing angle
    float             aisle_length;
    float             aisle_width;
	vector_t          *maplvl;              // Pointer to map levels
}MapDrawing_t;


typedef struct MapTree_Tag MapTree_t;
typedef struct  {
	uint32_t          id;                   // Venue map id
	char              name[NAME_STR_SIZE];  // Venue name
	uint32_t          version;              // Venue version
	MapType_t         map_type;             // map type
	bool_t            doors_available;      // Availability of doors information
	bool_t            grid_available;       // Grid Map availability
	double            min_max[4];           // Venue boundary
	vector_t          *mapdrawing;	        // Pointer to map drawings
	double            Rg2m[9];              // Projection Matrix
	double            Rm2g[9];              // Inverse Projection Matrix
	MapTree_t         *map_tree;            // R_tree structure
}MapObject_t;


MapObject_t* mapobject_init(const uint8_t *data_array, uint32_t size);

bool_t mapobject_process(MapObject_t* map_object);

bool_t mapobject_free(MapObject_t* map_object);

bool_t mapobject_validate_floornum(MapObject_t* map_object, int32_t floor_num);

vector2f_t mapobject_get_mxy(const MapObject_t* map_object, double lat, double lon);

void mapobject_get_latlon(const MapObject_t* map_object, double *lat, double *lon, const vector2f_t *mxy);

MapErr_t mapobject_get_mapentity (const MapObject_t *map_obj,
	                              const vector2f_t *point, 
						          const int32_t lvl, 
						          MapEntity_t **mapenitity);

MapErr_t mapobject_get_mapnode_id(const MapObject_t *map_obj, uint32_t id, const int32_t lvl, MapNode_t **mapnode);

MapErr_t mapobject_get_mapentities_range_name(const MapObject_t *map_obj,
	                                          const vector2f_t *point,
	                                          float range,
	                                          const int32_t lvl,
	                                          char *name,
								              ptr_vector_t **mapentities);

MapErr_t mapobject_get_mapnode_range(const MapObject_t *map_obj,
	                                 const vector2f_t *point,
	                                 float range,
	                                 const int32_t lvl,
	                                 ptr_vector_t **mapnodes);

MapErr_t mapobject_get_mapnode_type(const MapObject_t *map_obj,
                                    NodeType_t type,
	                                const int32_t lvl,
	                                ptr_vector_t **mapnodes);

MapErr_t mapobject_get_mapnode_type_neu(const MapObject_t *map_obj,
	NodeType_t type,
	const int32_t lvl,
	ptr_vector_t *mapnodes);

MapErr_t mapobject_get_mapnode_ellipse(const MapObject_t *map_obj,
	                                   const polygon_t *ellipse,
	                                   const int32_t lvl,
	                                   ptr_vector_t **mapnodes);

MapEntity_t* mapobject_get_nearest_mapentity(ptr_vector_t *in, const vector2f_t *pos);

MapErr_t mapobject_get_range_nearest_mapnode(const MapObject_t *map_obj,
											  const vector2f_t *point,
											  float range,
											  const int32_t lvl,
											  MapNode_t **mapnodes);

MapNode_t* mapobject_get_nearest_mapnode(ptr_vector_t *in, const vector2f_t *pos);

uint32_t mapobject_get_background_id(const MapObject_t *map_object, int32_t lvl);

MapEntity_t* mapobject_get_background_mapentity(const MapObject_t* map_object, int32_t lvl);

bool_t mapobject_is_edge_aligned(const MapObject_t* map_object, edge_t *edge);

bool_t mapobject_is_link_aligned(const MapObject_t* map_object, MapLink_t *link);

bool_t mapobject_is_link_connected(const MapLink_t *a, MapLink_t *b);

void mapobject_set_link_slope(MapLink_t *maplink);

void mapobject_set_link_distance(MapLink_t *maplink);

float  mapobject_get_orientation(const MapObject_t* map_object);

float vector2f_link_distance(const vector2f_t *p, const MapLink_t *link);

void vector2f_link_project(vector2f_t *p, const MapLink_t *link);

bool_t mapobject_link_equal(const MapLink_t *a, const MapLink_t *b);

void mapobject_reset_walls_listhead(MapObject_t *map_object);

uint32_t
mapobject_get_maplink_num(MapObject_t *map_object);


uint32_t
mapobject_get_num_lvl(MapObject_t *map_object);


float mapobject_get_aisle_length(MapObject_t *map_object); 

float mapobject_get_aisle_width(MapObject_t *map_object);

IMM_END_DECLS

#endif
