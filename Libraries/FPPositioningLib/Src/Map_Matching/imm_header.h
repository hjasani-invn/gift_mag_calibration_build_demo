#include "imm_vector.h"
#include "imm_geoobject.h"
#include "imm_types.h"


#define NAME_STR_SIZE  (32)

#if !(defined (IMM_STMT_START) && defined (IMM_STMT_END))
#define IMM_STMT_START  do
#define IMM_STMT_END    while (0)
#endif

//struct list_head {
//	struct list_head *next, *prev;
//};

typedef enum {
	imm_background = 0,
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
} EntityType_t;

typedef enum {
	imm_tradeshow = 0,
	imm_office,
	imm_shoppingmall,
	imm_retailstore
} MapType_t;


typedef struct MapEntity_tag {
	int               id;                   // Map entity id
	EntityType_t      type;                 // Map entity Type
	char              name[NAME_STR_SIZE];  // Map entity Name
	float             min_max[4];           // Map entity Bounds
	polygon_t         shape;                // Pointer to the map entity shape
	bool              is_accessible;        // Is accessible
	bool              is_ignored;           // Is ignored map entity 
	vector_t          *entrances;
	vector_t          *exits;
} MapEntity_t;

typedef enum {
	intersect_node = 0,
	end_node,
	escalator_node,
	elevator_node
} NodeType_t;

typedef struct MapNode_tag {
	int               id;
	NodeType_t        type;
	vector2f_t        map_pos;
	vector_t          *link_node_id;
	ptr_vector_t      *link_node;
	ptr_vector_t      *extended_links;
	vector_t          *sin_incoming_angles;
	vector_t          *cos_incoming_angles;
	ptr_vector_t      *anchor_points_link_end_nodes;
	ptr_vector_t      *anchor_points_2d_array;
	uint32_t          extra_connected_incoming_angle;
	ptr_vector_t      *in_links;
	ptr_vector_t      *out_links;
} MapNode_t;

typedef struct MapLink_tag {
	int               id;
	MapNode_t         *start;
	MapNode_t         *end;
	float             slope;
	float             distance;
} MapLink_t;

typedef struct Gate_tag {
	bool              position_available;   // is entrance information available
	vector2f_t        location;             // entrance location
	edge_t            *edge;
	bool              orientation_availble; // is orientation information available
	float             orientation;          // entrance orientation
} Gate_t;

typedef struct walllist_tag {
	edgelist_t        wall;
	bool              is_inner;
} walllist_t;

typedef struct MapLvl_tag {
	int               id;                   // Level id
	int               lvl;                  // Level number
	vector_t          *mapentity;           // Pointer to map entities
	vector_t          *mapnodes;
	vector_t          *maplinks;
	int               bg_id;                // Background id
	vector_t          *entrances;
} MapLvl_t;

typedef struct MapDrawing_tag {
	int               id;                   // Map drawing id
	char              name[NAME_STR_SIZE];  // Map drawing name
	bool              root;                 // Is root drawing
	float             angle;                // Map drawing angle
	vector_t          *maplvl;              // Pointer to map levels
	float             aisle_length;
	float             aisle_width;
} MapDrawing_t;

typedef struct MapTree_Tag MapTree_t;
typedef struct {
	int               id;                   // Venue map id
	char              name[NAME_STR_SIZE];  // Venue name
	int               version;              // Venue version
	MapType_t         map_type;             // map type
	bool              doors_available;      // Availability of doors information
	bool              grid_available;       // Grid Map availability
	bool              entity_available;     // Geometric map info
	double            min_max[4];           // Venue boundary
	vector_t          *mapdrawing;	        // Pointer to map drawings
	double            Rg2m[9];              // Projection Matrix
	double            Rm2g[9];              // Inverse Projection Matrix
	MapTree_t         *map_tree;            // R_tree structure
}MapObject_t;


MapObject_t* mapobject_init(const uint8_t *data_array, size_t size);