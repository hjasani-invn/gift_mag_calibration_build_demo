#include "imm_header.h"
#include "imm_math.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#ifdef _DEBUG
#include <stdio.h>
//#include <string.h>
//#include "imm_list.h"
//#include "imm_math.h"
#endif // DEBUG      
 
static size_t binary_read(void *dest, size_t size, int num, const uint8_t **src);
static void load_shape_from_bin_file(polygon_t *shape, const uint8_t **src);
static bool load_binary_mapfile(MapObject_t* map_object, const uint8_t *binary_array, size_t size);
static void mapobject_reset_walls_listhead(MapObject_t *map_object);


void* mem_allocate(size_t mem_size)
{
	void *mem;
	mem = malloc(mem_size);

	if (mem) {
		return mem;
	}
	else {
#ifdef _DEBUG
		printf("Error allocate memory.\n");
#endif // DEBUG      
		exit(-1);
	}
}

void mem_free(void* mem)
{
	return_if_fail(mem);
	free(mem);
}

static size_t binary_read(void *dest, size_t size, int num, const uint8_t **src)
{
	size_t read_bytes = size * num;
	memcpy(dest, *src, read_bytes);
	*src += read_bytes;
	return read_bytes;
}

static void mapobject_reset_walls_listhead(MapObject_t *map_object)
{
	int i, j, k;
	for (i = 0; i < map_object->mapdrawing->len; i++) {
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

static void load_shape_from_bin_file(polygon_t *shape, const uint8_t **src)
{
	int vertex_num, i;
	walllist_t *tmp;
	vector2f_t vertex;

	binary_read(&vertex_num, sizeof(vertex_num), 1, src);

	polygon_init(shape);

	for (i = 0; i < vertex_num; i++) {
		tmp = (walllist_t *)mem_allocate(sizeof(walllist_t));
		tmp->is_inner = false;

		binary_read(&vertex.X, sizeof(float), 1, src);
		binary_read(&vertex.Y, sizeof(float), 1, src);

		tmp->wall.id = shape->num;
		tmp->wall.edge.start = vertex;

		list_add_tail(&(tmp->wall.list), &shape->edge_list);
		shape->num++;
	}

	polygon_close_path(shape);

	if (shape->num != vertex_num)
		return;	
}

static void load_gate_from_bin_file(Gate_t *gate, const uint8_t **src)
{
	memset(gate, 0, sizeof(Gate_t));

	binary_read(&gate->position_available, sizeof(gate->position_available), 1, src);
	*src += 3;
	binary_read(&gate->location, sizeof(gate->location), 1, src);
	*src += 4;
	binary_read(&gate->orientation_availble, sizeof(gate->orientation_availble), 1, src);
	*src += 3;
	binary_read(&gate->orientation, sizeof(gate->orientation), 1, src);
}

static bool load_binary_mapfile(MapObject_t *map_object, const uint8_t *binary_array, size_t size)
{
	uint32_t i, j, k;
	uint32_t mapdrawing_num;

	return_val_if_fail(size > 0, false);

	memset(map_object, 0, sizeof(*map_object));

	binary_read(&map_object->id, sizeof(map_object->id), 1, &binary_array);
	binary_read(map_object->name, sizeof(map_object->name), 1, &binary_array);
	binary_read(&map_object->version, sizeof(map_object->version), 1, &binary_array);
	binary_read(&map_object->map_type, sizeof(map_object->map_type), 1, &binary_array);
	binary_read(&map_object->doors_available, sizeof(map_object->doors_available), 1, &binary_array);
	binary_read(&map_object->grid_available, sizeof(map_object->grid_available), 1, &binary_array);
	binary_read(map_object->min_max, sizeof(map_object->min_max), 1, &binary_array);

	// Map drawing;
	binary_read(&mapdrawing_num, sizeof(mapdrawing_num), 1, &binary_array);
	map_object->mapdrawing = vector_new(sizeof(MapDrawing_t));

	for (i = 0; i < mapdrawing_num; i++) 
	{
		MapDrawing_t mapdrawing;
		uint32_t mapdlvl_num;
		double trans_mat[6];

		memset(&mapdrawing, 0, sizeof(mapdrawing));
		binary_read(&mapdrawing.id, sizeof(mapdrawing.id), 1, &binary_array);
		binary_read(mapdrawing.name, sizeof(mapdrawing.name), 1, &binary_array);
		binary_read(&mapdrawing.root, sizeof(mapdrawing.root), 1, &binary_array);
		binary_read(&mapdrawing.angle, sizeof(mapdrawing.angle), 1, &binary_array);

		if (map_object->version > 0) 
		{
			binary_read(&mapdrawing.aisle_length, sizeof(mapdrawing.aisle_length), 1, &binary_array);
			binary_read(&mapdrawing.aisle_width, sizeof(mapdrawing.aisle_width), 1, &binary_array);
		}
		else 
		{
			mapdrawing.aisle_length = 20.0f;
			mapdrawing.aisle_width = 3.0f;
		}

		binary_read(trans_mat, sizeof(trans_mat), 1, &binary_array);

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
		binary_read(&mapdlvl_num, sizeof(mapdlvl_num), 1, &binary_array);

		mapdrawing.maplvl = vector_new(sizeof(MapLvl_t));
		for (j = 0; j < mapdlvl_num; j++) 
		{
			MapLvl_t maplvl;
			uint32_t mapentity_num;
			uint32_t mapnode_num;

			memset(&maplvl, 0, sizeof(MapLvl_t));

			binary_read(&maplvl.id, sizeof(maplvl.id), 1, &binary_array);
			binary_read(&maplvl.lvl, sizeof(maplvl.lvl), 1, &binary_array);
			binary_read(&maplvl.bg_id, sizeof(maplvl.bg_id), 1, &binary_array);
			binary_read(&mapnode_num, sizeof(mapnode_num), 1, &binary_array);
			maplvl.mapnodes = vector_new(sizeof(MapNode_t));
			maplvl.maplinks = vector_new(sizeof(MapLink_t));

			for (k = 0; k < mapnode_num; k++) 
			{
				MapNode_t mapnode = { 0 };
				uint32_t id_num = 0, id_idx = 0, node_id = 0;
				binary_read(&mapnode.id, sizeof(mapnode.id), 1, &binary_array);
				binary_read(&mapnode.type, sizeof(mapnode.type), 1, &binary_array);
				binary_read(&mapnode.map_pos, sizeof(mapnode.map_pos), 1, &binary_array);
				binary_read(&id_num, sizeof(id_num), 1, &binary_array);

				mapnode.link_node_id = vector_new(sizeof(int));
				mapnode.link_node = ptr_vector_new();
				mapnode.extended_links = ptr_vector_new();
				mapnode.sin_incoming_angles = vector_new(sizeof(float));
				mapnode.cos_incoming_angles = vector_new(sizeof(float));
				mapnode.anchor_points_link_end_nodes = ptr_vector_new();
				mapnode.anchor_points_2d_array = ptr_vector_new();
				mapnode.extra_connected_incoming_angle = 1000;
				mapnode.in_links = ptr_vector_new();
				mapnode.out_links = ptr_vector_new();

				for (id_idx = 0; id_idx < id_num; id_idx++) 
				{
					binary_read(&node_id, sizeof(node_id), 1, &binary_array);
					vector_append_element(mapnode.link_node_id, node_id);
				}
				vector_append_element(maplvl.mapnodes, mapnode);
			}

			binary_read(&mapentity_num, sizeof(mapentity_num), 1, &binary_array);
			maplvl.mapentity = vector_new(sizeof(MapEntity_t));

			for (k = 0; k < mapentity_num; k++) 
			{
				MapEntity_t mapentity;
				uint32_t gate_idx, gate_num;

				memset(&mapentity, 0, sizeof(mapentity));

				binary_read(&mapentity.id, sizeof(mapentity.id), 1, &binary_array);
				binary_read(mapentity.name, sizeof(mapentity.name), 1, &binary_array);
				binary_read(&mapentity.type, sizeof(mapentity.type), 1, &binary_array);
				binary_read(mapentity.min_max, sizeof(mapentity.min_max), 1, &binary_array);
				binary_read(&mapentity.is_accessible, sizeof(mapentity.is_accessible), 1, &binary_array);
				binary_read(&mapentity.is_ignored, sizeof(mapentity.is_ignored), 1, &binary_array);

				load_shape_from_bin_file(&mapentity.shape, &binary_array);

				binary_read(&gate_num, sizeof(gate_num), 1, &binary_array);
				mapentity.entrances = vector_new(sizeof(Gate_t));
				for (gate_idx = 0; gate_idx < gate_num; gate_idx++) 
				{
					Gate_t gate;
					//binary_read (&gate, sizeof (Gate_t), 1, &binary_array);

					load_gate_from_bin_file(&gate, &binary_array);
					vector_append_element(mapentity.entrances, gate);
				}

				binary_read(&gate_num, sizeof(gate_num), 1, &binary_array);
				mapentity.exits = vector_new(sizeof(Gate_t));
				for (gate_idx = 0; gate_idx < gate_num; gate_idx++) 
				{
					Gate_t gate;
					//binary_read (&gate, sizeof (Gate_t), 1, &binary_array);

					load_gate_from_bin_file(&gate, &binary_array);
					vector_append_element(mapentity.exits, gate);
				}

				if (mapentity.shape.num == 0)
					continue;

				vector_append_element(maplvl.mapentity, mapentity);

			}

			if (map_object->version > 1) 
			{
				uint32_t gate_idx, gate_num;
				maplvl.entrances = vector_new(sizeof(Gate_t));
				binary_read(&gate_num, sizeof(gate_num), 1, &binary_array);
				for (gate_idx = 0; gate_idx < gate_num; gate_idx++) 
				{
					Gate_t gate;
					//binary_read(&gate, sizeof(Gate_t), 1, &binary_array);

					load_gate_from_bin_file(&gate, &binary_array);
					vector_append_element(maplvl.entrances, gate);
				}
			}

			vector_append_element(mapdrawing.maplvl, maplvl);


		}
		vector_append_element(map_object->mapdrawing, mapdrawing);
	}

	mapobject_reset_walls_listhead(map_object);

	return true;
}


MapObject_t* mapobject_init(const uint8_t *data_array, size_t size)
{
	MapObject_t *mapobject = (MapObject_t *)mem_allocate(sizeof(MapObject_t));

	return_val_if_fail(mapobject != NULL, NULL);

	memset(mapobject, 0, sizeof(MapObject_t));

	if (load_binary_mapfile(mapobject, data_array, size)) {
		return mapobject;
	}
	else {
		mem_free(mapobject);
		return NULL;
	}
}