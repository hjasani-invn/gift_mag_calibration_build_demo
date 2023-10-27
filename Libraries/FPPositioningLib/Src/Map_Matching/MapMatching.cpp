#include "MapMatching.hpp"
#include <algorithm>
#include <cstring> 
#include <string>
#include <math.h>
#include <cmath>
#if MM_DEBUG_OUT
#include <iostream>
#include <fstream>
#endif

MapMatching::MapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes, double max_X, double max_Y, int16_t floor_shift, bool floor_zero_enable, const GeoLocConverter2D &converter)
{
    is_valid = true;

    mm_rng = new (std::nothrow) pf_random::rng32;

    if (nullptr == mm_rng)
    {
        is_valid = false;
        return;
    }

	mm_rng->seed(2, 8, 16, 128); // setting seed to replicate results with each run

	mm_cell_size = 0.1;
	mm_floor_shift = floor_shift;
	mm_floor_zero_enable = floor_zero_enable;
	
	map_cell_count_X = (int)ceil((max_X - 0) / mm_cell_size);
	map_cell_count_Y = (int)ceil((max_Y - 0) / mm_cell_size);

	mMinX = 0;
	mMinY = 0;
	mMaxX = max_X;
	mMaxY = max_Y;

	mMinFloor = 0;
	mMaxFloor = 0;

	for (int i = 0; i < MAPMATCHING_MAXPOLYGON; ++i)
	{
		mapPolygonXCoords[i] = (int *)malloc(map_cell_count_Y * sizeof(int));
		_ASSERT(mapPolygonXCoords[i]);
	}

	MapObject_t *map_object = mapobject_init(pMap, mapFileSizeInBytes);

	if (map_object == NULL)
	{
		is_valid = false;
	}

	one_floor_byte_size = map_cell_count_X * map_cell_count_Y;

	mpLineInternalCoords = (int(*)[2])malloc(std::max(map_cell_count_X + 2, map_cell_count_Y + 2) * sizeof(int) * 2);

	mpScannedLine = (uint8_t*)malloc(std::max(map_cell_count_X + 1, map_cell_count_Y + 1) * sizeof(uint8_t));

	MapDrawing_t *map_drawing;
	MapLvl_t *map_lvl;
	MapEntity_t *map_entity;
	vector2f_t *Pi, *Pi1;

	// creating bytemap for each floor
	// parsing map_object to get all floors from it first
	for (int i = 0; i < map_object->mapdrawing->len; i++)
	{
		map_drawing = &vector_at(map_object->mapdrawing, MapDrawing_t, i);

		for (int j = 0; j < map_drawing->maplvl->len; j++)
		{
			map_lvl = &vector_at(map_drawing->maplvl, MapLvl_t, j);
			// convert map_lvl->lvl to logical floor number
			int current_logical_floor = real_floor_to_logical_floor(map_lvl->lvl);

			uint8_t* p_bytemap = (uint8_t*)calloc(one_floor_byte_size, sizeof(uint8_t)); // filling each bytemap with zeros as default value

			//for (int k = 0; k < map_cell_count_Y; ++k)
			//{
			//	fill_horizontal(p_bytemap, 0, k, map_cell_count_X, 255); // 255 is fully accessible
			//}

			MM_Bytemaps.insert(std::pair<int8_t, uint8_t*>(current_logical_floor, p_bytemap));
		}
	}

	for (int k = 0; k < map_object->mapdrawing->len; k++)
	{
		map_drawing = &vector_at(map_object->mapdrawing, MapDrawing_t, k);

		for (int j = 0; j < map_drawing->maplvl->len; j++)
		{
			map_lvl = &vector_at(map_drawing->maplvl, MapLvl_t, j);
			int geo_num = map_lvl->mapentity->len;

			int current_logical_floor = real_floor_to_logical_floor(map_lvl->lvl);
			uint8_t* p_current_bytemap = NULL;
			bool success = get_bytemap_from_floor_number(current_logical_floor, p_current_bytemap); // bytemap to be used while iterating through objects on this floor

			for (int i = 0; i < geo_num; i++)
			{
				map_entity = &vector_at(map_lvl->mapentity, MapEntity_t, i);
				// map_entity type check
				EntityType_t type = map_entity->type;

				if (type == imm_background)
				{
					// filling background polygon with "available" cells
					int polygon_edges[MAPMATCHING_MAXPOLYGON][2];
					int edge_number = 0;
					edgelist_t *edge = list_entry(&map_entity->shape.edge_list.next, edgelist_t, list);

					list_for_each_entry(edge, edgelist_t, &map_entity->shape.edge_list, list)
					{
						Pi = &edge->edge.start;
						Pi1 = &edge->edge.end;

						// converting polygon edges to lat-lon

						double g_vec[3];
						double m_vec[3];

						const double *Rm2g = map_object->Rm2g;

						m_vec[0] = Pi->X;
						m_vec[1] = Pi->Y;
						m_vec[2] = 1.0;

						matmul("NN", 3, 1, 3, 1.0, Rm2g, m_vec, 0.0, g_vec);

						// convert to FPL coordinate frame
						// getting XY in FPPE frame
						double x, y;
						converter.Geo2Local(g_vec[0], g_vec[1], &x, &y);

						int cell_x, cell_y;
						user_to_bytemap_coords(x, y, cell_x, cell_y);
						
						bool coords_ok = check_bytemap_coordinates_validity(cell_x, cell_y);
						if (!coords_ok)
						{
							clear_memory();
							is_valid = false;
							return;
						}

						polygon_edges[edge_number][0] = cell_x;
						polygon_edges[edge_number][1] = cell_y;

						++edge_number;
					}

					set_polygon(p_current_bytemap, polygon_edges, edge_number, 255);
					break;
				}
			}

			// drawing obstacles over background
			for (int i = 0; i < geo_num; i++)
			{
				map_entity = &vector_at(map_lvl->mapentity, MapEntity_t, i);

				// map_entity type check
				// WARNING: currently it is just a very simple check, good enough to draw obstacles, but not rooms with doors
				bool accessible = map_entity->is_accessible;

				EntityType_t type = map_entity->type;
				char* ent_name = map_entity->name;
				std::string s_ent_name(ent_name);

				int polygon_edges[MAPMATCHING_MAXPOLYGON][2];
				int edge_number = 0;
				edgelist_t *edge = list_entry(&map_entity->shape.edge_list.next, edgelist_t, list);

				list_for_each_entry(edge, edgelist_t, &map_entity->shape.edge_list, list)
				{
					Pi = &edge->edge.start;
					Pi1 = &edge->edge.end;

					// converting polygon edges to lat-lon

					double g_vec[3];
					double m_vec[3];

					const double *Rm2g = map_object->Rm2g;

					m_vec[0] = Pi->X;
					m_vec[1] = Pi->Y;
					m_vec[2] = 1.0;

					matmul("NN", 3, 1, 3, 1.0, Rm2g, m_vec, 0.0, g_vec);

					// convert to FPL coordinate frame
					// getting XY in FPPE frame
					double x, y;
					converter.Geo2Local(g_vec[0], g_vec[1], &x, &y);

					int cell_x, cell_y;
					user_to_bytemap_coords(x, y, cell_x, cell_y);

					bool coords_ok = check_bytemap_coordinates_validity(cell_x, cell_y);
					if (!coords_ok)
					{
						clear_memory();
						is_valid = false;
						return;
					}

					polygon_edges[edge_number][0] = cell_x;
					polygon_edges[edge_number][1] = cell_y;

					++edge_number;
				}

				//if ((type != imm_background) && (type != imm_lvl_change) && (s_ent_name != "Escalator") && (s_ent_name != "Elevator")) // Northland case
				if (!accessible)
				{
					set_polygon(p_current_bytemap, polygon_edges, edge_number, 125);
				}
			}

			// drawing walls of rooms
			for (int i = 0; i < geo_num; i++)
			{
				map_entity = &vector_at(map_lvl->mapentity, MapEntity_t, i);

				// map_entity type check
				// WARNING: currently it is just a very simple check, good enough to draw obstacles, but not rooms with doors
				bool accessible = map_entity->is_accessible;

				EntityType_t type = map_entity->type;
				char* ent_name = map_entity->name;
				std::string s_ent_name(ent_name);

				int polygon_edges[MAPMATCHING_MAXPOLYGON][2];
				int edge_number = 0;
				edgelist_t *edge = list_entry(&map_entity->shape.edge_list.next, edgelist_t, list);

				list_for_each_entry(edge, edgelist_t, &map_entity->shape.edge_list, list)
				{
					Pi = &edge->edge.start;
					Pi1 = &edge->edge.end;

					// converting polygon edges to lat-lon

					double g_vec[3];
					double m_vec[3];

					const double *Rm2g = map_object->Rm2g;

					m_vec[0] = Pi->X;
					m_vec[1] = Pi->Y;
					m_vec[2] = 1.0;

					matmul("NN", 3, 1, 3, 1.0, Rm2g, m_vec, 0.0, g_vec);

					// convert to FPL coordinate frame
					// getting XY in FPPE frame
					double x, y;
					converter.Geo2Local(g_vec[0], g_vec[1], &x, &y);

					int cell_x, cell_y;
					user_to_bytemap_coords(x, y, cell_x, cell_y);

					bool coords_ok = check_bytemap_coordinates_validity(cell_x, cell_y);
					if (!coords_ok)
					{
						clear_memory();
						is_valid = false;
						return;
					}

					polygon_edges[edge_number][0] = cell_x;
					polygon_edges[edge_number][1] = cell_y;

					++edge_number;
				}

				if ((type == imm_room) || (type == imm_lvl_change))
				{
					// draw walls of the room
					set_polygon_borders(p_current_bytemap, polygon_edges, edge_number, 0);
				}
			}
			
			// drawing doors
			for (int i = 0; i < geo_num; i++)
			{
				map_entity = &vector_at(map_lvl->mapentity, MapEntity_t, i);

				// map_entity type check
				// WARNING: currently it is just a very simple check, good enough to draw obstacles, but not rooms with doors
				bool accessible = map_entity->is_accessible;

				EntityType_t type = map_entity->type;
				char* ent_name = map_entity->name;
				std::string s_ent_name(ent_name);

				int polygon_edges[MAPMATCHING_MAXPOLYGON][2];
				int edge_number = 0;
				edgelist_t *edge = list_entry(&map_entity->shape.edge_list.next, edgelist_t, list);

				list_for_each_entry(edge, edgelist_t, &map_entity->shape.edge_list, list)
				{
					Pi = &edge->edge.start;
					Pi1 = &edge->edge.end;

					// converting polygon edges to lat-lon

					double g_vec[3];
					double m_vec[3];

					const double *Rm2g = map_object->Rm2g;

					m_vec[0] = Pi->X;
					m_vec[1] = Pi->Y;
					m_vec[2] = 1.0;

					matmul("NN", 3, 1, 3, 1.0, Rm2g, m_vec, 0.0, g_vec);

					// convert to FPL coordinate frame
					// getting XY in FPPE frame
					double x, y;
					converter.Geo2Local(g_vec[0], g_vec[1], &x, &y);

					int cell_x, cell_y;
					user_to_bytemap_coords(x, y, cell_x, cell_y);
					bool coords_ok = check_bytemap_coordinates_validity(cell_x, cell_y);
					if (!coords_ok)
					{
						clear_memory();
						is_valid = false;
						return;
					}

					polygon_edges[edge_number][0] = cell_x;
					polygon_edges[edge_number][1] = cell_y;

					++edge_number;
				}

				if (type == imm_door)
				{
					// draw doors
					set_polygon_borders(p_current_bytemap, polygon_edges, edge_number, 255);
					set_polygon(p_current_bytemap, polygon_edges, edge_number, 255);
				}
			}
		}
	}
	
	// files for debugging
#if MM_DEBUG_OUT
	std::string image_file_name = "dbg_map_floor_";

	std::map<int8_t, uint8_t*>::iterator it;
	for (it = MM_Bytemaps.begin(); it != MM_Bytemaps.end(); ++it)
	{
		int floor = it->first;

		std::string output_image_name = image_file_name + std::to_string(floor) + ".bmp";
		debug_store_bmp(it->second, output_image_name);
	}

	std::string text_file_name = "dbg_map_floor_";

	for (it = MM_Bytemaps.begin(); it != MM_Bytemaps.end(); ++it)
	{
		int floor = it->first;
		std::string output_text_file_name = text_file_name + std::to_string(floor) + ".csv";
		debug_store_txt(it->second, output_text_file_name);
	}
#endif

}

void MapMatching::clear_memory()
{
	if (mpScannedLine != 0)
		free(mpScannedLine);
	if (mpLineInternalCoords != 0)
		free(mpLineInternalCoords);

	for (int i = 0; i < MAPMATCHING_MAXPOLYGON; ++i)
		free(mapPolygonXCoords[i]);

	std::map<int8_t, uint8_t*>::iterator it;
	for (it = MM_Bytemaps.begin(); it != MM_Bytemaps.end(); ++it)
	{
		free(it->second);
	}

	delete mm_rng;
}

MapMatching::~MapMatching()
{
	clear_memory();
}

bool MapMatching::get_validity()
{
	return is_valid;
}

int MapMatching::real_floor_to_logical_floor(int real_floor)
{
	int logical_floor = real_floor - mm_floor_shift;
	if ((!mm_floor_zero_enable) && (mm_floor_shift < 0))
	{
		logical_floor -= (real_floor > 0) ? 1 : 0;
	}

	return logical_floor;
}

bool MapMatching::get_bytemap_from_floor_number(int logical_floor, uint8_t* &pBytemap)
{
	bool success = false;
	// checking that bytemap exists
	if (MM_Bytemaps.find(logical_floor) != MM_Bytemaps.end())
	{
		pBytemap = MM_Bytemaps[logical_floor];
		success = true;
	}
		
	return success;
}

bool MapMatching::check_bytemap_coordinates_validity(int cell_x, int cell_y)
{
	if ((cell_x < 0) || (cell_y < 0) || (cell_x >= map_cell_count_X) || (cell_y >= map_cell_count_Y))
		return false;

	return true;
}

uint8_t MapMatching::get_cell_value(uint8_t* pBytemap, int cell_x, int cell_y)
{
	if ((cell_x < 0) || (cell_y < 0) || (cell_x >= map_cell_count_X) || (cell_y >= map_cell_count_Y))
		return (uint8_t)0;
	
	return pBytemap[cell_y * map_cell_count_X + cell_x];
}

void MapMatching::set_cell_value(uint8_t* pBytemap, int cell_x, int cell_y, uint8_t cell_value)
{
	pBytemap[cell_y * map_cell_count_X + cell_x] = cell_value;
}

void MapMatching::user_to_bytemap_coords(double x, double y, int &cell_x, int &cell_y)
{
	cell_x = (int)floor(0.5 + x / mm_cell_size);
	cell_y = (int)floor(0.5 + y / mm_cell_size);
}

void MapMatching::bytemap_to_user_coords(int cell_x, int cell_y, double &UserX, double &UserY)
{
	UserX = cell_x * mm_cell_size + mm_cell_size/2;
	UserY = cell_y * mm_cell_size + mm_cell_size/2;
}

void MapMatching::set_line(uint8_t *pBytemap, int cell_x1, int cell_y1, int cell_x2, int cell_y2, uint8_t cell_value)
{
	int deltaX = abs(cell_x2 - cell_x1);
	int deltaY = abs(cell_y2 - cell_y1);
	float diag = (float)sqrt((double)(deltaX*deltaX + deltaY*deltaY));
	int stepX = (cell_x2 >= cell_x1) ? 1 : -1;
	int stepY = (cell_y2 >= cell_y1) ? 1 : -1;
	int acc = 0;
	int count;
	int lineLength = 0;

	if (deltaX >= deltaY) //rather horizontal
	{
		count = deltaX + 1;

		while (count--)
		{
			set_cell_value(pBytemap, cell_x1, cell_y1, cell_value);

			mpLineInternalCoords[lineLength][0] = cell_x1;
			mpLineInternalCoords[lineLength][1] = cell_y1;
			lineLength++;
			acc += deltaY * 2;
			cell_x1 += stepX;

			if (acc >= deltaX)
			{
				acc -= deltaX * 2;
				cell_y1 += stepY;
			}
		}
	}
	else //rather vertical
	{
		count = deltaY + 1;

		while (count--)
		{
			set_cell_value(pBytemap, cell_x1, cell_y1, cell_value);

			mpLineInternalCoords[lineLength][0] = cell_x1;
			mpLineInternalCoords[lineLength][1] = cell_y1;
			lineLength++;
			acc += deltaX * 2;
			cell_y1 += stepY;

			if (acc >= deltaY)
			{
				acc -= deltaY * 2;
				cell_x1 += stepX;
			}
		}
	}

	mLastLineLength = lineLength;
}

void MapMatching::set_polygon_borders(uint8_t *pBytemap, int aPointCoords[][2], int numberOfPoints, uint8_t cell_value)
{
	int cell_x1 = 0;
	int cell_y1 = 0;
	int cell_x2 = 0;
	int cell_y2 = 0;

	for (int lineNumber = 0; lineNumber < numberOfPoints; lineNumber++)
	{
		cell_x1 = aPointCoords[lineNumber][0];
		cell_y1 = aPointCoords[lineNumber][1];
		cell_x2 = aPointCoords[(lineNumber + 1) % numberOfPoints][0];
		cell_y2 = aPointCoords[(lineNumber + 1) % numberOfPoints][1];
		set_line(pBytemap, cell_x1, cell_y1, cell_x2, cell_y2, cell_value);
		// additional thickness to prevent particles goind through thin walls
		if (cell_y1 != cell_y2)
			set_line(pBytemap, cell_x1 + 1, cell_y1, cell_x2 + 1, cell_y2, cell_value);
		else
			set_line(pBytemap, cell_x1, cell_y1 + 1, cell_x2, cell_y2 + 1, cell_value);
	}
}

void MapMatching::set_polygon(uint8_t *pBytemap, int aPointCoords[][2], int numberOfPoints, uint8_t cell_value)
{
	int aMinY[MAPMATCHING_MAXPOLYGON];
	int aMaxY[MAPMATCHING_MAXPOLYGON];
	int aX[MAPMATCHING_MAXPOLYGON];
	int lineNumber, myY, number_of_x;
	int cell_x1, cell_y1, cell_x2, cell_y2;
	int polygonMinY, polygonMaxY;
	int aYDirection[MAPMATCHING_MAXPOLYGON];
	int yDirectionLast;
	int aYCorrection[MAPMATCHING_MAXPOLYGON];

	polygonMinY = map_cell_count_Y;
	polygonMaxY = 0;

	if (numberOfPoints < 3)
		return;

	_ASSERT(numberOfPoints <= MAPMATCHING_MAXPOLYGON);

	//Solve a problem with last points of each line
	yDirectionLast = 0;

	for (lineNumber = 0; lineNumber < numberOfPoints; lineNumber++)
	{
		aYDirection[lineNumber] = 0;

		if (aPointCoords[lineNumber][1] < aPointCoords[(lineNumber + 1) % numberOfPoints][1])
		{
			yDirectionLast = aYDirection[lineNumber] = 1;
		}

		if (aPointCoords[lineNumber][1] > aPointCoords[(lineNumber + 1) % numberOfPoints][1])
		{
			yDirectionLast = aYDirection[lineNumber] = -1;
		}
	}

	for (lineNumber = 0; lineNumber < numberOfPoints; lineNumber++)
	{
		aYCorrection[lineNumber] = 0;

		if (aYDirection[lineNumber])
		{
			if (aYDirection[lineNumber] == yDirectionLast)
			{
				aYCorrection[lineNumber] = yDirectionLast;
			}
			else
			{
				yDirectionLast = aYDirection[lineNumber];
			}
		}
	}

	for (lineNumber = 0; lineNumber < numberOfPoints; lineNumber++)
	{
		cell_x1 = aPointCoords[lineNumber][0];
		cell_y1 = aPointCoords[lineNumber][1];
		cell_x2 = aPointCoords[(lineNumber + 1) % numberOfPoints][0];
		cell_y2 = aPointCoords[(lineNumber + 1) % numberOfPoints][1];
		set_line(pBytemap, cell_x1, cell_y1, cell_x2, cell_y2, cell_value);

		aMinY[lineNumber] = std::min(cell_y1, cell_y2);
		aMaxY[lineNumber] = std::max(cell_y1, cell_y2);
		polygonMinY = std::min(polygonMinY, aMinY[lineNumber]);
		polygonMaxY = std::max(polygonMaxY, aMaxY[lineNumber]);

		for (int i = 0; i < mLastLineLength; ++i)
		{
			myY = mpLineInternalCoords[i][1] - aMinY[lineNumber]; //take relative Y
			_ASSERT((myY >= 0) && (myY < map_cell_count_Y));
			mapPolygonXCoords[lineNumber][myY] = mpLineInternalCoords[i][0]; //store X
		}

		if (aYCorrection[lineNumber] && (aMinY[lineNumber] != aMaxY[lineNumber]))
		{
			if (aYDirection[lineNumber] == -1)
			{
				aMaxY[lineNumber]--;
			}
			else
			{
				aMinY[lineNumber]++;

				for (int i = 0; i < aMaxY[lineNumber] - aMinY[lineNumber]; ++i)
				{
					mapPolygonXCoords[lineNumber][i] = mapPolygonXCoords[lineNumber][i + 1];
				}
			}

		}
	}

	for (myY = polygonMinY; myY < polygonMaxY; myY++)
	{
		number_of_x = 0;

		for (lineNumber = 0; lineNumber < numberOfPoints; lineNumber++)
		{
			if ((myY >= aMinY[lineNumber]) && (myY <= aMaxY[lineNumber]) && aYDirection[lineNumber])
			{
				aX[number_of_x++] = mapPolygonXCoords[lineNumber][myY - aMinY[lineNumber]];
			}
		}

		_ASSERT((number_of_x & 1) == 0);
		bubble_sort(aX, number_of_x);

		for (lineNumber = 0; lineNumber < number_of_x / 2; lineNumber++)
		{
			fill_horizontal(pBytemap, aX[lineNumber * 2], myY, aX[lineNumber * 2 + 1] - aX[lineNumber * 2] + 1, cell_value);
		}
	}
}

void MapMatching::bubble_sort(int *pArray, int count)
{
	int temp;

	for (int i = 0; i < count - 1; ++i)
	{
		for (int j = 0; j < count - 1 - i; ++j)
		{
			if (pArray[j] > pArray[j + 1])
			{
				temp = pArray[j];
				pArray[j] = pArray[j + 1];
				pArray[j + 1] = temp;
			}
		}
	}
}

uint8_t MapMatching::check_line_validness(uint8_t *pBytemap, double x1, double y1, double x2, double y2)
{
	int cell_x1, cell_y1, cell_x2, cell_y2;
	int length = 0;
	uint8_t result = 255;
	user_to_bytemap_coords(x1, y1, cell_x1, cell_y1);
	user_to_bytemap_coords(x2, y2, cell_x2, cell_y2);
	
	int delta_x = abs(cell_x2 - cell_x1);
	int delta_y = abs(cell_y2 - cell_y1);
	int step_x = (cell_x2 >= cell_x1) ? 1 : -1;
	int step_y = (cell_y2 >= cell_y1) ? 1 : -1;
	int acc = 0;
	int count;

	uint8_t* pScannedLine = mpScannedLine;

	if (delta_x >= delta_y) //rather horizontal
	{
		count = delta_x + 1;
		length = count;

		while (count--)
		{
			*pScannedLine++ = get_cell_value(pBytemap, cell_x1, cell_y1);
			acc += delta_y * 2;
			cell_x1 += step_x;

			if (acc >= delta_x)
			{
				acc -= delta_x * 2;
				cell_y1 += step_y;
			}
		}
	}
	else //rather vertical
	{
		count = delta_y + 1;
		length = count;

		while (count--)
		{
			*pScannedLine++ = get_cell_value(pBytemap, cell_x1, cell_y1);
			acc += delta_x * 2;
			cell_y1 += step_y;

			if (acc >= delta_y)
			{
				acc -= delta_y * 2;
				cell_x1 += step_x;
			}
		}
	}

	for (int i = 0; i < length; ++i)
		if (result > mpScannedLine[i])
			result = mpScannedLine[i];

	return result;
}

double MapMatching::get_particle_weight(double x1, double y1, double x2, double y2, int floor1, int floor2)
{
	double weight = 1.0;

	if (floor1 != floor2)
	{
		weight = 1.0;
		return weight;
	}

	uint8_t* pBytemap = NULL;
	bool success = get_bytemap_from_floor_number(floor1, pBytemap);

	if (!success)
		return 1.0; // NOT killing any particles if map for current floor is not found
	
	uint8_t min_obstacle_value = check_line_validness(pBytemap, x1, y1, x2, y2);
	if ((int)min_obstacle_value == 0)
		weight = 0.0;
	else if ((int)min_obstacle_value == 255)
		weight = 1.0;
	else
	{
		// value is 125, the result is random 
		uint32_t max_rand = mm_rng->get_max_rand();
		uint32_t rand_int = mm_rng->uniform_rand();

		double rand_float = (double)rand_int / max_rand;

		if (rand_float >= transparency_threshold)
			weight = 1.0;
		else
			weight = 0.0;
	}

	return weight;
}

void MapMatching::fill_horizontal(uint8_t *pBytemap, int cell_x, int cell_y, int length, uint8_t cell_value)
{
	while (length--)
	{
		set_cell_value(pBytemap, cell_x, cell_y, cell_value);
		cell_x++;
	}
}

#if MM_DEBUG_OUT
void MapMatching::debug_store_bmp(uint8_t *pBytemap, std::string file_name)
{
	int w = map_cell_count_X; 
	int h = map_cell_count_Y; 
	
	FILE *f;
	unsigned char *img = NULL;
	int filesize = 54 + 3 * w*h;  //w is image width, h is image height, both int
	if (img)
		free(img);
	img = (unsigned char *)malloc(3 * w*h);
	memset(img, 0, sizeof(img));
	int x;
	int y;
	int r;
	int g;
	int b;

	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			x = i; y = (h - 1) - j;
					   
			r = 255;
			g = 255;
			b = 255;

			uint8_t val = get_cell_value(pBytemap, x, y);
			if (val == 0)
			{
				r = 127;
				g = 127;
				b = 127;
			}

			if (val == 125)
			{
				r = 0;
				g = 255;
				b = 0;
			}

			if (r > 255) r = 255;
			if (g > 255) g = 255;
			if (b > 255) b = 255;
			img[(x + y * w) * 3 + 2] = (unsigned char)(r);
			img[(x + y * w) * 3 + 1] = (unsigned char)(g);
			img[(x + y * w) * 3 + 0] = (unsigned char)(b);
		}
	}

	unsigned char bmpfileheader[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
	unsigned char bmpinfoheader[40] = { 40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0 };
	unsigned char bmppad[3] = { 0,0,0 };

	bmpfileheader[2] = (unsigned char)(filesize);
	bmpfileheader[3] = (unsigned char)(filesize >> 8);
	bmpfileheader[4] = (unsigned char)(filesize >> 16);
	bmpfileheader[5] = (unsigned char)(filesize >> 24);

	bmpinfoheader[4] = (unsigned char)(w);
	bmpinfoheader[5] = (unsigned char)(w >> 8);
	bmpinfoheader[6] = (unsigned char)(w >> 16);
	bmpinfoheader[7] = (unsigned char)(w >> 24);
	bmpinfoheader[8] = (unsigned char)(h);
	bmpinfoheader[9] = (unsigned char)(h >> 8);
	bmpinfoheader[10] = (unsigned char)(h >> 16);
	bmpinfoheader[11] = (unsigned char)(h >> 24);

	f = fopen(file_name.c_str(), "wb");
	fwrite(bmpfileheader, 1, 14, f);
	fwrite(bmpinfoheader, 1, 40, f);
	for (int i = 0; i < h; i++)
	{
		//fwrite(img + (w*(h - i - 1) * 3), 3, w, f);
		fwrite(img + (w*(i) * 3), 3, w, f);
		fwrite(bmppad, 1, (4 - (w * 3) % 4) % 4, f);
	}
	fclose(f);
	free(img);
}

void MapMatching::debug_store_txt(uint8_t *pBytemap, std::string file_name)
{
	std::ofstream outputStream;

	outputStream.open(file_name);

	if (outputStream.fail())
	{
		outputStream.close();
	}

	for (int i = 0; i < map_cell_count_X; ++i)
	{
		for (int j = 0; j < map_cell_count_Y; ++j)
		{
			uint8_t map_value = get_cell_value(pBytemap, i, j);
			outputStream << (int)map_value << ",";
		}

		outputStream << 999 << std::endl;
	}

	outputStream.close();
}

#endif

void MapMatching::GetMapCoords(double UserX, double UserY, int &MapX, int &MapY) const
{
    if (UserX < mMinX)
        UserX = mMinX;
    if (UserX > mMaxX)
        UserX = mMaxX;
    if (UserY < mMinY)
        UserY = mMinY;
    if (UserY > mMaxY)
        UserY = mMaxY;

    MapX = (int)floor((UserX - mMinX - mm_cell_size / 2) / mm_cell_size + 0.5);
    MapY = (int)floor((UserY - mMinY - mm_cell_size / 2) / mm_cell_size + 0.5);

    if (MapX >= map_cell_count_X)
        MapX = map_cell_count_X - 1;
    if (MapY >= map_cell_count_Y)
        MapY = map_cell_count_Y - 1;
}

int MapMatching::GetCellCenterPosition(int idx, double &X, double &Y) const
{
    int idx_floor = idx % (map_cell_count_X * map_cell_count_Y);
    int idx_y = idx_floor / map_cell_count_X;
    int idx_x = idx_floor % map_cell_count_X;

    X = idx_x * mm_cell_size - mMinX + mm_cell_size / 2;
    Y = idx_y * mm_cell_size - mMinY + mm_cell_size / 2;

    return idx / (map_cell_count_X * map_cell_count_Y);
}

// The function traversable status of cell which includes specified position (x,y)
int8_t  MapMatching::CheckPositionStatus(double x, double y, double level, uint8_t min_transparency_level)
{
    static uint8_t transparency_ = 1;
    int logical_floor = (int)floor(level + 0.5);
	uint8_t* p_bytemap;
	bool success = get_bytemap_from_floor_number(logical_floor, p_bytemap);
	if (!success)
		return 0;

    int cell_x; 
    int cell_y;
    user_to_bytemap_coords(x, y, cell_x, cell_y);
    uint8_t transparency_level = get_cell_value(p_bytemap, cell_x, cell_y);
    if (transparency_level < min_transparency_level)
    {
        //std::cout << "aaa:" << "   " << x << "   " << y << "   " 
        //<< cell_x << "   " << cell_y << "   " << (int)transparency_level << std::endl;
        return 0;
    }
    else
        return transparency_;
}

#ifndef M_PI
#define M_PI (3.1415926535897932384626433832795)
#endif

// The function returns list of coordinates of cells center of all traversable cells
// located in circle of radius R from specified position (x,y)
bool MapMatching::GetTraversableCells(double x, double y, double level, double R,
    uint8_t min_transparency_level,
    std::vector<std::pair<double, double>> &list)
{
    list.clear();
    int logical_floor = (int)floor(level + 0.5);
    uint8_t* p_bytemap;
	bool success = get_bytemap_from_floor_number(logical_floor, p_bytemap);
	if (!success)
		return false;

    double fi = 0;
    static const double pi2 = 2 * M_PI;
    static const double dfi = pi2 / 20;
    int    cell_x, cell_y;
    std::vector<int> indexes;
    while (fi < pi2)
    {
        double dx = R * cos(fi);
        double dy = R * sin(fi);
        double x_cur = x + dx;
        double y_cur = y + dy;
        if (x_cur < mMinX)
            x_cur = mMinX;
        if (y_cur < mMinY)
            y_cur = mMinY;
        if (x_cur > mMaxX)
            x_cur = mMaxX;
        if (y_cur > mMaxY)
            y_cur = mMaxY;
        user_to_bytemap_coords(x_cur, y_cur, cell_x, cell_y);
        
        uint8_t transparency_level = get_cell_value(p_bytemap, cell_x, cell_y);
        if (transparency_level >= min_transparency_level)
        {
            int index = ConvertCoordsToIndex(cell_x, cell_y, logical_floor);
            if (std::find(indexes.begin(), indexes.end(), index) == indexes.end())
            {
                indexes.push_back(index);
                double UserX;
                double UserY;
                bytemap_to_user_coords(cell_x, cell_y, UserX, UserY);
                //std::cout << "cc1:" << "   " << fi << "   " << dx << "   " << dy << std::endl;
                //std::cout << "cc2:" << "   " << x_cur << "   " << y_cur << "   " << UserX << "   " << UserY << "   " << cell_x << "   " << cell_y << "   " << (int)transparency_level << std::endl;
                auto p = std::make_pair(UserX, UserY);
                list.push_back(p);
            }
        }
        fi += dfi;
    }
    
    if(list.size() > 0)
        return true;
    else
        return false;
}

// this function is to be used by initializers - they give a floor and desired rectangular area, 
// then this function creates a list of available cell indexes to be used in random position generation inside initializer
// function returns "false" if floor doesn't exist in the map
// avaliable_square_meters is returned for initializer to calculate the necessary number of particles
bool MapMatching::set_area_for_initializer(int floor, std::vector<tMFP_Particle3D> mfp_cells, const double mfp_cell_size, double &avaliable_square_meters)
{
	double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;

	int min_cell_x = 0, min_cell_y = 0, max_cell_x = map_cell_count_X, max_cell_y = map_cell_count_Y;
	
	// init memory
	unsigned int area_cell_count = (max_cell_x - min_cell_x + 1) * (max_cell_y - min_cell_y + 1);
	int* p_cells_x = (int*)calloc(area_cell_count, sizeof(int));
	int* p_cells_y = (int*)calloc(area_cell_count, sizeof(int));

	cells_x_in_area.insert(std::pair<int8_t, int*>(floor, p_cells_x));
	cells_y_in_area.insert(std::pair<int8_t, int*>(floor, p_cells_y));

	uint8_t* p_current_bytemap = NULL;
	bool success = get_bytemap_from_floor_number(floor, p_current_bytemap);
	
	if (!success)
	{
		return false;
	}

	// get all available cells from p_current_bytemap for idx_x in range [min_cell_x ... max_cell_x], idx_y in range [min_cell_y ... max_cell_y]
	// add them to p_cells_x and p_cells_y arrays and write cells_in_area_count element

	uint8_t v = 0;
	size_t free_cell_iter = 0;

	std::vector<tMFP_Particle3D>::iterator it;
	for (it = mfp_cells.begin(); it != mfp_cells.end(); ++it)
	{
		min_x = (it->X - mfp_cell_size / 2) / 100;
		max_x = (it->X + mfp_cell_size / 2) / 100;
		min_y = (it->Y - mfp_cell_size / 2) / 100;
		max_y = (it->Y + mfp_cell_size / 2) / 100;
		
		user_to_bytemap_coords(min_x, min_y, min_cell_x, min_cell_y);
		user_to_bytemap_coords(max_x, max_y, max_cell_x, max_cell_y);

		for (int i = min_cell_x; i < max_cell_x; ++i)
		{
			for (int j = min_cell_y; j < max_cell_y; ++j)
			{
				v = get_cell_value(p_current_bytemap, i, j);

				if (v == 255)
				{
					p_cells_x[free_cell_iter] = i;
					p_cells_y[free_cell_iter] = j;
					++free_cell_iter;
				}
			}
		}
	}
	
	avaliable_square_meters = mm_cell_size * mm_cell_size * free_cell_iter;

	cells_in_area_count.insert(std::pair<int8_t, size_t>(floor, free_cell_iter));
	
	return true;
}

void MapMatching::clear_area_for_initializer()
{
	std::map<int8_t, int*>::iterator it;
	for (it = cells_x_in_area.begin(); it != cells_x_in_area.end(); ++it)
	{
		free(it->second);
	}

	for (it = cells_y_in_area.begin(); it != cells_y_in_area.end(); ++it)
	{
		free(it->second);
	}
	
	cells_x_in_area.clear();
	cells_y_in_area.clear();
	cells_in_area_count.clear();

}

bool MapMatching::get_random_particle_pos_from_area(double &x, double &y, int floor, double random_number)
{
	if (cells_x_in_area.find(floor) != cells_x_in_area.end())
	{
		int* p_cells_x = cells_x_in_area[floor];
		int* p_cells_y = cells_y_in_area[floor];
		size_t cell_count = cells_in_area_count[floor];

		int rnd_idx = std::floor(cell_count * random_number);
		int out_cell_x = p_cells_x[rnd_idx];
		int out_cell_y = p_cells_y[rnd_idx];

		bytemap_to_user_coords(out_cell_x, out_cell_y, x, y);
	}
	else
	{
		return false;
	}

	return true;
}