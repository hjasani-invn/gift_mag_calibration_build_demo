#ifndef __MAP_MATCHING_HPP
#define __MAP_MATCHING_HPP


#include <stdint.h>
#include "imm_header.h"
#include "imm_math.h"
#include "CoordinateConverter.h"
#include <map>
#include <vector>
#include  <utility>
#include "../magnetic/MFP.h"
#include "uniform_random.hpp"
#include "rng32.hpp"

#define MAPMATCHING_MAXPOLYGON 1024
#define MM_DEBUG_OUT 0

#ifndef _ASSERT
#define _ASSERT(expr) ((void)0)
#endif

typedef struct tagBITMAPFILEHEADER
{
	unsigned short bfType;
	unsigned long bfSize;
	unsigned short bfReserved1;
	unsigned short bfReserved2;
	unsigned long bfOffBits;
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
	unsigned long biSize;
	long biWidth;
	long biHeight;
	unsigned short biPlanes;
	unsigned short biBitCount;
	unsigned long biCompression;
	unsigned long biSizeImage;
	long biXPelsPerMeter;
	long biYPelsPerMeter;
	unsigned long biClrUsed;
	unsigned long biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;

class MapMatching
{
public:
	MapMatching(const uint8_t* const pMap, const size_t mapFileSizeInBytes, double max_X, double max_Y, int16_t floor_shift, bool floor_zero_enable, const GeoLocConverter2D &converter);

	~MapMatching();

	void clear_memory();

	// The function traversable status of cell which includes specified position (x,y)
	int8_t  CheckPositionStatus(double x, double y, double floor, uint8_t min_transparency_level);

	// The function returns list of coordinates of cells center of all traversable cells
	// located in circle of radius R from specified position (x,y)
	bool GetTraversableCells(double x, double y, double floor, double R,
		uint8_t min_transparency_level,
		std::vector<std::pair<double, double>>  &list);

	double get_particle_weight(double x1, double y1, double x2, double y2, int floor_1, int floor_2);

	bool get_validity();

	void GetMapDim(double & minX, double & maxX, double & minY, double & maxY)
	{
		minX = mMinX; maxX = mMaxX; minY = mMinY; maxY = mMaxY;
	}

	bool set_area_for_initializer(int floor, std::vector<tMFP_Particle3D> mfp_cells, const double mfp_cell_size, double &avaliable_square_meters);

	void clear_area_for_initializer();

	bool get_random_particle_pos_from_area(double &x, double &y, int floor, double random_number);

private:
	
    double GetMapCellSize() const
        { return mm_cell_size; }
    void GetMapCoords(double UserX, double UserY, int &MapX, int &MapY) const;
    int ConvertCoordsToIndex(int MapX, int MapY, int Floor) const
        { return (Floor - mMinFloor) * (map_cell_count_X * map_cell_count_Y) + MapY * map_cell_count_X + MapX; }
    bool CheckPosition(int X, int Y) const
        { return (X >= mMinX && X <= mMaxX && Y >= mMinY && Y <= mMaxY); }
    int GetCellCenterPosition(int idx, double &X, double &Y) const;

	void set_cell_value(uint8_t* pBytemap, int cell_x, int cell_y, uint8_t cell_value);
	uint8_t get_cell_value(uint8_t* pBytemap, int cell_x, int cell_y);
	void user_to_bytemap_coords(double x, double y, int &cell_x, int &cell_y);
	void bytemap_to_user_coords(int cell_x, int cell_y, double &UserX, double &UserY);
	void set_line(uint8_t *pBytemap, int cell_x1, int cell_y1, int cell_x2, int cell_y2, uint8_t cell_value);
	void set_polygon_borders(uint8_t *pBytemap, int aPointCoords[][2], int numberOfPoints, uint8_t cell_value);
	void set_polygon(uint8_t *pBytemap, int aPointCoords[][2], int numberOfPoints, uint8_t cell_value);
	uint8_t check_line_validness(uint8_t *pBytemap, double x1, double y1, double x2, double y2);

	bool check_bytemap_coordinates_validity(int cell_x, int cell_y);
	
	void fill_horizontal(uint8_t *pBytemap, int cell_x, int cell_y, int length, uint8_t NewCellValue);
	void bubble_sort(int *pArray, int count);

	int real_floor_to_logical_floor(int real_floor);
	bool get_bytemap_from_floor_number(int logical_floor, uint8_t* &pBytemap);
	void debug_store_bmp(uint8_t *pBitmap, std::string file_name);
	void debug_store_txt(uint8_t *pBytemap, std::string file_name);

	std::map<int8_t, uint8_t*> MM_Bytemaps;
	size_t map_cell_count_X; // number of cells in X direction
	size_t map_cell_count_Y; // number of cells in Y direction
	double mm_cell_size; // cell size in meters
	size_t one_floor_byte_size;
	int *mapPolygonXCoords[MAPMATCHING_MAXPOLYGON];
	int16_t mm_floor_shift;
	bool mm_floor_zero_enable;

	bool is_valid;

	int mLastLineLength;            // length of the last painted line
	int (*mpLineInternalCoords)[2]; // coordinates of all points belonging to the last painted line
	
	std::map<int8_t, int*> cells_x_in_area; // x coordinates of all available cells for each floor number (to be used by initializers)
	std::map<int8_t, int*> cells_y_in_area; // y coordinates of all available cells for each floor number (to be used by initializers)
	std::map<int8_t, size_t> cells_in_area_count; // for each floor total count of available cells in area specified by initializer

	uint8_t* mpScannedLine; // buffer with cell values belonging to the last scanned line

    double mMinX, mMaxX, mMinY, mMaxY; //Physical coordinates in a local system that bound the area of a map
    int    mMinFloor, mMaxFloor;

	pf_random::uniform_random *mm_rng; // to be used for obstacle opacity calculations

	const double transparency_threshold = 2.0;
};

#endif //__MAP_MATCHING_HPP