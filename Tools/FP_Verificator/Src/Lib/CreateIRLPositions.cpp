
#include "CreateIRLPositions.hpp"
#include "CoordinateConverter.h"
#include "imm_mapobject.h"

namespace FPVerificator
{
    void create_irl_positions(
        // input
        Venue &venue, // venue struct with building parameters
        char* pBuffer, // buffer with IRL data
        uint32_t buffer_size, // buffer size 
        double interpolation_interval, // interval of interpolation for IRL positions
        // output
        std::vector<OccupancyOfPosition> &irlpositions // IRL positions with number of measuments
        )
    {
        GeoLocConverter2D geo2local;
        bool res = geo2local.SetFrameParams(venue.origin_lattitude, venue.origin_longitude, venue.alfa, venue.beta, venue.origin_azimuth);

        MapObject_t* gMapObject = mapobject_init((uint8_t *)pBuffer, buffer_size);
        mapobject_process(gMapObject);
        //////////////////

        std::vector<OccupancyOfPosition> tmpposlist;

        {
            MapDrawing_t *mapdraw = &vector_at(gMapObject->mapdrawing, MapDrawing_t, 0);
            MapLvl_t *maplvl;
            uint32_t i, j, k;
            MapNode_t *mapnode;

            OccupancyOfPosition  pos;

            if (gMapObject->grid_available)
            {
                for (i = 0; i < mapdraw->maplvl->len; i++)
                {
                    maplvl = &vector_at(mapdraw->maplvl, MapLvl_t, i);

                    // cycle by nodes
                    for (j = 0; j < maplvl->mapnodes->len; j++)
                    {
                        mapnode = &vector_at(maplvl->mapnodes, MapNode_t, j);

                        double node_lat, node_lon;
                        double node_X, node_Y;
                        mapobject_get_latlon(gMapObject, &node_lat, &node_lon, &mapnode->map_pos);
                        //                 std::cout << node_lat << "         " << node_lon;
                        // NWD conversion - this convertion is provided by ?oordinate?onverter
                        geo2local.Geo2Local(node_lat, node_lon, &node_X, &node_Y);
                        pos.X = node_X;
                        pos.Y = node_Y;
                        tmpposlist.push_back(pos);

                        for (k = 0; k < mapnode->link_node->len; k++)
                        {
                            MapNode_t *node_i = (MapNode_t *)ptr_vector_at(mapnode->link_node, k);

                            double node_i_lat, node_i_lon;
                            double node_i_X, node_i_Y;

                            mapobject_get_latlon(gMapObject, &node_i_lat, &node_i_lon, &node_i->map_pos);
                            //                      std::cout << node_i_lat << "         " << node_i_lon;
                            // NWD conversion - this convertion is provided by ?oordinate?onverter
                            geo2local.Geo2Local(node_i_lat, node_i_lon, &node_i_X, &node_i_Y);
                            pos.X = node_i_X;
                            pos.Y = node_i_Y;
                            tmpposlist.push_back(pos);
#if 1 /////

                            double dX = -(node_X - node_i_X);
                            double dY = -(node_Y - node_i_Y);

                            double X = node_X;
                            double Y = node_Y;

                            double lat, lon;

                            int nX = (int)(dX / interpolation_interval + 1);
                            int nY = (int)(dY / interpolation_interval + 1);
                            int N = (std::max)(nX, nY);

                            dX /= N;
                            dY /= N;

                            for (int n = 1; n < N; n++)
                            {
                                X += dX;
                                Y += dY;
                                pos.X = X;
                                pos.Y = Y;
                                tmpposlist.push_back(pos);

                            }
#endif
                        }
                    } // end of cycle by nodes

                }
            }
        }


        std::sort(tmpposlist.begin(), tmpposlist.end(), compare_occupancy_of_positions_by_x);

        //std::vector<PositionAndMagOccupancy> diffposlist;
        double difference = interpolation_interval / 2;

        auto it_prev = tmpposlist.begin();
        OccupancyOfPosition  pos_prev = *it_prev;
        irlpositions.push_back(pos_prev);

        auto it = tmpposlist.begin();
        it++;
        OccupancyOfPosition  pos;

        while (it != tmpposlist.end())
        {
            pos = *it;
            if (
                (fabs(pos_prev.X - pos.X) > difference) ||
                (fabs(pos_prev.Y - pos.Y) > difference)
                )
            {
                irlpositions.push_back(pos);
                it_prev = it;
                pos_prev = *it_prev;
            }
            it++;
        }
        /*
        uint32_t  all = irlpositions.size();
        uint32_t  grey = 0;
        uint32_t  red = 0;
        uint32_t  yellow = 0;
        uint32_t  blue = 0;
        uint32_t  green = 0;
        */
    }
}
