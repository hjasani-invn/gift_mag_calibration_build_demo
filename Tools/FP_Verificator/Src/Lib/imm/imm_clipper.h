#ifndef _IMM_CLIPPER_H_
#define _IMM_CLIPPER_H_

#include "imm_types.h"
#include "imm_geoobject.h"
#include "imm_geo_int64.h"


IMM_BEGIN_DECLS

typedef struct clipper_tag clipper_t;

clipper_t* clipper_init();

void clipper_offset_polygons (int_polygons_t* subject, 
	                          int_polygons_t* solution, 
	                          double clipper_offset, 
	                          JoinType_t join_type, 
	                          double miter_limit);

bool_t clipper_add_polygons(clipper_t *clipper, const int_polygons_t *ppg, PolyType_t poly_type);

void clipper_free(clipper_t *clipper);

bool_t clipper_execute (clipper_t *clipper,
	                    ClipType_t clip_type, 
	                    int_polygons_t *solution,
	                    PolyFillType_t subj_filltype, 
	                    PolyFillType_t clip_filltype);

double int_polygon_area (const int_polygon_t *poly);

IMM_END_DECLS

#endif