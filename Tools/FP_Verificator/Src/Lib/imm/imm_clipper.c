#include <string.h>
#include <math.h>

#include "imm_clipper.h"
#include "imm_memory.h"

#define LOW_RANGE        (1518500249)
#define HORIZONTAL       (-1.0E+40)
#define TOLERANCE        (1.0e-20)

#ifndef PI
#define PI        (3.1415926535897932384626433832795) //!< better value
#endif

typedef enum { esNeither = 0, esLeft = 1, esRight = 2, esBoth = 3 } edge_side_t;
typedef enum { ipNone = 0, ipLeft = 1, ipRight = 2, ipBoth = 3 } intersect_protects_t;
typedef enum { dRightToLeft, dLeftToRight } direction_t;

// TYPES ONLY USED IN CLIPPER!
typedef ptr_vector_t edge_list_t;
typedef ptr_vector_t polyout_list_t;
typedef ptr_vector_t join_list_t;
typedef ptr_vector_t horz_join_list_t;


typedef struct _cledge_t {
	int64_t                xbot;
	int64_t                ybot;
	int64_t                xcurr;
	int64_t                ycurr;
	int64_t                xtop;
	int64_t                ytop;
	double                 dx;
	int64_t                tmp_x;
	PolyType_t             poly_type;
	edge_side_t            side;
	int32_t                wind_delta; //1 or -1 depending on winding direction
	int32_t                wind_cnt;
	int32_t                wind_cnt2;  //winding count of the opposite poly_type
	int32_t                out_idx;
	struct _cledge_t       *next;
	struct _cledge_t       *prev;
	struct _cledge_t       *next_in_lml;
	struct _cledge_t       *next_in_ael;
	struct _cledge_t       *prev_in_ael;
	struct _cledge_t       *next_in_sel;
	struct _cledge_t       *prev_in_sel;
}cledge_t;


typedef struct _local_minima_t{
	int64_t                 Y;
	cledge_t                *leftBound;
	cledge_t                *rightBound;
	struct _local_minima_t  *next;
}local_minima_t;


typedef struct _scanbeam_t{
	int64_t                 Y;
	struct _scanbeam_t      *next;
}scanbeam_t;


typedef struct _intersect_node_t {
	cledge_t                 *edge1;
	cledge_t                 *edge2;
	vector2d_t               pt;
	struct _intersect_node_t *next;
}intersect_node_t;


typedef struct _out_pt_t {
	int32_t                  idx;
	vector2d_t               pt;
	struct _out_pt_t         *next;
	struct _out_pt_t         *prev;
}out_pt_t;


typedef struct _out_rec_t{
	int32_t                  idx;
	bool_t                   is_hole;
	struct _out_rec_t        *first_left;
	struct _out_rec_t        *append_link;
	out_pt_t                 *pts;
	out_pt_t                 *bottom_pt;
	out_pt_t                 *bottom_flag;
	edge_side_t              sides;
}out_rec_t;


struct clipper_tag{
	PolyFillType_t            subject_filltype;
	PolyFillType_t            clip_filltype;
	ClipType_t                clip_type;
	local_minima_t            *minima_list;
	local_minima_t            *current_lm;
	scanbeam_t                *scanbeam;
	cledge_t                  *active_edges;
	cledge_t                  *sorted_edges;
	intersect_node_t          *intersect_nodes;
	edge_list_t               *m_edges;
	polyout_list_t            *poly_outs;
	join_list_t               *joins;
	horz_join_list_t          *horz_joins;
	bool_t                    reverse_output;
};


typedef struct  {
	double                     X;
	double                     Y;
}double_point_t;


typedef struct  {
	int64_t                    left;
	int64_t                    top;
	int64_t                    right;
	int64_t                    bottom;
}int_rect_t;


typedef struct  {
	vector2d_t                 pt1a;
	vector2d_t                 pt1b;
	int32_t                    poly1_idx;
	vector2d_t                 pt2a;
	vector2d_t                 pt2b;
	int                        poly2_idx;
}join_rec_t;


typedef struct  {
	cledge_t                   *edge;
	int32_t                    saved_idx;
}horz_join_rec_t;


static void
clipper_internal_free(clipper_t *clipper);


static int64_t
Round(double val)
{
	if ((val < 0))
		return (int64_t)(val - 0.5);
	else
		return (int64_t)(val + 0.5);
}


static bool_t
slopes_equal(const vector2d_t* pt1, const vector2d_t* pt2, const vector2d_t* pt3)
{
	return (pt1->Y - pt2->Y) * (pt2->X - pt3->X) == (pt1->X - pt2->X) * (pt2->Y - pt3->Y);
}


static bool_t
points_slopes_equal(const vector2d_t *pt1, const vector2d_t *pt2,
const vector2d_t *pt3, const vector2d_t *pt4)
{
	bool_t is_equal = (pt1->Y - pt2->Y) * (pt3->X - pt4->X) == (pt1->X - pt2->X) * (pt3->Y - pt4->Y);
	return is_equal;

	//return (pt1->Y-pt2->Y) * (pt3->X-pt4->X) == (pt1->X - pt2->X) * (pt3->Y - pt4->Y);
}


static bool_t
edge_slopes_equal(const cledge_t *e1, const cledge_t *e2)
{
	return (e1->ytop - e1->ybot) * (e2->xtop - e2->xbot) ==
		(e1->xtop - e1->xbot) * (e2->ytop - e2->ybot);
}


static bool_t
intersect_point(cledge_t *edge1, cledge_t *edge2, vector2d_t *ip)
{
	double b1, b2;

	if (edge_slopes_equal(edge1, edge2))
		return FALSE;
	else if (NEAR_ZERO(edge1->dx)) {
		ip->X = edge1->xbot;
		if (NEAR_EQUAL(edge2->dx, HORIZONTAL)) {
			ip->Y = edge2->ybot;
		}
		else {
			b2 = edge2->ybot - (edge2->xbot / edge2->dx);
			ip->Y = Round(ip->X / edge2->dx + b2);
		}
	}
	else if (NEAR_ZERO(edge2->dx)) {
		ip->X = edge2->xbot;
		if (NEAR_EQUAL(edge1->dx, HORIZONTAL)) {
			ip->Y = edge1->ybot;
		}
		else {
			b1 = edge1->ybot - (edge1->xbot / edge1->dx);
			ip->Y = Round(ip->X / edge1->dx + b1);
		}
	}
	else {
		b1 = edge1->xbot - edge1->ybot * edge1->dx;
		b2 = edge2->xbot - edge2->ybot * edge2->dx;
		b2 = (b2 - b1) / (edge1->dx - edge2->dx);
		ip->Y = Round(b2);
		ip->X = Round(edge1->dx * b2 + b1);
	}

	return
		//can be *so close* to the top of one edge that the rounded Y equals one ytop ...
		(ip->Y == edge1->ytop && ip->Y >= edge2->ytop && edge1->tmp_x > edge2->tmp_x) ||
		(ip->Y == edge2->ytop && ip->Y >= edge1->ytop && edge1->tmp_x > edge2->tmp_x) ||
		(ip->Y > edge1->ytop && ip->Y > edge2->ytop);
}


static void
set_dx(cledge_t* e)
{
	if (e->ybot == e->ytop)
		e->dx = HORIZONTAL;
	else
		e->dx = (double)(e->xtop - e->xbot) / (double)(e->ytop - e->ybot);
}


static double
get_dx(const vector2d_t *pt1, const vector2d_t *pt2)
{
	if (pt1->Y == pt2->Y)
		return HORIZONTAL;
	else
		return (double)(pt2->X - pt1->X) / (double)(pt2->Y - pt1->Y);
}


static int64_t
top_X(cledge_t *edge, const int64_t currentY)
{
	if (currentY == edge->ytop)
		return edge->xtop;

	return edge->xbot + Round(edge->dx * (currentY - edge->ybot));
}


static void
swap_X(cledge_t* e)
{
	//swap horizontal edges' top and bottom x's so they follow the natural
	//progression of the bounds - ie so their xbots will align with the
	//adjoining lower edge. [Helpful in the ProcessHorizontal() method.]
	e->xcurr = e->xtop;
	e->xtop = e->xbot;
	e->xbot = e->xcurr;
}


static void
swap_points(vector2d_t *pt1, vector2d_t *pt2)
{
	vector2d_t tmp = *pt1;
	*pt1 = *pt2;
	*pt2 = tmp;
}


static void
swap_sides(cledge_t *edge1, cledge_t *edge2)
{
	edge_side_t side = edge1->side;
	edge1->side = edge2->side;
	edge2->side = side;
}


static void
swap_poly_indexes(cledge_t *edge1, cledge_t *edge2)
{
	int32_t out_idx = edge1->out_idx;
	edge1->out_idx = edge2->out_idx;
	edge2->out_idx = out_idx;
}


static bool_t
get_overlap_segment(vector2d_t pt1a, vector2d_t pt1b, vector2d_t pt2a,
vector2d_t pt2b, vector2d_t* pt1, vector2d_t* pt2)
{
	//precondition: segments are colinear.
	if (pt1a.Y == pt1b.Y || ABS((pt1a.X - pt1b.X) / (pt1a.Y - pt1b.Y)) > 1) {
		if (pt1a.X > pt1b.X)
			swap_points(&pt1a, &pt1b);

		if (pt2a.X > pt2b.X)
			swap_points(&pt2a, &pt2b);

		if (pt1a.X > pt2a.X)
			*pt1 = pt1a;
		else
			*pt1 = pt2a;

		if (pt1b.X < pt2b.X)
			*pt2 = pt1b;
		else
			*pt2 = pt2b;

		return pt1->X < pt2->X;
	}
	else {
		if (pt1a.Y < pt1b.Y)
			swap_points(&pt1a, &pt1b);

		if (pt2a.Y < pt2b.Y)
			swap_points(&pt2a, &pt2b);

		if (pt1a.Y < pt2a.Y)
			*pt1 = pt1a;
		else
			*pt1 = pt2a;

		if (pt1b.Y > pt2b.Y)
			*pt2 = pt1b;
		else
			*pt2 = pt2b;

		return (pt1->Y > pt2->Y);
	}
}


static bool_t
orientation(out_rec_t *out_rec)
{
	//first make sure bottomPt is correctly assigned ...
	out_pt_t *op_bottom = out_rec->pts, *op = out_rec->pts->next;
	out_pt_t *op_prev, *op_next;
	vector2d_t ip1, ip2;

	while (op != out_rec->pts) {
		if (op->pt.Y >= op_bottom->pt.Y) {
			if (op->pt.Y > op_bottom->pt.Y || op->pt.X < op_bottom->pt.X)
				op_bottom = op;
		}
		op = op->next;
	}
	out_rec->bottom_pt = op_bottom;
	op_bottom->idx = out_rec->idx;

	op = op_bottom;
	//find vertices either side of bottomPt (skipping duplicate points) ....
	op_prev = op->prev;
	op_next = op->next;
	while (op != op_prev && int_vertex_equal(&op->pt, &op_prev->pt))
		op_prev = op_prev->prev;

	while (op != op_next && int_vertex_equal(&op->pt, &op_next->pt))
		op_next = op_next->next;

	ip1.X = op->pt.X - op_prev->pt.X;
	ip1.Y = op->pt.Y - op_prev->pt.Y;
	ip2.X = op_next->pt.X - op->pt.X;
	ip2.Y = op_next->pt.Y - op->pt.Y;

	return (ip1.X * ip2.Y - ip2.X * ip1.Y) > 0;
}


static double
internal_area(const out_rec_t *out_rec)
{
	out_pt_t *op = out_rec->pts;
	double a = 0.0;

	do {
		a += (op->prev->pt.X * op->pt.Y) - (op->pt.X * op->prev->pt.Y);
		op = op->next;
	} while (op != out_rec->pts);

	return a / 2.0;

}


double
int_polygon_area(const int_polygon_t *poly)
{
	uint32_t highI = poly->len - 1;
	double a;
	uint32_t i;

	return_val_if_fail(highI >= 2, 0.0);

	a = (double)int_polygon_at(poly, highI).X * int_polygon_at(poly, 0).Y -
		(double)int_polygon_at(poly, 0).X * int_polygon_at(poly, highI).Y;

	for (i = 0; i < highI; ++i) {
		a += (double)int_polygon_at(poly, i).X * int_polygon_at(poly, i + 1).Y -
			(double)int_polygon_at(poly, i + 1).X * int_polygon_at(poly, i).Y;
	}

	return a / 2.0;
}

static bool_t
point_is_vertex(const vector2d_t *pt, out_pt_t *pp)
{
	out_pt_t *pp2 = pp;
	do {
		if (int_vertex_equal(&pp2->pt, pt))
			return TRUE;

		pp2 = pp2->next;
	} while (pp2 != pp);

	return FALSE;
}

static bool_t
point_in_polygon(const vector2d_t *pt, out_pt_t *pp)
{
	out_pt_t *pp2 = pp;
	bool_t result = FALSE;

	do {
		if ((((pp2->pt.Y <= pt->Y) && (pt->Y < pp2->prev->pt.Y)) ||
			((pp2->prev->pt.Y <= pt->Y) && (pt->Y < pp2->pt.Y))) &&
			(pt->X < (pp2->prev->pt.X - pp2->pt.X) * (pt->Y - pp2->pt.Y) /
			(pp2->prev->pt.Y - pp2->pt.Y) + pp2->pt.X))
			result = !result;

		pp2 = pp2->next;
	} while (pp2 != pp);

	return result;
}

static int_polygon_t*
clipper_build_arc(const vector2d_t* pt,
const double a1,
const double a2,
const double r)
{
	uint32_t steps = MAX(6, (uint32_t)(sqrt(ABS(r)) * ABS(a2 - a1)));
	int_polygon_t* result = int_polygon_sized_new(steps);
	double da = (a2 - a1) / (steps - 1);
	double a = a1;
	uint32_t i;

	for (i = 0; i < steps; ++i) {
		int_polygon_at(result, i).X = pt->X + Round(cos(a)*r);
		int_polygon_at(result, i).Y = pt->Y + Round(sin(a)*r);
		a += da;
	}
	return result;
}


static double_point_t
clipper_get_unit_normal(const vector2d_t *pt1, const vector2d_t *pt2)
{
	double_point_t double_pt;
	double dx, dy, f;

	if (int_vertex_equal(pt1, pt2)) {
		double_pt.X = 0.0;
		double_pt.Y = 0.0;
		return double_pt;
	}

	dx = (double)(pt2->X - pt1->X);
	dy = (double)(pt2->Y - pt1->Y);
	f = 1 * 1.0 / sqrt(dx*dx + dy*dy);
	dx *= f;
	dy *= f;
	double_pt.X = dy;
	double_pt.Y = -dx;
	return double_pt;
}

static void
edge_init(cledge_t *e, cledge_t *e_next, cledge_t *e_prev, const vector2d_t *pt, PolyType_t poly_type)
{
	memset(e, 0, sizeof (cledge_t));

	e->next = e_next;
	e->prev = e_prev;
	e->xcurr = pt->X;
	e->ycurr = pt->Y;

	if (e->ycurr >= e->next->ycurr) {
		e->xbot = e->xcurr;
		e->ybot = e->ycurr;
		e->xtop = e->next->xcurr;
		e->ytop = e->next->ycurr;
		e->wind_delta = 1;
	}
	else {
		e->xtop = e->xcurr;
		e->ytop = e->ycurr;
		e->xbot = e->next->xcurr;
		e->ybot = e->next->ycurr;
		e->wind_delta = -1;
	}
	set_dx(e);
	e->poly_type = poly_type;
	e->out_idx = -1;
}

static void
insert_local_minima(clipper_t *clipper, local_minima_t *new_lm)
{
	local_minima_t *tmp_lm;
	if (!clipper->minima_list) {
		clipper->minima_list = new_lm;
	}
	else if (new_lm->Y >= clipper->minima_list->Y) {
		new_lm->next = clipper->minima_list;
		clipper->minima_list = new_lm;
	}
	else {
		tmp_lm = clipper->minima_list;
		while (tmp_lm->next && (new_lm->Y < tmp_lm->next->Y))
			tmp_lm = tmp_lm->next;

		new_lm->next = tmp_lm->next;
		tmp_lm->next = new_lm;
	}
}


static cledge_t*
add_bounds_to_lml(clipper_t *clipper, cledge_t *e)
{
	//Starting at the top of one bound we progress to the bottom where there's
	//a local minima. We then go to the top of the next bound. These two bounds
	//form the left and right (or right and left) bounds of the local minima.
	local_minima_t* new_lm;
	e->next_in_lml = NULL;
	e = e->next;
	for (;;) {
		if (NEAR_EQUAL(e->dx, HORIZONTAL)) {
			//nb: proceed through horizontals when approaching from their right,
			//    but break on horizontal minima if approaching from their left.
			//    This ensures 'local minima' are always on the left of horizontals.
			if (e->next->ytop < e->ytop && e->next->xbot > e->prev->xbot)
				break;

			if (e->xtop != e->prev->xbot)
				swap_X(e);

			e->next_in_lml = e->prev;
		}
		else if (e->ycurr == e->prev->ycurr)
			break;
		else
			e->next_in_lml = e->prev;
		e = e->next;
	}

	//e and e.prev are now at a local minima ...
	new_lm = (local_minima_t *)mem_allocate(sizeof (local_minima_t));
	new_lm->next = NULL;
	new_lm->Y = e->prev->ybot;

	if (NEAR_EQUAL(e->dx, HORIZONTAL)) {
		if (e->xbot != e->prev->xbot)
			swap_X(e);

		new_lm->leftBound = e->prev;

		new_lm->rightBound = e;
	}
	else if (e->dx < e->prev->dx) {
		new_lm->leftBound = e->prev;
		new_lm->rightBound = e;
	}
	else {
		new_lm->leftBound = e;
		new_lm->rightBound = e->prev;
	}
	new_lm->leftBound->side = esLeft;
	new_lm->rightBound->side = esRight;

	insert_local_minima(clipper, new_lm);

	for (;;) {
		if (e->next->ytop == e->ytop && !NEAR_EQUAL(e->next->dx, HORIZONTAL))
			break;
		e->next_in_lml = e->next;
		e = e->next;

		if (NEAR_EQUAL(e->dx, HORIZONTAL) && e->xbot != e->prev->xtop)
			swap_X(e);
	}
	return e->next;
}

void
clipper_offset_square(int_polygon_t* poly,
const double mul,
const vector2d_t* m_p,
const double_point_t* normal_mj,
const double_point_t* normal_mk,
const double m_delta)
{
	vector2d_t pt1, pt2;
	double a1, a2, dx;

	pt1.X = (int64_t)Round(m_p->X + normal_mk->X * m_delta);
	pt1.Y = (int64_t)Round(m_p->Y + normal_mk->Y * m_delta);

	pt2.X = (int64_t)Round(m_p->X + normal_mj->X * m_delta);
	pt2.Y = (int64_t)Round(m_p->Y + normal_mj->Y * m_delta);

	if ((normal_mk->X * normal_mj->Y - normal_mj->X * normal_mk->Y) * m_delta >= 0) {
		a1 = atan2(normal_mk->Y, normal_mk->X);
		a2 = atan2(-normal_mj->Y, -normal_mj->X);
		a1 = ABS(a2 - a1);

		if (a1 > PI)
			a1 = 2 * PI - a1;

		dx = tan((PI - a1) / 4.0) * ABS(m_delta * mul);

		pt1.X = (int64_t)(pt1.X - normal_mk->Y * dx),
			pt1.Y = (int64_t)(pt1.Y + normal_mk->X * dx);
		int_polygon_append_vertice(poly, pt1);

		pt2.X = (int64_t)(pt2.X + normal_mj->Y * dx);
		pt2.Y = (int64_t)(pt2.Y - normal_mj->X * dx);
		int_polygon_append_vertice(poly, pt2);
	}
	else {
		int_polygon_append_vertice(poly, pt1);
		int_polygon_append_vertice(poly, *m_p);
		int_polygon_append_vertice(poly, pt2);
	}
}


static void
clipper_offset_miter(int_polygon_t *poly,
const vector2d_t* m_p,
const double_point_t* normal_mj,
const double_point_t* normal_mk,
const double m_delta,
const double m_R)
{
	double q;
	vector2d_t pt1, pt2;

	if ((normal_mk->X * normal_mj->Y - normal_mj->X * normal_mk->Y) * m_delta >= 0) {
		q = m_delta / m_R;

		pt1.X = (int64_t)Round(m_p->X + (normal_mk->X + normal_mj->X) * q);
		pt1.Y = (int64_t)Round(m_p->Y + (normal_mk->Y + normal_mj->Y) * q);
		int_polygon_append_vertice(poly, pt1);
	}
	else {
		pt1.X = (int64_t)Round(m_p->X + normal_mk->X * m_delta);
		pt1.Y = (int64_t)Round(m_p->Y + normal_mk->Y * m_delta);

		pt2.X = (int64_t)Round(m_p->X + normal_mj->X * m_delta);
		pt2.Y = (int64_t)Round(m_p->Y + normal_mj->Y * m_delta);

		int_polygon_append_vertice(poly, pt1);
		int_polygon_append_vertice(poly, *m_p);
		int_polygon_append_vertice(poly, pt2);
	}
}

static void
clipper_offset_round(int_polygon_t *poly,
const vector2d_t* m_p,
const double_point_t* normal_mj,
const double_point_t* normal_mk,
const double m_delta)
{
	vector2d_t pt1, pt2;
	uint32_t m;
	double a1, a2;
	int_polygon_t *arc;

	pt1.X = (int64_t)Round(m_p->X + normal_mk->X * m_delta);
	pt1.Y = (int64_t)Round(m_p->Y + normal_mk->Y * m_delta);

	pt2.X = (int64_t)Round(m_p->X + normal_mj->X * m_delta);
	pt2.Y = (int64_t)Round(m_p->Y + normal_mj->Y * m_delta);

	int_polygon_append_vertice(poly, pt1);

	//round off reflex angles (ie > 180 deg) unless almost flat (ie < ~10deg).
	if ((normal_mk->X * normal_mj->Y - normal_mj->X * normal_mk->Y) * m_delta >= 0) {
		if (normal_mk->X * normal_mk->X + normal_mj->Y * normal_mk->Y < 0.985) {
			a1 = atan2(normal_mk->Y, normal_mk->X);
			a2 = atan2(normal_mj->Y, normal_mj->X);

			if (m_delta > 0 && a2 < a1)
				a2 += PI * 2.0;
			else if (m_delta < 0 && a2 > a1)
				a2 -= PI * 2;

			arc = clipper_build_arc(m_p, a1, a2, m_delta);

			for (m = 0; m < arc->len; m++)
				int_polygon_append_vertice(poly, int_polygon_at(arc, m));

			int_polygon_free(arc);
		}
	}
	else
		int_polygon_append_vertice(poly, *m_p);

	int_polygon_append_vertice(poly, pt2);
}

static void
pop_local_minima(clipper_t *clipper)
{
	if (!clipper->current_lm)
		return;

	clipper->current_lm = clipper->current_lm->next;
}
//------------------------------------------------------------------------------

static int_rect_t
get_bounds(clipper_t *clipper)
{
	int_rect_t result;
	local_minima_t* lm = clipper->minima_list;
	cledge_t *e, *bottom_e;

	if (!lm) {
		result.left = result.top = result.right = result.bottom = 0;
		return result;
	}

	result.left = lm->leftBound->xbot;
	result.top = lm->leftBound->ybot;
	result.right = lm->leftBound->xbot;
	result.bottom = lm->leftBound->ybot;

	while (lm) {
		if (lm->leftBound->ybot > result.bottom)
			result.bottom = lm->leftBound->ybot;

		e = lm->leftBound;
		while (1) {
			bottom_e = e;
			while (e->next_in_lml) {
				if (e->xbot < result.left)
					result.left = e->xbot;

				if (e->xbot > result.right)
					result.right = e->xbot;
				e = e->next_in_lml;
			}

			if (e->xbot < result.left)
				result.left = e->xbot;

			if (e->xbot > result.right)
				result.right = e->xbot;

			if (e->xtop < result.left)
				result.left = e->xtop;

			if (e->xtop > result.right)
				result.right = e->xtop;

			if (e->ytop < result.top)
				result.top = e->ytop;

			if (bottom_e == lm->leftBound)
				e = lm->rightBound;
			else
				break;
		}
		lm = lm->next;
	}
	return result;
}

clipper_t*
clipper_init()
{
	clipper_t *clipper = (clipper_t *)mem_allocate(sizeof(clipper_t));
	clipper->minima_list = NULL;
	clipper->current_lm = NULL;
	clipper->scanbeam = NULL;
	clipper->active_edges = NULL;
	clipper->sorted_edges = NULL;
	clipper->intersect_nodes = NULL;
	clipper->reverse_output = FALSE;

	clipper->m_edges = ptr_vector_new();
	clipper->poly_outs = ptr_vector_new();
	clipper->joins = ptr_vector_new();
	clipper->horz_joins = ptr_vector_new();

	return clipper;
}


static bool_t
clipper_add_polygon(clipper_t *clipper, const int_polygon_t *pg, PolyType_t poly_type)
{
	uint32_t len = pg->len;
	int_polygon_t *p;
	cledge_t *e, *e_highest;
	uint32_t i, j = 0;
	int64_t maxVal;
	cledge_t *edges;

	if (len < 3) {
		printf("[Clipper ERROR]: Not sufficient vertices. \n");
		return FALSE;
	}

	p = int_polygon_sized_new(len);

	int_polygon_copy_vertex(p, 0, pg, 0);

	maxVal = LOW_RANGE;

	for (i = 0; i < len; ++i) {
		if (ABS(int_polygon_at(pg, i).X) > maxVal || ABS(int_polygon_at(pg, i).Y) > maxVal) {
			printf("[Clipper ERROR]: Range invalid.\n");
			int_polygon_free(p);
			return FALSE;
		}

		if (i == 0 || int_vertex_equal(&int_polygon_at(p, j), &int_polygon_at(pg, i))) {
			continue;
		}
		else if (j > 0 && slopes_equal(&int_polygon_at(p, j - 1),
			&int_polygon_at(p, j),
			&int_polygon_at(pg, i))) {
			if (int_vertex_equal(&int_polygon_at(p, j - 1), &int_polygon_at(pg, i)))
				j--;
		}
		else{
			j++;
		}
		int_polygon_copy_vertex(p, j, pg, i);
	}

	if (j < 2) {
		printf("[Clipper ERROR]: No sufficient valid vertices. \n");
		int_polygon_free(p);
		return FALSE;
	}

	len = j + 1;
	while (len > 2) {
		//nb: test for point equality before testing slopes ...
		if (int_vertex_equal(&int_polygon_at(p, j), &int_polygon_at(p, 0))) {
			j--;
		}
		else if (int_vertex_equal(&int_polygon_at(p, 0), &int_polygon_at(p, 1)) ||
			slopes_equal(&int_polygon_at(p, j), &int_polygon_at(p, 0), &int_polygon_at(p, 1))) {
			int_polygon_copy_vertex(p, 0, p, j--);
		}
		else if (slopes_equal(&int_polygon_at(p, j - 1), &int_polygon_at(p, j), &int_polygon_at(p, 0))) {
			j--;
		}
		else if (slopes_equal(&int_polygon_at(p, 0), &int_polygon_at(p, 1), &int_polygon_at(p, 2))) {
			for (i = 2; i <= j; ++i) {
				int_polygon_copy_vertex(p, i - 1, p, i);
			}
			j--;
		}
		else {
			break;
		}
		len--;
	}
	if (len < 3) {
		printf("[Clipper ERROR]: No sufficient valid vertices. \n");
		int_polygon_free(p);
		return FALSE;
	}

	//create a new edge array ...
	edges = (cledge_t *)mem_allocate(sizeof (cledge_t)* len);
	ptr_vector_append_element(clipper->m_edges, edges);

	//convert vertices to a double-linked-list of edges and initialize ...
	edges[0].xcurr = int_polygon_at(p, 0).X;
	edges[0].ycurr = int_polygon_at(p, 0).Y;

	edge_init(&edges[len - 1], &edges[0], &edges[len - 2], &int_polygon_at(p, len - 1), poly_type);

	for (i = len - 2; i > 0; --i) {
		edge_init(&edges[i], &edges[i + 1], &edges[i - 1], &int_polygon_at(p, i), poly_type);
	}

	edge_init(&edges[0], &edges[1], &edges[len - 1], &int_polygon_at(p, 0), poly_type);

	//reset xcurr & ycurr and find 'eHighest' (given the Y axis coordinates
	//increase downward so the 'highest' edge will have the smallest ytop) ...
	e = &edges[0];
	e_highest = e;

	do {
		e->xcurr = e->xbot;
		e->ycurr = e->ybot;

		if (e->ytop < e_highest->ytop) {
			e_highest = e;
		}

		e = e->next;
	} while (e != &edges[0]);

	//make sure eHighest is positioned so the following loop works safely ...
	if (e_highest->wind_delta > 0)
		e_highest = e_highest->next;

	if (NEAR_EQUAL(e_highest->dx, HORIZONTAL))
		e_highest = e_highest->next;

	//finally insert each local minima ...
	e = e_highest;
	do {
		e = add_bounds_to_lml(clipper, e);
	} while (e != e_highest);

	int_polygon_free(p);

	return TRUE;
}


bool_t
clipper_add_polygons(clipper_t *clipper, const int_polygons_t *ppg, PolyType_t poly_type)
{
	bool_t result = TRUE;
	uint32_t i;

	for (i = 0; i < ppg->len; ++i)
	if (!clipper_add_polygon(clipper, int_polygons_at(ppg, i), poly_type))
		result = FALSE;

	return result;
}


static void
dispose_outpts(out_pt_t** pp)
{
	out_pt_t *tmp_pp;
	return_if_fail(*pp);

	(*pp)->prev->next = NULL;

	while (*pp) {
		tmp_pp = *pp;
		(*pp) = (*pp)->next;
		mem_free(tmp_pp);
	}
}

static void
reverse_poly_pt_links(out_pt_t *pp)
{
	out_pt_t *pp1, *pp2;
	pp1 = pp;
	do {
		pp2 = pp1->next;
		pp1->next = pp1->prev;
		pp1->prev = pp2;
		pp1 = pp2;
	} while (pp1 != pp);
}

static void
dispose_all_poly_pts(clipper_t *clipper)
{
	uint32_t i;
	out_rec_t *out_rec;
	for (i = 0; i < clipper->poly_outs->len; i++) {
		out_rec = (out_rec_t *)ptr_vector_at(clipper->poly_outs, i);

		if (out_rec->pts) {
			dispose_outpts(&out_rec->pts);
		}
	}

	ptr_vector_clear(clipper->poly_outs, TRUE);
}

static void
insert_scanbeam(clipper_t *clipper, const int64_t Y)
{
	scanbeam_t* new_sb, *sb2;
	if (!clipper->scanbeam) {
		clipper->scanbeam = (scanbeam_t *)mem_allocate(sizeof (scanbeam_t));
		clipper->scanbeam->next = NULL;
		clipper->scanbeam->Y = Y;
	}
	else if (Y > clipper->scanbeam->Y) {
		new_sb = (scanbeam_t *)mem_allocate(sizeof (scanbeam_t));
		new_sb->Y = Y;
		new_sb->next = clipper->scanbeam;
		clipper->scanbeam = new_sb;
	}
	else {
		sb2 = clipper->scanbeam;
		while (sb2->next && (Y <= sb2->next->Y))
			sb2 = sb2->next;

		if (Y == sb2->Y)
			return; //ie ignores duplicates

		new_sb = (scanbeam_t *)mem_allocate(sizeof (scanbeam_t));
		new_sb->Y = Y;
		new_sb->next = sb2->next;
		sb2->next = new_sb;
	}
}

static void
clipper_reset(clipper_t *clipper)
{
	local_minima_t *lm;
	cledge_t *e;

	clipper->current_lm = clipper->minima_list;

	if (!clipper->minima_list)
		return; //ie nothing to process

	//reset all edges ...
	lm = clipper->minima_list;

	while (lm) {
		e = lm->leftBound;
		while (e) {
			e->xcurr = e->xbot;
			e->ycurr = e->ybot;
			e->side = esLeft;
			e->out_idx = -1;
			e = e->next_in_lml;
		}

		e = lm->rightBound;
		while (e) {
			e->xcurr = e->xbot;
			e->ycurr = e->ybot;
			e->side = esRight;
			e->out_idx = -1;
			e = e->next_in_lml;
		}

		lm = lm->next;
	}

	clipper->scanbeam = NULL;
	clipper->active_edges = NULL;
	clipper->sorted_edges = NULL;
	dispose_all_poly_pts(clipper);

	lm = clipper->minima_list;
	while (lm) {
		insert_scanbeam(clipper, lm->Y);
		insert_scanbeam(clipper, lm->leftBound->ytop);
		lm = lm->next;
	}
}

static int64_t
pop_scanbeam(clipper_t *clipper)
{
	int64_t Y = clipper->scanbeam->Y;
	scanbeam_t* sb2 = clipper->scanbeam;
	clipper->scanbeam = clipper->scanbeam->next;
	mem_free(sb2);
	return Y;
}

static bool_t
e2_inserts_before_e1(const cledge_t *e1, const cledge_t *e2)
{
	if (e2->xcurr == e1->xcurr)
		return (e2->dx) > (e1->dx);
	else
		return (e2->xcurr) < (e1->xcurr);
}

static void
insert_edge_into_ael(clipper_t *clipper, cledge_t *edge)
{
	cledge_t *e;

	edge->prev_in_ael = NULL;
	edge->next_in_ael = NULL;

	if (!clipper->active_edges) {
		clipper->active_edges = edge;
	}
	else if (e2_inserts_before_e1(clipper->active_edges, edge))
	{
		edge->next_in_ael = clipper->active_edges;
		clipper->active_edges->prev_in_ael = edge;
		clipper->active_edges = edge;
	}
	else {
		e = clipper->active_edges;
		while (e->next_in_ael && !e2_inserts_before_e1(e->next_in_ael, edge))
			e = e->next_in_ael;

		edge->next_in_ael = e->next_in_ael;

		if (e->next_in_ael)
			e->next_in_ael->prev_in_ael = edge;

		edge->prev_in_ael = e;
		e->next_in_ael = edge;
	}
}

static bool_t
is_even_odd_filltype(clipper_t *clipper, const cledge_t* edge)
{
	if (edge->poly_type == pt_subject)
		return clipper->subject_filltype == pf_even_odd;
	else
		return clipper->clip_filltype == pf_even_odd;
}

static bool_t
is_even_odd_alt_filltype(clipper_t *clipper, const cledge_t* edge)
{
	if (edge->poly_type == pt_subject)
		return clipper->clip_filltype == pf_even_odd;
	else
		return clipper->subject_filltype == pf_even_odd;
}

static void
set_winding_count(clipper_t *clipper, cledge_t *edge)
{
	cledge_t *e = edge->prev_in_ael;

	//find the edge of the same poly_type that immediately precedes 'edge' in AEL
	while (e && e->poly_type != edge->poly_type)
		e = e->prev_in_ael;

	if (!e) {
		edge->wind_cnt = edge->wind_delta;
		edge->wind_cnt2 = 0;
		e = clipper->active_edges; //ie get ready to calc windCnt2
	}
	else if (is_even_odd_filltype(clipper, edge)) {
		//EvenOdd filling ...
		edge->wind_cnt = 1;
		edge->wind_cnt2 = e->wind_cnt2;
		e = e->next_in_ael; //ie get ready to calc windCnt2
	}
	else {
		//nonZero, Positive or Negative filling ...
		if (e->wind_cnt * e->wind_delta < 0) {
			if (ABS(e->wind_cnt) > 1) {
				if (e->wind_delta * edge->wind_delta < 0)
					edge->wind_cnt = e->wind_cnt;
				else
					edge->wind_cnt = e->wind_cnt + edge->wind_delta;
			}
			else
				edge->wind_cnt = e->wind_cnt + e->wind_delta + edge->wind_delta;
		}
		else {
			if (ABS(e->wind_cnt) > 1 && e->wind_delta * edge->wind_delta < 0)
				edge->wind_cnt = e->wind_cnt;
			else if (e->wind_cnt + edge->wind_delta == 0)
				edge->wind_cnt = e->wind_cnt;
			else edge->wind_cnt = e->wind_cnt + edge->wind_delta;
		}

		edge->wind_cnt2 = e->wind_cnt2;
		e = e->next_in_ael; //ie get ready to calc windCnt2
	}

	//update windCnt2 ...
	if (is_even_odd_alt_filltype(clipper, edge)) {
		//EvenOdd filling ...
		while (e != edge) {
			edge->wind_cnt2 = (edge->wind_cnt2 == 0) ? 1 : 0;
			e = e->next_in_ael;
		}
	}
	else {
		//nonZero, Positive or Negative filling ...
		while (e != edge) {
			edge->wind_cnt2 += e->wind_delta;
			e = e->next_in_ael;
		}
	}
}

static void
add_cledge_to_sel(clipper_t *clipper, cledge_t *edge)
{
	//SEL pointers in PEdge are reused to build a list of horizontal edges.
	//However, we don't need to worry about order with horizontal edge processing.
	if (!clipper->sorted_edges) {
		clipper->sorted_edges = edge;
		edge->prev_in_sel = NULL;
		edge->next_in_sel = NULL;
	}
	else {
		edge->next_in_sel = clipper->sorted_edges;
		edge->prev_in_sel = NULL;
		clipper->sorted_edges->prev_in_sel = edge;
		clipper->sorted_edges = edge;
	}
}

bool_t
is_contributing(clipper_t *clipper, const cledge_t *edge)
{
	PolyFillType_t pft, pft2;

	if (edge->poly_type == pt_subject) {
		pft = clipper->subject_filltype;
		pft2 = clipper->clip_filltype;
	}
	else {
		pft = clipper->clip_filltype;;
		pft2 = clipper->subject_filltype;;
	}

	switch (pft) {
	case pf_even_odd:
	case pf_non_zero:
		if (ABS(edge->wind_cnt) != 1)
			return FALSE;
		break;
	case pf_positive:
		if (edge->wind_cnt != 1)
			return FALSE;
		break;
	case pf_negative:
	default: //pftNegative
		if (edge->wind_cnt != -1)
			return FALSE;
	}

	switch (clipper->clip_type) {
	case ct_intersection:
		switch (pft2) {
		case pf_even_odd:
		case pf_non_zero:
			return (edge->wind_cnt2 != 0);
		case pf_positive:
			return (edge->wind_cnt2 > 0);
		case pf_negative:
		default:
			return (edge->wind_cnt2 < 0);
		}
	case ct_union:
		switch (pft2) {
		case pf_even_odd:
		case pf_non_zero:
			return (edge->wind_cnt2 == 0);
		case pf_positive:
			return (edge->wind_cnt2 <= 0);
		case pf_negative:
		default:
			return (edge->wind_cnt2 >= 0);
		}
	case ct_difference:
		if (edge->poly_type == pt_subject) {
			switch (pft2) {
			case pf_even_odd:
			case pf_non_zero:
				return (edge->wind_cnt2 == 0);
			case pf_positive:
				return (edge->wind_cnt2 <= 0);
			case pf_negative:
			default:
				return (edge->wind_cnt2 >= 0);
			}
		}
		else {
			switch (pft2) {
			case pf_even_odd:
			case pf_non_zero:
				return (edge->wind_cnt2 != 0);
			case pf_positive:
				return (edge->wind_cnt2 > 0);
			case pf_negative:
			default:
				return (edge->wind_cnt2 < 0);
			}
		}
	case ct_xor:
	default:
		return TRUE;
	}
}

static out_rec_t*
create_out_rec()
{
	out_rec_t* result = (out_rec_t *)mem_allocate(sizeof (out_rec_t));
	result->is_hole = FALSE;
	result->first_left = 0;
	result->append_link = 0;
	result->pts = 0;
	result->bottom_pt = 0;
	result->sides = esNeither;
	result->bottom_flag = 0;

	return result;
}

static void
set_hole_state(clipper_t *clipper, cledge_t *e, out_rec_t *outRec)
{
	bool_t is_hole = FALSE;
	cledge_t *e2 = e->prev_in_ael;

	while (e2) {
		if (e2->out_idx >= 0) {
			is_hole = !is_hole;
			if (!outRec->first_left)
				outRec->first_left = (out_rec_t *)ptr_vector_at(clipper->poly_outs, e2->out_idx);
		}
		e2 = e2->prev_in_ael;
	}
	if (is_hole)
		outRec->is_hole = TRUE;
}

static void
dispose_bottom_pt(out_rec_t *out_rec)
{
	out_pt_t *next = out_rec->bottom_pt->next;
	out_pt_t *prev = out_rec->bottom_pt->prev;

	if (out_rec->pts == out_rec->bottom_pt)
		out_rec->pts = next;

	mem_free(out_rec->bottom_pt);

	next->prev = prev;
	prev->next = next;

	out_rec->bottom_pt = next;
}


static void
add_out_pt(clipper_t *clipper, cledge_t *e, const vector2d_t* pt)
{
	bool_t to_front = (e->side == esLeft);
	out_rec_t *out_rec;
	out_pt_t *op, *opBot, *op2;

	if (e->out_idx < 0) {
		out_rec = create_out_rec();
		ptr_vector_append_element(clipper->poly_outs, out_rec);
		out_rec->idx = (int32_t)clipper->poly_outs->len - 1;
		e->out_idx = out_rec->idx;
		op = (out_pt_t *)mem_allocate(sizeof (out_pt_t));
		out_rec->pts = op;
		out_rec->bottom_pt = op;
		op->pt.X = pt->X;
		op->pt.Y = pt->Y;
		op->idx = out_rec->idx;
		op->next = op;
		op->prev = op;
		set_hole_state(clipper, e, out_rec);
	}
	else {
		out_rec = (out_rec_t *)ptr_vector_at(clipper->poly_outs, e->out_idx);
		op = out_rec->pts;

		if ((to_front && int_vertex_equal(pt, &op->pt)) ||
			(!to_front && int_vertex_equal(pt, &op->prev->pt)))
			return;

		if ((e->side | out_rec->sides) != out_rec->sides) {
			//check for 'rounding' artefacts ...
			if (out_rec->sides == esNeither && pt->Y == op->pt.Y) {
				if (to_front) {
					if (pt->X == op->pt.X + 1)
						return;    //ie wrong side of bottomPt
				}
				else if (pt->X == op->pt.X - 1)
					return; //ie wrong side of bottomPt
			}

			out_rec->sides = (edge_side_t)(out_rec->sides | e->side);

			if (out_rec->sides == esBoth) {
				if (to_front) {
					opBot = out_rec->pts;
					op2 = opBot->next; //op2 == right side
					if (opBot->pt.Y != op2->pt.Y && opBot->pt.Y != pt->Y &&
						((opBot->pt.X - pt->X) / (opBot->pt.Y - pt->Y) <
						(opBot->pt.X - op2->pt.X) / (opBot->pt.Y - op2->pt.Y)))
						out_rec->bottom_flag = opBot;
				}
				else {
					opBot = out_rec->pts->prev;
					op2 = opBot->next; //op2 == left side
					if (opBot->pt.Y != op2->pt.Y && opBot->pt.Y != pt->Y &&
						((opBot->pt.X - pt->X) / (opBot->pt.Y - pt->Y) >
						(opBot->pt.X - op2->pt.X) / (opBot->pt.Y - op2->pt.Y)))
						out_rec->bottom_flag = opBot;
				}
			}
		}

		op2 = (out_pt_t *)mem_allocate(sizeof (out_pt_t));
		op2->pt = *pt;
		op2->idx = out_rec->idx;

		if (op2->pt.Y == out_rec->bottom_pt->pt.Y &&
			op2->pt.X < out_rec->bottom_pt->pt.X)
			out_rec->bottom_pt = op2;

		op2->next = op;
		op2->prev = op->prev;
		op2->prev->next = op2;
		op->prev = op2;

		if (to_front)
			out_rec->pts = op2;
	}
}

static void
clear_joins(clipper_t *clipper)
{
	ptr_vector_clear(clipper->joins, TRUE);
	ptr_vector_free(clipper->joins, TRUE);
}

void
clear_horz_joins(clipper_t *clipper)
{
	ptr_vector_clear(clipper->horz_joins, TRUE);
	ptr_vector_free(clipper->horz_joins, TRUE);
}

static void
add_horz_join(clipper_t *clipper, cledge_t *e, int32_t idx)
{
	horz_join_rec_t *hj = (horz_join_rec_t *)mem_allocate(sizeof(horz_join_rec_t));
	hj->edge = e;
	hj->saved_idx = idx;
	ptr_vector_append_element(clipper->horz_joins, hj);
}

static void
add_join(clipper_t *clipper,
const cledge_t *e1, const cledge_t *e2,
const int32_t e1_out_idx, const int32_t e2_out_idx)
{
	join_rec_t* jr = (join_rec_t *)mem_allocate(sizeof (join_rec_t));

	if (e1_out_idx >= 0)
		jr->poly1_idx = e1_out_idx;
	else
		jr->poly1_idx = e1->out_idx;

	jr->pt1a.X = e1->xcurr;
	jr->pt1a.Y = e1->ycurr;

	jr->pt1b.X = e1->xtop;
	jr->pt1b.Y = e1->ytop;

	if (e2_out_idx >= 0)
		jr->poly2_idx = e2_out_idx;
	else
		jr->poly2_idx = e2->out_idx;

	jr->pt2a.X = e2->xcurr;
	jr->pt2a.Y = e2->ycurr;

	jr->pt2b.X = e2->xtop;
	jr->pt2b.Y = e2->ytop;

	ptr_vector_append_element(clipper->joins, jr);
}

static void
add_local_min_poly(clipper_t *clipper, cledge_t *e1, cledge_t *e2, const vector2d_t *pt)
{
	cledge_t *e, *prev_e;

	if (NEAR_EQUAL(e2->dx, HORIZONTAL) || (e1->dx > e2->dx)) {
		add_out_pt(clipper, e1, pt);
		e2->out_idx = e1->out_idx;
		e1->side = esLeft;
		e2->side = esRight;
		e = e1;
		if (e->prev_in_ael == e2)
			prev_e = e2->prev_in_ael;
		else
			prev_e = e->prev_in_ael;
	}
	else {
		add_out_pt(clipper, e2, pt);
		e1->out_idx = e2->out_idx;
		e1->side = esRight;
		e2->side = esLeft;
		e = e2;
		if (e->prev_in_ael == e1)
			prev_e = e1->prev_in_ael;
		else
			prev_e = e->prev_in_ael;
	}
	if (prev_e && prev_e->out_idx >= 0 &&
		(top_X(prev_e, pt->Y) == top_X(e, pt->Y)) &&
		edge_slopes_equal(e, prev_e))
		add_join(clipper, e, prev_e, -1, -1);
}

static bool_t
first_is_bottom_pt(const out_pt_t* btm_pt1, const out_pt_t* btm_pt2)
{
	out_pt_t *p = btm_pt1->prev;
	double dx1p, dx1n, dx2p, dx2n;

	while (int_vertex_equal(&p->pt, &btm_pt1->pt) && (p != btm_pt1))
		p = p->prev;

	dx1p = ABS(get_dx(&btm_pt1->pt, &p->pt));
	p = btm_pt1->next;
	while (int_vertex_equal(&p->pt, &btm_pt1->pt) && (p != btm_pt1))
		p = p->next;

	dx1n = ABS(get_dx(&btm_pt1->pt, &p->pt));

	p = btm_pt2->prev;
	while (int_vertex_equal(&p->pt, &btm_pt2->pt) && (p != btm_pt2))
		p = p->prev;

	dx2p = ABS(get_dx(&btm_pt2->pt, &p->pt));
	p = btm_pt2->next;
	while (int_vertex_equal(&p->pt, &btm_pt2->pt) && (p != btm_pt2))
		p = p->next;

	dx2n = ABS(get_dx(&btm_pt2->pt, &p->pt));

	return (dx1p >= dx2p && dx1p >= dx2n) || (dx1n >= dx2p && dx1n >= dx2n);
}

static out_rec_t*
get_lowermost_rec(out_rec_t *out_rec1, out_rec_t *out_rec2)
{
	//work out which polygon fragment has the correct hole state ...
	out_pt_t *out_pt1 = out_rec1->bottom_pt;
	out_pt_t *out_pt2 = out_rec2->bottom_pt;

	if (out_pt1->pt.Y > out_pt2->pt.Y)
		return out_rec1;
	else if (out_pt1->pt.Y < out_pt2->pt.Y)
		return out_rec2;
	else if (out_pt1->pt.X < out_pt2->pt.X)
		return out_rec1;
	else if (out_pt1->pt.X > out_pt2->pt.X)
		return out_rec2;
	else if (out_pt1->next == out_pt1)
		return out_rec2;
	else if (out_pt2->next == out_pt2)
		return out_rec1;
	else if (first_is_bottom_pt(out_pt1, out_pt2))
		return out_rec1;
	else
		return out_rec2;
}

static void
append_polygon(clipper_t *clipper, cledge_t *e1, cledge_t *e2)
{
	//get the start and ends of both output polygons ...
	out_rec_t *out_rec1 = (out_rec_t *)ptr_vector_at(clipper->poly_outs, e1->out_idx);
	out_rec_t *out_rec2 = (out_rec_t *)ptr_vector_at(clipper->poly_outs, e2->out_idx);
	out_rec_t *hole_state_rec;
	out_pt_t *p1_lft, *p1_rt, *p2_lft, *p2_rt;
	edge_side_t side;
	int32_t ok_idx, obsolete_idx;
	cledge_t* e;
	uint32_t i;
	join_rec_t *join_i;
	horz_join_rec_t *horz_join_i;

	if (out_rec1->first_left == out_rec2)
		hole_state_rec = out_rec2;
	else if (out_rec2->first_left == out_rec1)
		hole_state_rec = out_rec1;
	else
		hole_state_rec = get_lowermost_rec(out_rec1, out_rec2);

	p1_lft = out_rec1->pts;
	p1_rt = p1_lft->prev;
	p2_lft = out_rec2->pts;
	p2_rt = p2_lft->prev;


	//join e2 poly onto e1 poly and delete pointers to e2 ...
	if (e1->side == esLeft) {
		if (e2->side == esLeft) {
			//z y x a b c
			reverse_poly_pt_links(p2_lft);
			p2_lft->next = p1_lft;
			p1_lft->prev = p2_lft;
			p1_rt->next = p2_rt;
			p2_rt->prev = p1_rt;
			out_rec1->pts = p2_rt;
		}
		else {
			//x y z a b c
			p2_rt->next = p1_lft;
			p1_lft->prev = p2_rt;
			p2_lft->prev = p1_rt;
			p1_rt->next = p2_lft;
			out_rec1->pts = p2_lft;
		}
		side = esLeft;
	}
	else {
		if (e2->side == esRight) {
			//a b c z y x
			reverse_poly_pt_links(p2_lft);
			p1_rt->next = p2_rt;
			p2_rt->prev = p1_rt;
			p2_lft->next = p1_lft;
			p1_lft->prev = p2_lft;
		}
		else {
			//a b c x y z
			p1_rt->next = p2_lft;
			p2_lft->prev = p1_rt;
			p1_lft->prev = p2_rt;
			p2_rt->next = p1_lft;
		}
		side = esRight;
	}

	if (hole_state_rec == out_rec2) {
		out_rec1->bottom_pt = out_rec2->bottom_pt;
		out_rec1->bottom_pt->idx = out_rec1->idx;

		if (out_rec2->first_left != out_rec1)
			out_rec1->first_left = out_rec2->first_left;

		out_rec1->is_hole = out_rec2->is_hole;
	}

	out_rec2->pts = NULL;
	out_rec2->bottom_pt = NULL;
	out_rec2->append_link = out_rec1;
	ok_idx = e1->out_idx;
	obsolete_idx = e2->out_idx;

	e1->out_idx = -1; //nb: safe because we only get here via AddLocalMaxPoly
	e2->out_idx = -1;

	e = clipper->active_edges;
	while (e) {
		if (e->out_idx == obsolete_idx) {
			e->out_idx = ok_idx;
			e->side = side;
			break;
		}
		e = e->next_in_ael;
	}

	for (i = 0; i < clipper->joins->len; ++i) {
		join_i = (join_rec_t *)ptr_vector_at(clipper->joins, i);

		if (join_i->poly1_idx == obsolete_idx)
			join_i->poly1_idx = ok_idx;

		if (join_i->poly2_idx == obsolete_idx)
			join_i->poly2_idx = ok_idx;
	}

	for (i = 0; i < clipper->horz_joins->len; ++i) {
		horz_join_i = (horz_join_rec_t *)ptr_vector_at(clipper->horz_joins, i);
		if (horz_join_i->saved_idx == obsolete_idx)
			horz_join_i->saved_idx = ok_idx;
	}

}

static void
do_edge1(clipper_t *clipper, cledge_t *edge1, cledge_t *edge2, const vector2d_t *pt)
{
	add_out_pt(clipper, edge1, pt);
	swap_sides(edge1, edge2);
	swap_poly_indexes(edge1, edge2);
}

static void
do_edge2(clipper_t *clipper, cledge_t *edge1, cledge_t *edge2, const vector2d_t *pt)
{
	add_out_pt(clipper, edge2, pt);
	swap_sides(edge1, edge2);
	swap_poly_indexes(edge1, edge2);
}


static void
do_both_edges(clipper_t *clipper, cledge_t *edge1, cledge_t *edge2, const vector2d_t *pt)
{
	add_out_pt(clipper, edge1, pt);
	add_out_pt(clipper, edge2, pt);
	swap_sides(edge1, edge2);
	swap_poly_indexes(edge1, edge2);
}

static void
add_local_max_poly(clipper_t *clipper, cledge_t *e1, cledge_t *e2, const vector2d_t *pt)
{
	add_out_pt(clipper, e1, pt);
	if (e1->out_idx == e2->out_idx) {
		e1->out_idx = -1;
		e2->out_idx = -1;
	}
	else
		append_polygon(clipper, e1, e2);
}

static void
delete_from_ael(clipper_t *clipper, cledge_t *e)
{
	cledge_t *ael_prev = e->prev_in_ael;
	cledge_t *ael_next = e->next_in_ael;

	if (!ael_prev &&  !ael_next && (e != clipper->active_edges))
		return; //already deleted

	if (ael_prev)
		ael_prev->next_in_ael = ael_next;
	else
		clipper->active_edges = ael_next;

	if (ael_next)
		ael_next->prev_in_ael = ael_prev;

	e->next_in_ael = NULL;
	e->prev_in_ael = NULL;
}

static void
delete_from_sel(clipper_t *clipper, cledge_t *e)
{
	cledge_t *sel_prev = e->prev_in_sel;
	cledge_t *sel_next = e->next_in_sel;

	if (!sel_prev && !sel_next && (e != clipper->sorted_edges))
		return; //already delete

	if (sel_prev)
		sel_prev->next_in_sel = sel_next;
	else
		clipper->sorted_edges = sel_next;

	if (sel_next)
		sel_next->prev_in_sel = sel_prev;

	e->next_in_sel = NULL;
	e->prev_in_sel = NULL;
}

static void
intersect_edges(clipper_t *clipper, cledge_t *e1, cledge_t *e2, const vector2d_t *pt, intersect_protects_t protects)
{
	//e1 will be to the left of e2 BELOW the intersection. Therefore e1 is before
	//e2 in AEL except when e1 is being inserted at the intersection point ...
	bool_t e1stops = !(ipLeft & protects) && !e1->next_in_lml &&
		e1->xtop == pt->X && e1->ytop == pt->Y;

	bool_t e2stops = !(ipRight & protects) && !e2->next_in_lml &&
		e2->xtop == pt->X && e2->ytop == pt->Y;

	bool_t e1_contributing = (e1->out_idx >= 0);
	bool_t e2_contributing = (e2->out_idx >= 0);
	int32_t old_E1_wind_cnt;
	PolyFillType_t e1_filltype, e2_filltype, e1_filltype2, e2_filltype2;
	int64_t e1Wc, e2Wc, e1Wc2, e2Wc2;;

	//update winding counts...
	//assumes that e1 will be to the right of e2 ABOVE the intersection
	if (e1->poly_type == e2->poly_type) {
		if (is_even_odd_filltype(clipper, e1)) {
			old_E1_wind_cnt = e1->wind_cnt;
			e1->wind_cnt = e2->wind_cnt;
			e2->wind_cnt = old_E1_wind_cnt;
		}
		else {
			if (e1->wind_cnt + e2->wind_delta == 0)
				e1->wind_cnt = -e1->wind_cnt;
			else
				e1->wind_cnt += e2->wind_delta;

			if (e2->wind_cnt - e1->wind_delta == 0)
				e2->wind_cnt = -e2->wind_cnt;
			else
				e2->wind_cnt -= e1->wind_delta;
		}
	}
	else {
		if (!is_even_odd_filltype(clipper, e2))
			e1->wind_cnt2 += e2->wind_delta;
		else
			e1->wind_cnt2 = (e1->wind_cnt2 == 0) ? 1 : 0;

		if (!is_even_odd_filltype(clipper, e1))
			e2->wind_cnt2 -= e1->wind_delta;
		else
			e2->wind_cnt2 = (e2->wind_cnt2 == 0) ? 1 : 0;
	}


	if (e1->poly_type == pt_subject) {
		e1_filltype = clipper->subject_filltype;
		e1_filltype2 = clipper->clip_filltype;
	}
	else {
		e1_filltype = clipper->clip_filltype;
		e1_filltype2 = clipper->subject_filltype;
	}

	if (e2->poly_type == pt_subject) {
		e2_filltype = clipper->subject_filltype;
		e2_filltype2 = clipper->clip_filltype;
	}
	else {
		e2_filltype = clipper->clip_filltype;
		e2_filltype2 = clipper->subject_filltype;
	}

	switch (e1_filltype) {
	case pf_positive:
		e1Wc = e1->wind_cnt;
		break;
	case pf_negative:
		e1Wc = -e1->wind_cnt;
		break;
	case pf_non_zero:
	case pf_even_odd:
	default:
		e1Wc = ABS(e1->wind_cnt);
	}

	switch (e2_filltype) {
	case pf_positive:
		e2Wc = e2->wind_cnt;
		break;
	case pf_negative:
		e2Wc = -e2->wind_cnt;
		break;
	case pf_non_zero:
	case pf_even_odd:
	default:
		e2Wc = ABS(e2->wind_cnt);
	}

	if (e1_contributing && e2_contributing) {
		if (e1stops || e2stops ||
			(e1Wc != 0 && e1Wc != 1) || (e2Wc != 0 && e2Wc != 1) ||
			(e1->poly_type != e2->poly_type && clipper->clip_type != ct_xor))
			add_local_max_poly(clipper, e1, e2, pt);
		else
			do_both_edges(clipper, e1, e2, pt);
	}
	else if (e1_contributing) {
		if ((e2Wc == 0 || e2Wc == 1) &&
			(clipper->clip_type != ct_intersection ||
			e2->poly_type == pt_subject || (e2->wind_cnt2 != 0)))
			do_edge1(clipper, e1, e2, pt);
	}
	else if (e2_contributing) {
		if ((e1Wc == 0 || e1Wc == 1) &&
			(clipper->clip_type != ct_intersection ||
			e1->poly_type == pt_subject || (e1->wind_cnt2 != 0)))
			do_edge2(clipper, e1, e2, pt);
	}
	else if ((e1Wc == 0 || e1Wc == 1) &&
		(e2Wc == 0 || e2Wc == 1) && !e1stops && !e2stops) {
		//neither edge is currently contributing ...
		switch (e1_filltype2) {
		case pf_positive:
			e1Wc2 = e1->wind_cnt2;
			break;
		case pf_negative:
			e1Wc2 = -e1->wind_cnt2;
			break;
		case pf_non_zero:
		case pf_even_odd:
		default:
			e1Wc2 = ABS(e1->wind_cnt2);
		}

		switch (e2_filltype2) {
		case pf_positive:
			e2Wc2 = e2->wind_cnt2;
			break;
		case pf_negative:
			e2Wc2 = -e2->wind_cnt2;
			break;
		case pf_non_zero:
		case pf_even_odd:
		default:
			e2Wc2 = ABS(e2->wind_cnt2);
		}

		if (e1->poly_type != e2->poly_type)
			add_local_min_poly(clipper, e1, e2, pt);
		else if (e1Wc == 1 && e2Wc == 1)
			switch (clipper->clip_type) {
			case ct_intersection:
				if (e1Wc2 > 0 && e2Wc2 > 0)
					add_local_min_poly(clipper, e1, e2, pt);
				break;
			case ct_union:
				if (e1Wc2 <= 0 && e2Wc2 <= 0)
					add_local_min_poly(clipper, e1, e2, pt);
				break;
			case ct_difference:
				if (((e1->poly_type == pt_clip) && (e1Wc2 > 0) && (e2Wc2 > 0)) ||
					((e1->poly_type == pt_subject) && (e1Wc2 <= 0) && (e2Wc2 <= 0)))
					add_local_min_poly(clipper, e1, e2, pt);
				break;
			case ct_xor:
				add_local_min_poly(clipper, e1, e2, pt);
		}
		else
			swap_sides(e1, e2);
	}

	if ((e1stops != e2stops) &&
		((e1stops && (e1->out_idx >= 0)) || (e2stops && (e2->out_idx >= 0))))
	{
		swap_sides(e1, e2);
		swap_poly_indexes(e1, e2);
	}

	//finally, delete any non-contributing maxima edges  ...
	if (e1stops)
		delete_from_ael(clipper, e1);

	if (e2stops)
		delete_from_ael(clipper, e2);
}

static void
insert_localminima_into_ael(clipper_t *clipper, const int64_t botY)
{
	cledge_t *lb, *rb, *e;
	vector2d_t int_pt1, int_pt2, int_pt3, int_pt4, pt, pt2;
	uint32_t i;
	horz_join_rec_t *hj;

	while (clipper->current_lm && (clipper->current_lm->Y == botY)) {
		lb = clipper->current_lm->leftBound;
		rb = clipper->current_lm->rightBound;

		insert_edge_into_ael(clipper, lb);
		insert_scanbeam(clipper, lb->ytop);
		insert_edge_into_ael(clipper, rb);

		if (is_even_odd_filltype(clipper, lb)) {
			lb->wind_delta = 1;
			rb->wind_delta = 1;
		}
		else {
			rb->wind_delta = -lb->wind_delta;
		}

		set_winding_count(clipper, lb);
		rb->wind_cnt = lb->wind_cnt;
		rb->wind_cnt2 = lb->wind_cnt2;

		if (NEAR_EQUAL(rb->dx, HORIZONTAL)) {
			//nb: only rightbounds can have a horizontal bottom edge
			add_cledge_to_sel(clipper, rb);
			insert_scanbeam(clipper, rb->next_in_lml->ytop);
		}
		else
			insert_scanbeam(clipper, rb->ytop);

		if (is_contributing(clipper, lb)) {
			int_pt1.X = lb->xcurr;
			int_pt1.Y = clipper->current_lm->Y;
			add_local_min_poly(clipper, lb, rb, &int_pt1);
		}

		//if any output polygons share an edge, they'll need joining later ...
		if (rb->out_idx >= 0) {
			if (NEAR_EQUAL(rb->dx, HORIZONTAL)) {
				for (i = 0; i < clipper->horz_joins->len; ++i) {
					hj = (horz_join_rec_t *)ptr_vector_at(clipper->horz_joins, i);
					int_pt1.X = hj->edge->xbot;
					int_pt1.Y = hj->edge->ybot;

					int_pt2.X = hj->edge->xtop;
					int_pt2.Y = hj->edge->ytop;

					int_pt3.X = rb->xbot;
					int_pt3.Y = rb->ybot;

					int_pt4.X = rb->xtop;
					int_pt4.Y = rb->ytop;

					//if horizontals rb and hj.edge overlap, flag for joining later ...
					if (get_overlap_segment(int_pt1, int_pt2, int_pt3, int_pt4, &pt, &pt2))
						add_join(clipper, hj->edge, rb, hj->saved_idx, -1);
				}
			}
		}

		if (lb->next_in_ael != rb) {
			if (rb->out_idx >= 0 && rb->prev_in_ael->out_idx >= 0 && edge_slopes_equal(rb->prev_in_ael, rb))
				add_join(clipper, rb, rb->prev_in_ael, -1, -1);

			e = lb->next_in_ael;
			pt.X = lb->xcurr;
			pt.Y = lb->ycurr;

			while (e != rb) {
				//nb: For calculating winding counts etc, IntersectEdges() assumes
				//that param1 will be to the right of param2 ABOVE the intersection ...
				intersect_edges(clipper, rb, e, &pt, ipNone); //order important here
				e = e->next_in_ael;
			}
		}
		pop_local_minima(clipper);
	}
}

static bool_t
is_top_horz(clipper_t *clipper, const int64_t X_pos)
{
	cledge_t *e = clipper->sorted_edges;
	while (e) {
		if ((X_pos >= MIN(e->xcurr, e->xtop)) &&
			(X_pos <= MAX(e->xcurr, e->xtop)))
			return FALSE;

		e = e->next_in_sel;
	}
	return TRUE;
}


static bool_t
is_minima(cledge_t *e)
{
	return e && (e->prev->next_in_lml != e) && (e->next->next_in_lml != e);
}


static bool_t
is_maxima(cledge_t *e, const int64_t Y)
{
	return e && e->ytop == Y && !e->next_in_lml;
}


static bool_t
is_intermediate(cledge_t *e, const int64_t Y)
{
	return e->ytop == Y && e->next_in_lml;
}


static cledge_t*
get_maxima_pair(cledge_t *e)
{
	if (!is_maxima(e->next, e->ytop) || e->next->xtop != e->xtop)
		return e->prev;
	else
		return e->next;
}


static void
swap_positions_in_ael(clipper_t *clipper, cledge_t *edge1, cledge_t *edge2)
{
	cledge_t *next, *prev;

	if (!edge1->next_in_ael &&  !edge1->prev_in_ael)
		return;
	if (!edge2->next_in_ael &&  !edge2->prev_in_ael)
		return;

	if (edge1->next_in_ael == edge2) {
		next = edge2->next_in_ael;
		if (next)
			next->prev_in_ael = edge1;

		prev = edge1->prev_in_ael;
		if (prev)
			prev->next_in_ael = edge2;

		edge2->prev_in_ael = prev;
		edge2->next_in_ael = edge1;
		edge1->prev_in_ael = edge2;
		edge1->next_in_ael = next;
	}
	else if (edge2->next_in_ael == edge1) {
		next = edge1->next_in_ael;
		if (next)
			next->prev_in_ael = edge2;

		prev = edge2->prev_in_ael;
		if (prev)
			prev->next_in_ael = edge1;

		edge1->prev_in_ael = prev;
		edge1->next_in_ael = edge2;
		edge2->prev_in_ael = edge1;
		edge2->next_in_ael = next;
	}
	else {
		next = edge1->next_in_ael;
		prev = edge1->prev_in_ael;
		edge1->next_in_ael = edge2->next_in_ael;
		if (edge1->next_in_ael)
			edge1->next_in_ael->prev_in_ael = edge1;

		edge1->prev_in_ael = edge2->prev_in_ael;
		if (edge1->prev_in_ael)
			edge1->prev_in_ael->next_in_ael = edge1;

		edge2->next_in_ael = next;
		if (edge2->next_in_ael)
			edge2->next_in_ael->prev_in_ael = edge2;

		edge2->prev_in_ael = prev;
		if (edge2->prev_in_ael)
			edge2->prev_in_ael->next_in_ael = edge2;
	}

	if (!edge1->prev_in_ael)
		clipper->active_edges = edge1;
	else if (!edge2->prev_in_ael)
		clipper->active_edges = edge2;
}


static void
swap_positions_in_sel(clipper_t *clipper, cledge_t *edge1, cledge_t *edge2)
{
	cledge_t *next, *prev;

	if (!(edge1->next_in_sel) && !(edge1->prev_in_sel))
		return;
	if (!(edge2->next_in_sel) && !(edge2->prev_in_sel))
		return;

	if (edge1->next_in_sel == edge2) {
		next = edge2->next_in_sel;
		if (next)
			next->prev_in_sel = edge1;

		prev = edge1->prev_in_sel;

		if (prev)
			prev->next_in_sel = edge2;

		edge2->prev_in_sel = prev;
		edge2->next_in_sel = edge1;
		edge1->prev_in_sel = edge2;
		edge1->next_in_sel = next;
	}
	else if (edge2->next_in_sel == edge1) {
		next = edge1->next_in_sel;
		if (next)
			next->prev_in_sel = edge2;

		prev = edge2->prev_in_sel;
		if (prev)
			prev->next_in_sel = edge1;

		edge1->prev_in_sel = prev;
		edge1->next_in_sel = edge2;
		edge2->prev_in_sel = edge1;
		edge2->next_in_sel = next;
	}
	else {
		next = edge1->next_in_sel;
		prev = edge1->prev_in_sel;
		edge1->next_in_sel = edge2->next_in_sel;

		if (edge1->next_in_sel)
			edge1->next_in_sel->prev_in_sel = edge1;

		edge1->prev_in_sel = edge2->prev_in_sel;
		if (edge1->prev_in_sel)
			edge1->prev_in_sel->next_in_sel = edge1;

		edge2->next_in_sel = next;
		if (edge2->next_in_sel)
			edge2->next_in_sel->prev_in_sel = edge2;

		edge2->prev_in_sel = prev;
		if (edge2->prev_in_sel)
			edge2->prev_in_sel->next_in_sel = edge2;
	}

	if (!edge1->prev_in_sel)
		clipper->sorted_edges = edge1;
	else if (!edge2->prev_in_sel)
		clipper->sorted_edges = edge2;
}


static cledge_t*
get_next_in_ael(cledge_t *e, direction_t dir)
{
	if (dir == dLeftToRight)
		return e->next_in_ael;
	else
		return e->prev_in_ael;
}

static void
update_edge_into_ael(clipper_t *clipper, cledge_t **e)
{
	cledge_t* ael_prev, *ael_next;

	if (!(*e)->next_in_lml) {
		printf("[Clipper Error]: update_edge_into_ael.\n");
	}
	ael_prev = (*e)->prev_in_ael;
	ael_next = (*e)->next_in_ael;
	(*e)->next_in_lml->out_idx = (*e)->out_idx;
	if (ael_prev)
		ael_prev->next_in_ael = (*e)->next_in_lml;
	else
		clipper->active_edges = (*e)->next_in_lml;

	if (ael_next)
		ael_next->prev_in_ael = (*e)->next_in_lml;

	(*e)->next_in_lml->side = (*e)->side;
	(*e)->next_in_lml->wind_delta = (*e)->wind_delta;
	(*e)->next_in_lml->wind_cnt = (*e)->wind_cnt;
	(*e)->next_in_lml->wind_cnt2 = (*e)->wind_cnt2;
	(*e) = (*e)->next_in_lml;
	(*e)->prev_in_ael = ael_prev;
	(*e)->next_in_ael = ael_next;

	if (!NEAR_EQUAL((*e)->dx, HORIZONTAL))
		insert_scanbeam(clipper, (*e)->ytop);
}

static void
process_horizontal(clipper_t *clipper, cledge_t *horz_edge)
{
	direction_t dir;
	cledge_t *e_max_pair, *e, *e_next;
	int64_t horz_left, horz_right;
	vector2d_t pt;

	if (horz_edge->xcurr < horz_edge->xtop) {
		horz_left = horz_edge->xcurr;
		horz_right = horz_edge->xtop;
		dir = dLeftToRight;
	}
	else {
		horz_left = horz_edge->xtop;
		horz_right = horz_edge->xcurr;
		dir = dRightToLeft;
	}

	if (horz_edge->next_in_lml)
		e_max_pair = NULL;
	else
		e_max_pair = get_maxima_pair(horz_edge);

	e = get_next_in_ael(horz_edge, dir);
	while (e) {
		e_next = get_next_in_ael(e, dir);

		if (e_max_pair ||
			((dir == dLeftToRight) && (e->xcurr <= horz_right)) ||
			((dir == dRightToLeft) && (e->xcurr >= horz_left))) {
			//ok, so far it looks like we're still in range of the horizontal edge
			if (e->xcurr == horz_edge->xtop && !e_max_pair) {
				if (edge_slopes_equal(e, horz_edge->next_in_lml)) {
					//if output polygons share an edge, they'll need joining later ...
					if (horz_edge->out_idx >= 0 && e->out_idx >= 0)
						add_join(clipper, horz_edge->next_in_lml, e, horz_edge->out_idx, -1);
					break; //we've reached the end of the horizontal line
				}
				else if (e->dx < horz_edge->next_in_lml->dx)
					//we really have got to the end of the intermediate horz edge so quit.
					//nb: More -ve slopes follow more +ve slopes ABOVE the horizontal.
					break;
			}

			if (e == e_max_pair) {
				//horzEdge is evidently a maxima horizontal and we've arrived at its end.
				if (dir == dLeftToRight) {
					pt.X = e->xcurr;
					pt.Y = horz_edge->ycurr;
					intersect_edges(clipper, horz_edge, e, &pt, ipNone);
				}
				else {
					pt.X = e->xcurr;
					pt.Y = horz_edge->ycurr;
					intersect_edges(clipper, e, horz_edge, &pt, ipNone);
				}

				if (e_max_pair->out_idx >= 0)
					printf("[Clipper Error]: ProcessHorizontal error\n");

				return;
			}
			else if (NEAR_EQUAL(e->dx, HORIZONTAL) && !is_minima(e) && !(e->xcurr > e->xtop)) {
				//An overlapping horizontal edge. Overlapping horizontal edges are
				//processed as if layered with the current horizontal edge (horizEdge)
				//being infinitesimally lower that the next (e). Therfore, we
				//intersect with e only if e.xcurr is within the bounds of horzEdge ...
				if (dir == dLeftToRight) {
					pt.X = e->xcurr;
					pt.Y = horz_edge->ycurr;
					intersect_edges(clipper, horz_edge, e, &pt, (is_top_horz(clipper, e->xcurr)) ? ipLeft : ipBoth);
				}
				else {
					pt.X = e->xcurr;
					pt.Y = horz_edge->ycurr;
					intersect_edges(clipper, e, horz_edge, &pt, (is_top_horz(clipper, e->xcurr)) ? ipRight : ipBoth);
				}
			}
			else if (dir == dLeftToRight) {
				pt.X = e->xcurr;
				pt.Y = horz_edge->ycurr;
				intersect_edges(clipper, horz_edge, e, &pt, (is_top_horz(clipper, e->xcurr)) ? ipLeft : ipBoth);
			}
			else {
				pt.X = e->xcurr;
				pt.Y = horz_edge->ycurr;
				intersect_edges(clipper, e, horz_edge, &pt, (is_top_horz(clipper, e->xcurr)) ? ipRight : ipBoth);
			}
			swap_positions_in_ael(clipper, horz_edge, e);
		}
		else if ((dir == dLeftToRight && e->xcurr > horz_right  && clipper->sorted_edges) ||
			(dir == dRightToLeft && e->xcurr < horz_left && clipper->sorted_edges))
			break;
		e = e_next;
	} //end while

	if (horz_edge->next_in_lml) {
		if (horz_edge->out_idx >= 0) {
			pt.X = horz_edge->xtop;
			pt.Y = horz_edge->ytop;
			add_out_pt(clipper, horz_edge, &pt);
		}
		update_edge_into_ael(clipper, &horz_edge);
	}
	else {
		if (horz_edge->out_idx >= 0) {
			pt.X = horz_edge->xtop;
			pt.Y = horz_edge->ycurr;
			intersect_edges(clipper, horz_edge, e_max_pair, &pt, ipBoth);
		}

		if (e_max_pair->out_idx >= 0)
			printf("[Clipper Error]: ProcessHorizontal error\n");

		delete_from_ael(clipper, e_max_pair);
		delete_from_ael(clipper, horz_edge);
	}
}

static void
process_horizontals(clipper_t *clipper)
{
	cledge_t *horz_edge = clipper->sorted_edges;
	while (horz_edge) {
		delete_from_sel(clipper, horz_edge);
		process_horizontal(clipper, horz_edge);
		horz_edge = clipper->sorted_edges;
	}
}

static bool_t
process1_before2(intersect_node_t *node1, intersect_node_t *node2)
{
	bool_t result;
	if (node1->pt.Y == node2->pt.Y) {
		if (node1->edge1 == node2->edge1 || node1->edge2 == node2->edge1) {
			result = node2->pt.X > node1->pt.X;
			if (node2->edge1->dx > 0)
				return !result;
			else
				return result;
		}
		else if (node1->edge1 == node2->edge2 || node1->edge2 == node2->edge2) {
			result = node2->pt.X > node1->pt.X;
			if (node2->edge2->dx > 0)
				return !result;
			else
				return result;
		}
		else
			return node2->pt.X > node1->pt.X;
	}
	else
		return node1->pt.Y > node2->pt.Y;
}


static void
add_intersect_node(clipper_t *clipper, cledge_t *e1, cledge_t *e2, const vector2d_t *pt)
{
	intersect_node_t* new_node = (intersect_node_t *)mem_allocate(sizeof (intersect_node_t));
	intersect_node_t* i_node;
	new_node->edge1 = e1;
	new_node->edge2 = e2;
	new_node->pt = *pt;
	new_node->next = NULL;

	if (!clipper->intersect_nodes)
		clipper->intersect_nodes = new_node;
	else if (process1_before2(new_node, clipper->intersect_nodes)) {
		new_node->next = clipper->intersect_nodes;
		clipper->intersect_nodes = new_node;
	}
	else {
		i_node = clipper->intersect_nodes;
		while (i_node->next  && process1_before2(i_node->next, new_node))
			i_node = i_node->next;

		new_node->next = i_node->next;
		i_node->next = new_node;
	}
}

static void
build_intersect_list(clipper_t *clipper, const int64_t botY, const int64_t topY)
{
	cledge_t *e, *e_next;
	vector2d_t pt;
	bool_t is_modified = TRUE;
	if (!clipper->active_edges)
		return;

	//prepare for sorting ...
	e = clipper->active_edges;
	e->tmp_x = top_X(e, topY);
	clipper->sorted_edges = e;
	clipper->sorted_edges->prev_in_sel = NULL;
	e = e->next_in_ael;
	while (e) {
		e->prev_in_sel = e->prev_in_ael;
		e->prev_in_sel->next_in_sel = e;
		e->next_in_sel = NULL;
		e->tmp_x = top_X(e, topY);
		e = e->next_in_ael;
	}

	//bubblesort ...
	while (is_modified && clipper->sorted_edges) {
		is_modified = FALSE;
		e = clipper->sorted_edges;
		while (e->next_in_sel) {
			e_next = e->next_in_sel;

			if (e->tmp_x > e_next->tmp_x &&
				intersect_point(e, e_next, &pt)) {

				if (pt.Y > botY) {
					pt.Y = botY;
					pt.X = top_X(e, pt.Y);
				}
				add_intersect_node(clipper, e, e_next, &pt);
				swap_positions_in_sel(clipper, e, e_next);
				is_modified = TRUE;
			}
			else
				e = e_next;
		}
		if (e->prev_in_sel) e->prev_in_sel->next_in_sel = NULL;
		else break;
	}
	clipper->sorted_edges = NULL;
}

static void
copy_ael_to_sel(clipper_t *clipper)
{
	cledge_t *e = clipper->active_edges;
	clipper->sorted_edges = e;

	if (!clipper->active_edges)
		return;

	clipper->sorted_edges->prev_in_sel = NULL;
	e = e->next_in_ael;

	while (e) {
		e->prev_in_sel = e->prev_in_ael;
		e->prev_in_sel->next_in_sel = e;
		e->next_in_sel = NULL;
		e = e->next_in_ael;
	}
}

static void
swap_intersect_nodes(intersect_node_t *int1, intersect_node_t *int2)
{
	cledge_t *e1 = int1->edge1;
	cledge_t *e2 = int1->edge2;
	vector2d_t p = int1->pt;

	int1->edge1 = int2->edge1;
	int1->edge2 = int2->edge2;
	int1->pt = int2->pt;

	int2->edge1 = e1;
	int2->edge2 = e2;
	int2->pt = p;
}

static bool_t
fixup_intersections(clipper_t *clipper)
{
	intersect_node_t *int1, *int2;
	cledge_t *e1, *e2;
	if (!clipper->intersect_nodes->next)
		return TRUE;

	copy_ael_to_sel(clipper);
	int1 = clipper->intersect_nodes;
	int2 = clipper->intersect_nodes->next;

	while (int2) {
		e1 = int1->edge1;
		if (e1->prev_in_sel == int1->edge2)
			e2 = e1->prev_in_sel;
		else if (e1->next_in_sel == int1->edge2)
			e2 = e1->next_in_sel;
		else {
			//The current intersection is out of order, so try and swap it with
			//a subsequent intersection ...
			while (int2) {
				if (int2->edge1->next_in_sel == int2->edge2 ||
					int2->edge1->prev_in_sel == int2->edge2)
					break;
				else int2 = int2->next;
			}
			if (!int2)
				return FALSE; //oops!!!

			//found an intersect node that can be swapped ...
			swap_intersect_nodes(int1, int2);
			e1 = int1->edge1;
			e2 = int1->edge2;
		}
		swap_positions_in_sel(clipper, e1, e2);
		int1 = int1->next;
		int2 = int1->next;
	}

	clipper->sorted_edges = NULL;

	//finally, check the last intersection too ...
	return (int1->edge1->prev_in_sel == int1->edge2 ||
		int1->edge1->next_in_sel == int1->edge2);
}

static void
process_intersect_list(clipper_t *clipper)
{
	intersect_node_t *i_node;
	while (clipper->intersect_nodes) {
		i_node = clipper->intersect_nodes->next;

		intersect_edges(clipper, clipper->intersect_nodes->edge1,
			clipper->intersect_nodes->edge2, &clipper->intersect_nodes->pt, ipBoth);

		swap_positions_in_ael(clipper, clipper->intersect_nodes->edge1, clipper->intersect_nodes->edge2);

		mem_free(clipper->intersect_nodes);

		clipper->intersect_nodes = i_node;
	}
}

static bool_t
process_intersections(clipper_t *clipper, const int64_t botY, const int64_t topY)
{
	if (!clipper->active_edges)
		return TRUE;

	build_intersect_list(clipper, botY, topY);

	if (!clipper->intersect_nodes)
		return TRUE;

	if (fixup_intersections(clipper))
		process_intersect_list(clipper);
	else
		return FALSE;

	return TRUE;
}

static void
do_maxima(clipper_t *clipper, cledge_t *e, int64_t topY)
{
	cledge_t *e_max_pair = get_maxima_pair(e);
	int64_t X = e->xtop;
	cledge_t *e_next = e->next_in_ael;
	vector2d_t pt;
	while (e_next != e_max_pair) {
		if (!e_next){
			printf("[Clipper Error]: do_maxima error.\n");
			return;
		}
		pt.X = X;
		pt.Y = topY;
		intersect_edges(clipper, e, e_next, &pt, ipBoth);
		e_next = e_next->next_in_ael;
	}

	if (e->out_idx < 0 && e_max_pair->out_idx < 0) {
		delete_from_ael(clipper, e);
		delete_from_ael(clipper, e_max_pair);
	}
	else if (e->out_idx >= 0 && e_max_pair->out_idx >= 0) {
		pt.X = X;
		pt.Y = topY;
		intersect_edges(clipper, e, e_max_pair, &pt, ipNone);
	}
	else
		printf("[Clipper Error]: do_maxima error\n");
}


static void
process_edges_at_topof_scanbeam(clipper_t *clipper, const int64_t topY)
{
	cledge_t *e = clipper->active_edges;
	cledge_t *e_prior;
	vector2d_t pt, pt2, int_pt1, int_pt2, int_pt3, int_pt4;
	uint32_t i;
	horz_join_rec_t *hj;

	while (e) {
		//1. process maxima, treating them as if they're 'bent' horizontal edges,
		//   but exclude maxima with horizontal edges. nb: e can't be a horizontal.
		if (is_maxima(e, topY) && !NEAR_EQUAL(get_maxima_pair(e)->dx, HORIZONTAL)) {
			//'e' might be removed from AEL, as may any following edges so ...
			e_prior = e->prev_in_ael;
			do_maxima(clipper, e, topY);
			if (!e_prior)
				e = clipper->active_edges;
			else
				e = e_prior->next_in_ael;
		}
		else
		{
			//2. promote horizontal edges, otherwise update xcurr and ycurr ...
			if (is_intermediate(e, topY) && NEAR_EQUAL(e->next_in_lml->dx, HORIZONTAL)) {
				if (e->out_idx >= 0) {
					int_pt1.X = e->xtop;
					int_pt1.Y = e->ytop;
					add_out_pt(clipper, e, &int_pt1);

					for (i = 0; i < clipper->horz_joins->len; ++i) {
						hj = (horz_join_rec_t *)ptr_vector_at(clipper->horz_joins, i);
						int_pt1.X = hj->edge->xbot;
						int_pt1.Y = hj->edge->ybot;

						int_pt2.X = hj->edge->xtop;
						int_pt2.Y = hj->edge->ytop;

						int_pt3.X = e->next_in_lml->xbot;
						int_pt3.Y = e->next_in_lml->ybot;

						int_pt4.X = e->next_in_lml->xtop;
						int_pt4.Y = e->next_in_lml->ytop;

						if (get_overlap_segment(int_pt1, int_pt2, int_pt3, int_pt4, &pt, &pt2))
							add_join(clipper, hj->edge, e->next_in_lml, hj->saved_idx, e->out_idx);
					}

					add_horz_join(clipper, e->next_in_lml, e->out_idx);
				}
				update_edge_into_ael(clipper, &e);
				add_cledge_to_sel(clipper, e);
			}
			else {
				//this just simplifies horizontal processing ...
				e->xcurr = top_X(e, topY);
				e->ycurr = topY;
			}
			e = e->next_in_ael;
		}
	}

	//3. Process horizontals at the top of the scanbeam ...
	process_horizontals(clipper);

	//4. Promote intermediate vertices ...
	e = clipper->active_edges;
	while (e) {
		if (is_intermediate(e, topY)) {
			if (e->out_idx >= 0) {
				int_pt1.X = e->xtop;
				int_pt1.Y = e->ytop;
				add_out_pt(clipper, e, &int_pt1);
			}
			update_edge_into_ael(clipper, &e);


			//if output polygons share an edge, they'll need joining later ...
			if (e->out_idx >= 0 && e->prev_in_ael && e->prev_in_ael->out_idx >= 0 &&
				e->prev_in_ael->xcurr == e->xbot && e->prev_in_ael->ycurr == e->ybot) {
				int_pt1.X = e->xbot;
				int_pt1.Y = e->ybot;

				int_pt2.X = e->xtop;
				int_pt2.Y = e->ytop;

				int_pt3.X = e->prev_in_ael->xtop;
				int_pt3.Y = e->prev_in_ael->ytop;

				if (points_slopes_equal(&int_pt1, &int_pt2, &int_pt1, &int_pt3)) {
					add_out_pt(clipper, e->prev_in_ael, &int_pt1);
					add_join(clipper, e, e->prev_in_ael, -1, -1);
				}
			}
			else if (e->out_idx >= 0 && e->next_in_ael && e->next_in_ael->out_idx >= 0 &&
				e->next_in_ael->ycurr > e->next_in_ael->ytop &&
				e->next_in_ael->ycurr <= e->next_in_ael->ybot &&
				e->next_in_ael->xcurr == e->xbot && e->next_in_ael->ycurr == e->ybot) {

				int_pt1.X = e->xbot;
				int_pt1.Y = e->ybot;

				int_pt2.X = e->xtop;
				int_pt2.Y = e->ytop;

				int_pt4.X = e->next_in_ael->xtop;
				int_pt4.Y = e->next_in_ael->ytop;

				if (points_slopes_equal(&int_pt1, &int_pt2, &int_pt1, &int_pt4)) {
					add_out_pt(clipper, e->next_in_ael, &int_pt1);
					add_join(clipper, e, e->next_in_ael, -1, -1);
				}
			}
		}
		e = e->next_in_ael;
	}
}

static out_pt_t*
polygon_bottom(out_pt_t* pp)
{
	out_pt_t *p = pp->next;
	out_pt_t *result = pp;
	while (p != pp) {
		if (p->pt.Y > result->pt.Y)
			result = p;
		else if (p->pt.Y == result->pt.Y && p->pt.X < result->pt.X)
			result = p;
		p = p->next;
	}
	return result;
}

static bool_t
find_segment(out_pt_t **pp, vector2d_t *pt1, vector2d_t *pt2)
{
	//outPt1 & outPt2 => the overlap segment (if the function returns true)
	out_pt_t *pp2;
	vector2d_t pt1a, pt2a;
	bool_t a, b, c;

	if (!*pp)
		return FALSE;

	pp2 = (*pp);
	pt1a = *pt1;
	pt2a = *pt2;

	do {
		a = points_slopes_equal(&pt1a, &pt2a, &(*pp)->pt, &(*pp)->prev->pt);
		b = slopes_equal(&pt1a, &pt2a, &(*pp)->pt);
		c = get_overlap_segment(pt1a, pt2a, (*pp)->pt, (*pp)->prev->pt, pt1, pt2);

		if (a && b && c)
			return TRUE;

		(*pp) = (*pp)->next;
	} while ((*pp) != pp2);

	return FALSE;
}

static bool_t
pt3_is_between_pt1_pt2(const vector2d_t *pt1,
const vector2d_t *pt2, const vector2d_t *pt3)
{
	if (int_vertex_equal(pt1, pt3) || int_vertex_equal(pt2, pt3))
		return TRUE;
	else if (pt1->X != pt2->X)
		return (pt1->X < pt3->X) == (pt3->X < pt2->X);
	else
		return (pt1->Y < pt3->Y) == (pt3->Y < pt2->Y);
}

out_pt_t*
insert_poly_pt_between(out_pt_t* p1, out_pt_t* p2, const vector2d_t *pt)
{
	out_pt_t *result;

	if (p1 == p2) {
		printf("[Clipper Error]: JoinError.\n");
		return NULL;
	}

	result = (out_pt_t *)mem_allocate(sizeof (out_pt_t));
	result->pt = *pt;

	if (p2 == p1->next) {
		p1->next = result;
		p2->prev = result;
		result->next = p2;
		result->prev = p1;
	}
	else {
		p2->next = result;
		p1->prev = result;
		result->next = p1;
		result->prev = p2;
	}
	return result;
}

static void
fixup_out_polygon(out_rec_t *out_rec)
{
	//FixupOutPolygon() - removes duplicate points and simplifies consecutive
	//parallel edges by removing the middle vertex.
	out_pt_t *last_ok = NULL;
	out_pt_t *pp, *tmp;

	out_rec->pts = out_rec->bottom_pt;
	pp = out_rec->bottom_pt;

	while (1) {
		if (pp->prev == pp || pp->prev == pp->next){
			dispose_outpts(&pp);
			out_rec->pts = NULL;
			out_rec->bottom_pt = NULL;
			return;
		}
		//test for duplicate points and for same slope (cross-product) ...
		if (int_vertex_equal(&pp->pt, &pp->next->pt) ||
			slopes_equal(&pp->prev->pt, &pp->pt, &pp->next->pt))
		{
			last_ok = NULL;
			tmp = pp;
			if (pp == out_rec->bottom_pt)
				out_rec->bottom_pt = NULL; //flags need for updating

			pp->prev->next = pp->next;
			pp->next->prev = pp->prev;
			pp = pp->prev;
			mem_free(tmp);
		}
		else if (pp == last_ok)
			break;
		else {
			if (!last_ok)
				last_ok = pp;
			pp = pp->next;
		}
	}

	if (!out_rec->bottom_pt) {
		out_rec->bottom_pt = polygon_bottom(pp);
		out_rec->pts = out_rec->bottom_pt;
	}
}

static out_rec_t*
find_append_linkend(out_rec_t *out_rec)
{
	while (out_rec->append_link)
		out_rec = out_rec->append_link;

	return out_rec;
}

static void
fix_hole_linkage(clipper_t *clipper, out_rec_t *out_rec)
{
	out_rec_t *tmp;
	if (out_rec->bottom_pt)
		tmp = ((out_rec_t *)ptr_vector_at(clipper->poly_outs, out_rec->bottom_pt->idx))->first_left;
	else
		tmp = out_rec->first_left;

	if (out_rec == tmp) {
		printf("¡°Clipper Error]: HoleLinkage error. \n");
		return;
	}

	if (tmp) {
		if (tmp->append_link)
			tmp = find_append_linkend(tmp);
		if (tmp == out_rec)
			tmp = NULL;
		else if (tmp->is_hole) {
			fix_hole_linkage(clipper, tmp);
			tmp = tmp->first_left;
		}
	}

	out_rec->first_left = tmp;
	if (!tmp)
		out_rec->is_hole = FALSE;

	out_rec->append_link = NULL;
}

static void
check_hole_linkages1(clipper_t *clipper, out_rec_t *out_rec1, out_rec_t *out_rec2)
{
	//when a polygon is split into 2 polygons, make sure any holes the original
	//polygon contained link to the correct polygon ...
	uint32_t i;
	out_rec_t *orec;
	for (i = 0; i < clipper->poly_outs->len; ++i) {
		orec = (out_rec_t *)ptr_vector_at(clipper->poly_outs, i);
		if (orec->is_hole && orec->bottom_pt && orec->first_left == out_rec1 &&
			!point_in_polygon(&orec->bottom_pt->pt, out_rec1->pts))
			orec->first_left = out_rec2;
	}
}

static void
check_hole_linkages2(clipper_t *clipper, out_rec_t *out_rec1, out_rec_t *out_rec2)
{
	uint32_t i;
	out_rec_t *poly_out_i;
	//if a hole is owned by outRec2 then make it owned by outRec1 ...
	for (i = 0; i < clipper->poly_outs->len; ++i) {
		poly_out_i = (out_rec_t *)ptr_vector_at(clipper->poly_outs, i);
		if (poly_out_i->is_hole && poly_out_i->bottom_pt &&
			poly_out_i->first_left == out_rec2)
			poly_out_i->first_left = out_rec1;
	}
}

static void
join_common_edges(clipper_t *clipper, bool_t fix_hole_linkages)
{
	uint32_t i, k;
	join_rec_t *j, *j2;
	out_rec_t *out_rec1, *out_rec2;
	out_pt_t *pp1a, *pp2a, *p1, *p2, *p3, *p4, *prev;
	vector2d_t pt1, pt2, pt3, pt4;
	int32_t ok_idx;
	int32_t obsolete_idx;

	for (i = 0; i < clipper->joins->len; i++) {
		j = (join_rec_t *)ptr_vector_at(clipper->joins, i);
		out_rec1 = (out_rec_t *)ptr_vector_at(clipper->poly_outs, j->poly1_idx);
		pp1a = out_rec1->pts;
		out_rec2 = (out_rec_t *)ptr_vector_at(clipper->poly_outs, j->poly2_idx);
		pp2a = out_rec2->pts;
		pt1 = j->pt2a;
		pt2 = j->pt2b;
		pt3 = j->pt1a;
		pt4 = j->pt1b;

		if (!find_segment(&pp1a, &pt1, &pt2))
			continue;
		if (j->poly1_idx == j->poly2_idx) {
			//we're searching the same polygon for overlapping segments so
			//segment 2 mustn't be the same as segment 1 ...
			pp2a = pp1a->next;
			if (!find_segment(&pp2a, &pt3, &pt4) || (pp2a == pp1a))
				continue;
		}
		else if (!find_segment(&pp2a, &pt3, &pt4))
			continue;

		if (!get_overlap_segment(pt1, pt2, pt3, pt4, &pt1, &pt2))
			continue;

		prev = pp1a->prev;
		//get p1 & p2 polypts - the overlap start & endpoints on poly1
		if (int_vertex_equal(&pp1a->pt, &pt1))
			p1 = pp1a;
		else if (int_vertex_equal(&prev->pt, &pt1))
			p1 = prev;
		else
			p1 = insert_poly_pt_between(pp1a, prev, &pt1);

		if (int_vertex_equal(&pp1a->pt, &pt2))
			p2 = pp1a;
		else if (int_vertex_equal(&prev->pt, &pt2))
			p2 = prev;
		else if ((p1 == pp1a) || (p1 == prev))
			p2 = insert_poly_pt_between(pp1a, prev, &pt2);
		else if (pt3_is_between_pt1_pt2(&pp1a->pt, &p1->pt, &pt2))
			p2 = insert_poly_pt_between(pp1a, p1, &pt2);
		else
			p2 = insert_poly_pt_between(p1, prev, &pt2);

		//get p3 & p4 polypts - the overlap start & endpoints on poly2
		prev = pp2a->prev;
		if (int_vertex_equal(&pp2a->pt, &pt1))
			p3 = pp2a;
		else if (int_vertex_equal(&prev->pt, &pt1))
			p3 = prev;
		else
			p3 = insert_poly_pt_between(pp2a, prev, &pt1);

		if (int_vertex_equal(&pp2a->pt, &pt2))
			p4 = pp2a;
		else if (int_vertex_equal(&prev->pt, &pt2))
			p4 = prev;
		else if ((p3 == pp2a) || (p3 == prev))
			p4 = insert_poly_pt_between(pp2a, prev, &pt2);
		else if (pt3_is_between_pt1_pt2(&pp2a->pt, &p3->pt, &pt2))
			p4 = insert_poly_pt_between(pp2a, p3, &pt2);
		else
			p4 = insert_poly_pt_between(p3, prev, &pt2);

		//p1.pt == p3.pt and p2.pt == p4.pt so join p1 to p3 and p2 to p4 ...
		if (p1->next == p2 && p3->prev == p4) {
			p1->next = p3;
			p3->prev = p1;
			p2->prev = p4;
			p4->next = p2;
		}
		else if (p1->prev == p2 && p3->next == p4) {
			p1->prev = p3;
			p3->next = p1;
			p2->next = p4;
			p4->prev = p2;
		}
		else
			continue; //an orientation is probably wrong

		if (j->poly2_idx == j->poly1_idx) {
			//instead of joining two polygons, we've just created a new one by
			//splitting one polygon into two.
			out_rec1->pts = polygon_bottom(p1);
			out_rec1->bottom_pt = out_rec1->pts;
			out_rec1->bottom_pt->idx = out_rec1->idx;
			out_rec2 = create_out_rec();
			ptr_vector_append_element(clipper->poly_outs, out_rec2);
			out_rec2->idx = (int32_t)clipper->poly_outs->len - 1;
			j->poly2_idx = out_rec2->idx;
			out_rec2->pts = polygon_bottom(p2);
			out_rec2->bottom_pt = out_rec2->pts;
			out_rec2->bottom_pt->idx = out_rec2->idx;

			if (point_in_polygon(&out_rec2->pts->pt, out_rec1->pts)) {
				//outRec2 is contained by outRec1 ...
				out_rec2->is_hole = !out_rec1->is_hole;
				out_rec2->first_left = out_rec1;
				if (out_rec2->is_hole == orientation(out_rec2))
					reverse_poly_pt_links(out_rec2->pts);
			}
			else if (point_in_polygon(&out_rec1->pts->pt, out_rec2->pts)) {
				//outRec1 is contained by outRec2 ...
				out_rec2->is_hole = out_rec1->is_hole;
				out_rec1->is_hole = !out_rec2->is_hole;
				out_rec2->first_left = out_rec1->first_left;
				out_rec1->first_left = out_rec2;

				if (out_rec1->is_hole == orientation(out_rec1))
					reverse_poly_pt_links(out_rec1->pts);
				//make sure any contained holes now link to the correct polygon ...
				if (fix_hole_linkages)
					check_hole_linkages1(clipper,out_rec1, out_rec2);
			}
			else {
				out_rec2->is_hole = out_rec1->is_hole;
				out_rec2->first_left = out_rec1->first_left;
				//make sure any contained holes now link to the correct polygon ...
				if (fix_hole_linkages)
					check_hole_linkages1(clipper, out_rec1, out_rec2);
			}

			//now fixup any subsequent joins that match this polygon
			for (k = i + 1; k < clipper->joins->len; k++) {
				j2 = (join_rec_t *)ptr_vector_at(clipper->joins, k);
				if (j2->poly1_idx == j->poly1_idx && point_is_vertex(&j2->pt1a, p2))
					j2->poly1_idx = j->poly2_idx;
				if (j2->poly2_idx == j->poly1_idx && point_is_vertex(&j2->pt2a, p2))
					j2->poly2_idx = j->poly2_idx;
			}

			//now cleanup redundant edges too ...
			fixup_out_polygon(out_rec1);
			fixup_out_polygon(out_rec2);
		}
		else {
			//joined 2 polygons together ...

			//make sure any holes contained by outRec2 now link to outRec1 ...
			if (fix_hole_linkages)
				check_hole_linkages2(clipper, out_rec1, out_rec2);

			//now cleanup redundant edges too ...
			fixup_out_polygon(out_rec1);

			if (out_rec1->pts) {
				out_rec1->is_hole = !orientation(out_rec1);
				if (out_rec1->is_hole && !out_rec1->first_left)
					out_rec1->first_left = out_rec2->first_left;
			}

			//delete the obsolete pointer ...
			ok_idx = out_rec1->idx;
			obsolete_idx = out_rec2->idx;
			out_rec2->pts = NULL;
			out_rec2->bottom_pt = NULL;
			out_rec2->append_link = out_rec1;

			//now fixup any subsequent Joins that match this polygon
			for (k = i + 1; k < clipper->joins->len; k++) {
				j2 = (join_rec_t *)ptr_vector_at(clipper->joins, k);
				if (j2->poly1_idx == obsolete_idx) j2->poly1_idx = ok_idx;
				if (j2->poly2_idx == obsolete_idx) j2->poly2_idx = ok_idx;
			}
		}
	}
}

static int32_t
poly_sort(const void *o1, const void *o2)
{
	int32_t i1, i2, result;
	out_rec_t *or1 = (out_rec_t *)(*(void **)o1);
	out_rec_t *or2 = (out_rec_t *)(*(void **)o2);

	if (or1 == or2)
		return 0;

	if (!or1->pts || !or2->pts) {
		if (or1->pts != or2->pts) {
			if (or1->pts)
				return 1;
			else
				return -1;
		}
		else
			return -1;
	}

	if (or1->is_hole)
		i1 = or1->first_left->idx;
	else
		i1 = or1->idx;

	if (or2->is_hole)
		i2 = or2->first_left->idx;
	else
		i2 = or2->idx;

	result = i1 - i2;
	if (result == 0 && (or1->is_hole != or2->is_hole)) {
		if (or1->is_hole)
			return -1;
		else
			return 1;
	}
	else
		return (result < 0);
}

static bool_t
execute_internal(clipper_t *clipper, bool_t fix_hole_linkages)
{
	bool_t succeeded;
	int64_t botY, topY;
	uint32_t i;
	out_rec_t* out_rec;

	clipper_reset(clipper);
	return_val_if_fail(clipper->current_lm, TRUE);

	botY = pop_scanbeam(clipper);

	do {
		insert_localminima_into_ael(clipper, botY);
		ptr_vector_clear(clipper->horz_joins, TRUE);
		process_horizontals(clipper);
		topY = pop_scanbeam(clipper);
		succeeded = process_intersections(clipper, botY, topY);
		if (!succeeded)
			break;
		process_edges_at_topof_scanbeam(clipper, topY);
		botY = topY;
	} while (clipper->scanbeam);

	if (succeeded) {
		//tidy up output polygons and fix orientations where necessary ...
		for (i = 0; i < clipper->poly_outs->len; ++i) {
			out_rec = (out_rec_t *)ptr_vector_at(clipper->poly_outs, i);
			if (!out_rec->pts)
				continue;

			fixup_out_polygon(out_rec);

			if (!out_rec->pts)
				continue;

			if (out_rec->is_hole && fix_hole_linkages)
				fix_hole_linkage(clipper, out_rec);

			if (out_rec->bottom_pt == out_rec->bottom_flag &&
				(orientation(out_rec) != (internal_area(out_rec) > 0))) {
				dispose_bottom_pt(out_rec);
				fixup_out_polygon(out_rec);
			};


			if (out_rec->is_hole ==
				(clipper->reverse_output ^ orientation(out_rec)))
				reverse_poly_pt_links(out_rec->pts);
		}

		join_common_edges(clipper, fix_hole_linkages);

		if (fix_hole_linkages)
			ptr_vector_sort(clipper->poly_outs, poly_sort);
	}

	//out_rec_printf();

	clear_joins(clipper);
	clear_horz_joins(clipper);
	return succeeded;
}


static void
build_result(clipper_t *clipper, int_polygons_t *polys)
{
	uint32_t  k = 0, i;
	out_rec_t *poly_out_i;
	int_polygon_t *pg;
	out_pt_t *p;

	ptr_vector_resize(polys, clipper->poly_outs->len);

	for (i = 0; i < clipper->poly_outs->len; ++i) {
		poly_out_i = (out_rec_t *)ptr_vector_at(clipper->poly_outs, i);

		if (poly_out_i->pts) {
			if (int_polygons_at(polys, k)) {
				int_polygon_free(int_polygons_at(polys, k));
			}

			//int_polygons_at (polys, k) = int_polygon_new();
			((polys)->pdata)[k] = int_polygon_new();

			pg = int_polygons_at(polys, k);
			p = poly_out_i->pts;
			do {
				int_polygon_append_vertice(pg, p->pt);
				p = p->next;
			} while (p != poly_out_i->pts);
			//make sure each polygon has at least 3 vertices ...
			if (pg->len < 3)
				int_polygons_remove_element(polys, k, TRUE);
			else
				k++;
		}
	}
	ptr_vector_resize(polys, k);
}

bool_t
clipper_execute(clipper_t *clipper, ClipType_t clip_type, int_polygons_t *solution,
                PolyFillType_t subj_filltype, PolyFillType_t clip_filltype)
{
	bool_t succeeded;
	int_polygons_clear(solution, TRUE);

	clipper->subject_filltype = subj_filltype;
	clipper->clip_filltype = clip_filltype;
	clipper->clip_type = clip_type;

	succeeded = execute_internal(clipper, FALSE);

	if (succeeded)
		build_result(clipper, solution);

	return succeeded;
}




void
clipper_offset_polygons(int_polygons_t* in_polys,
                        int_polygons_t* out_polys,
                        double clipper_offset,
                        JoinType_t join_type,
                        double miter_limit)
{
	int_polygons_t *m_p;
	int_polygon_t *m_curr_poly, *outer;
	vector_t *normals = NULL;
	double m_RMin, m_R, delta_sq;
	uint32_t m_i, m_j, m_k, len;
	double a1;
	int_rect_t r;
	clipper_t *clipper;

	if (NEAR_ZERO(clipper_offset)) {
		int_polygons_copy(out_polys, in_polys, TRUE);
		return;
	}

	m_p = int_polygons_new();
	int_polygons_copy(m_p, in_polys, TRUE);

	if (miter_limit <= 1)
		miter_limit = 1;

	m_RMin = 2 / (miter_limit * miter_limit);

	delta_sq = clipper_offset * clipper_offset;

	int_polygons_clear(out_polys, TRUE);
	ptr_vector_resize(out_polys, in_polys->len);

	for (m_i = 0; m_i < in_polys->len; m_i++) {
		((out_polys)->pdata)[m_i] = int_polygon_new();
		//int_polygons_at (out_polys, m_i) = int_polygon_new();
		m_curr_poly = int_polygons_at(out_polys, m_i);

		len = int_polygons_at(in_polys, m_i)->len;

		if (len > 1
			&& int_polygons_vertex(m_p, m_i, 0).X == int_polygons_vertex(m_p, m_i, len - 1).X
			&& int_polygons_vertex(m_p, m_i, 0).Y == int_polygons_vertex(m_p, m_i, len - 1).Y) {
			len--;
		}

		//when 'shrinking' polygons - to minimize artefacts
		//strip those polygons that have an area < pi * delta^2 ...
		a1 = int_polygon_area(int_polygons_at(in_polys, m_i));
		if (clipper_offset < 0) {
			if (a1 > 0 && a1 < delta_sq * PI)
				len = 0;
		}
		else if (a1 < 0 && -a1 < delta_sq * PI)
			len = 0; //holes have neg. area

		if (len == 0 || (len < 3 && clipper_offset <= 0))
			continue;
		else if (len == 1) {

			//int_polygons_at(out_polys, m_i) = 
			((out_polys)->pdata)[m_i] =
				clipper_build_arc(&int_polygons_vertex(in_polys, m_i, len - 1), 0, 2 * PI, clipper_offset);
			continue;
		}

		//build normals ...
		normals = vector_new(sizeof (double_point_t));
		vector_resize(normals, len);

		vector_at(normals, double_point_t, len - 1)
			= clipper_get_unit_normal(&int_polygons_vertex(in_polys, m_i, len - 1),
			&int_polygons_vertex(in_polys, m_i, 0));

		for (m_j = 0; m_j < len - 1; ++m_j)
			vector_at(normals, double_point_t, m_j)
			= clipper_get_unit_normal(&int_polygons_vertex(in_polys, m_i, m_j),
			&int_polygons_vertex(in_polys, m_i, m_j + 1));

		m_k = len - 1;
		for (m_j = 0; m_j < len; ++m_j) {
			switch (join_type) {
			case jt_miter:
				m_R = 1 + (vector_at(normals, double_point_t, m_j).X * vector_at(normals, double_point_t, m_k).X +
					vector_at(normals, double_point_t, m_j).Y * vector_at(normals, double_point_t, m_k).Y);

				if (m_R >= m_RMin)
					clipper_offset_miter(int_polygons_at(out_polys, m_i),
					&int_polygons_vertex(m_p, m_i, m_j),
					&vector_at(normals, double_point_t, m_j),
					&vector_at(normals, double_point_t, m_k),
					clipper_offset,
					m_R);

				else
					clipper_offset_square(int_polygons_at(out_polys, m_i),
					miter_limit,
					&int_polygons_vertex(m_p, m_i, m_j),
					&vector_at(normals, double_point_t, m_j),
					&vector_at(normals, double_point_t, m_k),
					clipper_offset);

				break;

			case jt_square:
				clipper_offset_square(m_curr_poly,
					1.0,
					&int_polygons_vertex(m_p, m_i, m_j),
					&vector_at(normals, double_point_t, m_j),
					&vector_at(normals, double_point_t, m_k),
					clipper_offset);
				break;
			case jt_round:
				clipper_offset_round(m_curr_poly,
					&int_polygons_vertex(m_p, m_i, m_j),
					&vector_at(normals, double_point_t, m_j),
					&vector_at(normals, double_point_t, m_k),
					clipper_offset);
				break;
			}
			m_k = m_j;
		}
	}

	clipper = clipper_init();
	clipper_add_polygons(clipper, out_polys, pt_subject);

	if (clipper_offset > 0) {
		if (!clipper_execute(clipper, ct_union, out_polys, pf_positive, pf_positive))
			int_polygons_clear(out_polys, TRUE);
	}
	else {
		r = get_bounds(clipper);
		outer = int_polygon_sized_new(4);

		int_polygon_at(outer, 0).X = r.left - 10;
		int_polygon_at(outer, 0).Y = r.bottom + 10;

		int_polygon_at(outer, 1).X = r.right + 10;
		int_polygon_at(outer, 1).Y = r.bottom + 10;

		int_polygon_at(outer, 2).X = r.right + 10;
		int_polygon_at(outer, 2).Y = r.top - 10;

		int_polygon_at(outer, 3).X = r.left - 10;
		int_polygon_at(outer, 3).Y = r.top - 10;

		clipper_add_polygon(clipper, outer, pt_subject);
		if (clipper_execute(clipper, ct_union, out_polys, pf_negative, pf_negative)) {
			int_polygons_remove_element(out_polys, 0, TRUE);
			int_polygons_reverse(out_polys);

		}
		else {
			int_polygons_free(out_polys, TRUE);
		}

		int_polygon_free(outer);
	}

	vector_free(normals);
	int_polygons_clear(m_p, TRUE);
	int_polygons_free(m_p, TRUE);
	clipper_free(clipper);
}

static void
dispose_local_minima_list(clipper_t *clipper)
{
	local_minima_t * tmp_lm;
	while (clipper->minima_list) {
		tmp_lm = clipper->minima_list->next;
		mem_free(clipper->minima_list);
		clipper->minima_list = tmp_lm;
	}
	clipper->current_lm = NULL;
}

static void
dispose_scanbeam_list(clipper_t *clipper)
{
	scanbeam_t* sb2;
	while (clipper->scanbeam) {
		sb2 = clipper->scanbeam->next;
		mem_free(clipper->scanbeam);
		clipper->scanbeam = sb2;
	}
}

void clipper_free(clipper_t *clipper)
{
	clipper_internal_free(clipper);
	mem_free(clipper);
}


static void
clipper_internal_free(clipper_t *clipper)
{
	uint32_t i;
	dispose_all_poly_pts(clipper);
	dispose_local_minima_list(clipper);

	for (i = 0; i < clipper->m_edges->len; i++) {
		ptr_vector_clear(clipper->m_edges, TRUE);
	}

	dispose_scanbeam_list(clipper);

	clipper->minima_list = NULL;
	clipper->current_lm = NULL;
	clipper->scanbeam = NULL;
	clipper->active_edges = NULL;
	clipper->sorted_edges = NULL;
	clipper->intersect_nodes = NULL;
	clipper->reverse_output = FALSE;

	ptr_vector_free(clipper->m_edges, TRUE);
	ptr_vector_free(clipper->poly_outs, TRUE);
}

