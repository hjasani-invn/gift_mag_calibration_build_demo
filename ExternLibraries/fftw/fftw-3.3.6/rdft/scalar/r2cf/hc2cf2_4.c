/*
 * Copyright (c) 2003, 2007-14 Matteo Frigo
 * Copyright (c) 2003, 2007-14 Massachusetts Institute of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

/* This file was automatically generated --- DO NOT EDIT */
/* Generated on Fri Jan 27 16:12:33 EST 2017 */

#include "codelet-rdft.h"

#ifdef HAVE_FMA

/* Generated by: ../../../genfft/gen_hc2c.native -fma -reorder-insns -schedule-for-pipeline -compact -variables 4 -pipeline-latency 4 -twiddle-log3 -precompute-twiddles -n 4 -dit -name hc2cf2_4 -include hc2cf.h */

/*
 * This function contains 24 FP additions, 16 FP multiplications,
 * (or, 16 additions, 8 multiplications, 8 fused multiply/add),
 * 33 stack variables, 0 constants, and 16 memory accesses
 */
#include "hc2cf.h"

static void hc2cf2_4(R *Rp, R *Ip, R *Rm, R *Im, const R *W, stride rs, INT mb, INT me, INT ms)
{
     {
	  INT m;
	  for (m = mb, W = W + ((mb - 1) * 4); m < me; m = m + 1, Rp = Rp + ms, Ip = Ip + ms, Rm = Rm - ms, Im = Im - ms, W = W + 4, MAKE_VOLATILE_STRIDE(16, rs)) {
	       E Ti, Tq, To, Te, Ty, TA, Tm, Ts;
	       {
		    E T2, T6, T3, T5;
		    T2 = W[0];
		    T6 = W[3];
		    T3 = W[2];
		    T5 = W[1];
		    {
			 E T1, Tx, Td, Tw, Tj, Tl, Ta, T4, Tk, Tr;
			 T1 = Rp[0];
			 Ta = T2 * T6;
			 T4 = T2 * T3;
			 Tx = Rm[0];
			 {
			      E T8, Tb, T7, Tc;
			      T8 = Rp[WS(rs, 1)];
			      Tb = FNMS(T5, T3, Ta);
			      T7 = FMA(T5, T6, T4);
			      Tc = Rm[WS(rs, 1)];
			      {
				   E Tf, Th, T9, Tv, Tg, Tp;
				   Tf = Ip[0];
				   Th = Im[0];
				   T9 = T7 * T8;
				   Tv = T7 * Tc;
				   Tg = T2 * Tf;
				   Tp = T2 * Th;
				   Td = FMA(Tb, Tc, T9);
				   Tw = FNMS(Tb, T8, Tv);
				   Ti = FMA(T5, Th, Tg);
				   Tq = FNMS(T5, Tf, Tp);
			      }
			      Tj = Ip[WS(rs, 1)];
			      Tl = Im[WS(rs, 1)];
			 }
			 To = T1 - Td;
			 Te = T1 + Td;
			 Ty = Tw + Tx;
			 TA = Tx - Tw;
			 Tk = T3 * Tj;
			 Tr = T3 * Tl;
			 Tm = FMA(T6, Tl, Tk);
			 Ts = FNMS(T6, Tj, Tr);
		    }
	       }
	       {
		    E Tn, Tz, Tu, Tt;
		    Tn = Ti + Tm;
		    Tz = Tm - Ti;
		    Tu = Tq + Ts;
		    Tt = Tq - Ts;
		    Ip[WS(rs, 1)] = Tz + TA;
		    Im[0] = Tz - TA;
		    Rp[0] = Te + Tn;
		    Rm[WS(rs, 1)] = Te - Tn;
		    Rp[WS(rs, 1)] = To + Tt;
		    Rm[0] = To - Tt;
		    Ip[0] = Tu + Ty;
		    Im[WS(rs, 1)] = Tu - Ty;
	       }
	  }
     }
}

static const tw_instr twinstr[] = {
     {TW_CEXP, 1, 1},
     {TW_CEXP, 1, 3},
     {TW_NEXT, 1, 0}
};

static const hc2c_desc desc = { 4, "hc2cf2_4", twinstr, &GENUS, {16, 8, 8, 0} };

void X(codelet_hc2cf2_4) (planner *p) {
     X(khc2c_register) (p, hc2cf2_4, &desc, HC2C_VIA_RDFT);
}
#else				/* HAVE_FMA */

/* Generated by: ../../../genfft/gen_hc2c.native -compact -variables 4 -pipeline-latency 4 -twiddle-log3 -precompute-twiddles -n 4 -dit -name hc2cf2_4 -include hc2cf.h */

/*
 * This function contains 24 FP additions, 16 FP multiplications,
 * (or, 16 additions, 8 multiplications, 8 fused multiply/add),
 * 21 stack variables, 0 constants, and 16 memory accesses
 */
#include "hc2cf.h"

static void hc2cf2_4(R *Rp, R *Ip, R *Rm, R *Im, const R *W, stride rs, INT mb, INT me, INT ms)
{
     {
	  INT m;
	  for (m = mb, W = W + ((mb - 1) * 4); m < me; m = m + 1, Rp = Rp + ms, Ip = Ip + ms, Rm = Rm - ms, Im = Im - ms, W = W + 4, MAKE_VOLATILE_STRIDE(16, rs)) {
	       E T2, T4, T3, T5, T6, T8;
	       T2 = W[0];
	       T4 = W[1];
	       T3 = W[2];
	       T5 = W[3];
	       T6 = FMA(T2, T3, T4 * T5);
	       T8 = FNMS(T4, T3, T2 * T5);
	       {
		    E T1, Tp, Ta, To, Te, Tk, Th, Tl, T7, T9;
		    T1 = Rp[0];
		    Tp = Rm[0];
		    T7 = Rp[WS(rs, 1)];
		    T9 = Rm[WS(rs, 1)];
		    Ta = FMA(T6, T7, T8 * T9);
		    To = FNMS(T8, T7, T6 * T9);
		    {
			 E Tc, Td, Tf, Tg;
			 Tc = Ip[0];
			 Td = Im[0];
			 Te = FMA(T2, Tc, T4 * Td);
			 Tk = FNMS(T4, Tc, T2 * Td);
			 Tf = Ip[WS(rs, 1)];
			 Tg = Im[WS(rs, 1)];
			 Th = FMA(T3, Tf, T5 * Tg);
			 Tl = FNMS(T5, Tf, T3 * Tg);
		    }
		    {
			 E Tb, Ti, Tn, Tq;
			 Tb = T1 + Ta;
			 Ti = Te + Th;
			 Rm[WS(rs, 1)] = Tb - Ti;
			 Rp[0] = Tb + Ti;
			 Tn = Tk + Tl;
			 Tq = To + Tp;
			 Im[WS(rs, 1)] = Tn - Tq;
			 Ip[0] = Tn + Tq;
		    }
		    {
			 E Tj, Tm, Tr, Ts;
			 Tj = T1 - Ta;
			 Tm = Tk - Tl;
			 Rm[0] = Tj - Tm;
			 Rp[WS(rs, 1)] = Tj + Tm;
			 Tr = Th - Te;
			 Ts = Tp - To;
			 Im[0] = Tr - Ts;
			 Ip[WS(rs, 1)] = Tr + Ts;
		    }
	       }
	  }
     }
}

static const tw_instr twinstr[] = {
     {TW_CEXP, 1, 1},
     {TW_CEXP, 1, 3},
     {TW_NEXT, 1, 0}
};

static const hc2c_desc desc = { 4, "hc2cf2_4", twinstr, &GENUS, {16, 8, 8, 0} };

void X(codelet_hc2cf2_4) (planner *p) {
     X(khc2c_register) (p, hc2cf2_4, &desc, HC2C_VIA_RDFT);
}
#endif				/* HAVE_FMA */
