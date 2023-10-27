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
/* Generated on Fri Jan 27 16:13:09 EST 2017 */

#include "codelet-rdft.h"

#ifdef HAVE_FMA

/* Generated by: ../../../genfft/gen_r2cb.native -fma -reorder-insns -schedule-for-pipeline -compact -variables 4 -pipeline-latency 4 -sign 1 -n 5 -name r2cbIII_5 -dft-III -include r2cbIII.h */

/*
 * This function contains 12 FP additions, 10 FP multiplications,
 * (or, 2 additions, 0 multiplications, 10 fused multiply/add),
 * 18 stack variables, 5 constants, and 10 memory accesses
 */
#include "r2cbIII.h"

static void r2cbIII_5(R *R0, R *R1, R *Cr, R *Ci, stride rs, stride csr, stride csi, INT v, INT ivs, INT ovs)
{
     DK(KP1_902113032, +1.902113032590307144232878666758764286811397268);
     DK(KP1_118033988, +1.118033988749894848204586834365638117720309180);
     DK(KP500000000, +0.500000000000000000000000000000000000000000000);
     DK(KP2_000000000, +2.000000000000000000000000000000000000000000000);
     DK(KP618033988, +0.618033988749894848204586834365638117720309180);
     {
	  INT i;
	  for (i = v; i > 0; i = i - 1, R0 = R0 + ovs, R1 = R1 + ovs, Cr = Cr + ivs, Ci = Ci + ivs, MAKE_VOLATILE_STRIDE(20, rs), MAKE_VOLATILE_STRIDE(20, csr), MAKE_VOLATILE_STRIDE(20, csi)) {
	       E T1, T2, T3, Tc, Ta, T8, T9;
	       T8 = Ci[WS(csi, 1)];
	       T9 = Ci[0];
	       T1 = Cr[WS(csr, 2)];
	       T2 = Cr[WS(csr, 1)];
	       T3 = Cr[0];
	       Tc = FMS(KP618033988, T8, T9);
	       Ta = FMA(KP618033988, T9, T8);
	       {
		    E T6, T4, T5, T7, Tb;
		    T6 = T3 - T2;
		    T4 = T2 + T3;
		    R0[0] = FMA(KP2_000000000, T4, T1);
		    T5 = FNMS(KP500000000, T4, T1);
		    T7 = FNMS(KP1_118033988, T6, T5);
		    Tb = FMA(KP1_118033988, T6, T5);
		    R0[WS(rs, 2)] = FNMS(KP1_902113032, Ta, T7);
		    R1[0] = -(FMA(KP1_902113032, Ta, T7));
		    R1[WS(rs, 1)] = FMS(KP1_902113032, Tc, Tb);
		    R0[WS(rs, 1)] = FMA(KP1_902113032, Tc, Tb);
	       }
	  }
     }
}

static const kr2c_desc desc = { 5, "r2cbIII_5", {2, 0, 10, 0}, &GENUS };

void X(codelet_r2cbIII_5) (planner *p) {
     X(kr2c_register) (p, r2cbIII_5, &desc);
}

#else				/* HAVE_FMA */

/* Generated by: ../../../genfft/gen_r2cb.native -compact -variables 4 -pipeline-latency 4 -sign 1 -n 5 -name r2cbIII_5 -dft-III -include r2cbIII.h */

/*
 * This function contains 12 FP additions, 7 FP multiplications,
 * (or, 8 additions, 3 multiplications, 4 fused multiply/add),
 * 18 stack variables, 5 constants, and 10 memory accesses
 */
#include "r2cbIII.h"

static void r2cbIII_5(R *R0, R *R1, R *Cr, R *Ci, stride rs, stride csr, stride csi, INT v, INT ivs, INT ovs)
{
     DK(KP2_000000000, +2.000000000000000000000000000000000000000000000);
     DK(KP1_118033988, +1.118033988749894848204586834365638117720309180);
     DK(KP500000000, +0.500000000000000000000000000000000000000000000);
     DK(KP1_175570504, +1.175570504584946258337411909278145537195304875);
     DK(KP1_902113032, +1.902113032590307144232878666758764286811397268);
     {
	  INT i;
	  for (i = v; i > 0; i = i - 1, R0 = R0 + ovs, R1 = R1 + ovs, Cr = Cr + ivs, Ci = Ci + ivs, MAKE_VOLATILE_STRIDE(20, rs), MAKE_VOLATILE_STRIDE(20, csr), MAKE_VOLATILE_STRIDE(20, csi)) {
	       E Ta, Tc, T1, T4, T5, T6, Tb, T7;
	       {
		    E T8, T9, T2, T3;
		    T8 = Ci[WS(csi, 1)];
		    T9 = Ci[0];
		    Ta = FMA(KP1_902113032, T8, KP1_175570504 * T9);
		    Tc = FNMS(KP1_902113032, T9, KP1_175570504 * T8);
		    T1 = Cr[WS(csr, 2)];
		    T2 = Cr[WS(csr, 1)];
		    T3 = Cr[0];
		    T4 = T2 + T3;
		    T5 = FMS(KP500000000, T4, T1);
		    T6 = KP1_118033988 * (T3 - T2);
	       }
	       R0[0] = FMA(KP2_000000000, T4, T1);
	       Tb = T6 - T5;
	       R0[WS(rs, 1)] = Tb + Tc;
	       R1[WS(rs, 1)] = Tc - Tb;
	       T7 = T5 + T6;
	       R1[0] = T7 - Ta;
	       R0[WS(rs, 2)] = -(T7 + Ta);
	  }
     }
}

static const kr2c_desc desc = { 5, "r2cbIII_5", {8, 3, 4, 0}, &GENUS };

void X(codelet_r2cbIII_5) (planner *p) {
     X(kr2c_register) (p, r2cbIII_5, &desc);
}

#endif				/* HAVE_FMA */
