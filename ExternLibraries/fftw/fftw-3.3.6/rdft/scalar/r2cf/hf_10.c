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
/* Generated on Fri Jan 27 16:12:01 EST 2017 */

#include "codelet-rdft.h"

#ifdef HAVE_FMA

/* Generated by: ../../../genfft/gen_hc2hc.native -fma -reorder-insns -schedule-for-pipeline -compact -variables 4 -pipeline-latency 4 -n 10 -dit -name hf_10 -include hf.h */

/*
 * This function contains 102 FP additions, 72 FP multiplications,
 * (or, 48 additions, 18 multiplications, 54 fused multiply/add),
 * 72 stack variables, 4 constants, and 40 memory accesses
 */
#include "hf.h"

static void hf_10(R *cr, R *ci, const R *W, stride rs, INT mb, INT me, INT ms)
{
     DK(KP951056516, +0.951056516295153572116439333379382143405698634);
     DK(KP559016994, +0.559016994374947424102293417182819058860154590);
     DK(KP250000000, +0.250000000000000000000000000000000000000000000);
     DK(KP618033988, +0.618033988749894848204586834365638117720309180);
     {
	  INT m;
	  for (m = mb, W = W + ((mb - 1) * 18); m < me; m = m + 1, cr = cr + ms, ci = ci - ms, W = W + 18, MAKE_VOLATILE_STRIDE(20, rs)) {
	       E T29, T2d, T2c, T2e;
	       {
		    E T23, T1U, T8, T12, T1y, T1P, T25, T1H, T2b, T18, T10, T1Y, T1I, Tl, T13;
		    E T1J, Ty, T14, T1n, T1O, T24, T1K;
		    {
			 E T1, T1R, T3, T6, T2, T5;
			 T1 = cr[0];
			 T1R = ci[0];
			 T3 = cr[WS(rs, 5)];
			 T6 = ci[WS(rs, 5)];
			 T2 = W[8];
			 T5 = W[9];
			 {
			      E T1p, TY, T1x, T1F, TM, T16, T1r, TS;
			      {
				   E TF, T1w, TO, TR, T1u, TL, TN, TQ, T1q, TP;
				   {
					E TU, TX, TT, TW;
					{
					     E TB, TE, T1S, T4, TA, TD;
					     TB = cr[WS(rs, 4)];
					     TE = ci[WS(rs, 4)];
					     T1S = T2 * T6;
					     T4 = T2 * T3;
					     TA = W[6];
					     TD = W[7];
					     {
						  E T1T, T7, T1v, TC;
						  T1T = FNMS(T5, T3, T1S);
						  T7 = FMA(T5, T6, T4);
						  T1v = TA * TE;
						  TC = TA * TB;
						  T23 = T1T + T1R;
						  T1U = T1R - T1T;
						  T8 = T1 - T7;
						  T12 = T1 + T7;
						  TF = FMA(TD, TE, TC);
						  T1w = FNMS(TD, TB, T1v);
					     }
					}
					TU = cr[WS(rs, 1)];
					TX = ci[WS(rs, 1)];
					TT = W[0];
					TW = W[1];
					{
					     E TH, TK, TJ, T1t, TI, T1o, TV, TG;
					     TH = cr[WS(rs, 9)];
					     TK = ci[WS(rs, 9)];
					     T1o = TT * TX;
					     TV = TT * TU;
					     TG = W[16];
					     TJ = W[17];
					     T1p = FNMS(TW, TU, T1o);
					     TY = FMA(TW, TX, TV);
					     T1t = TG * TK;
					     TI = TG * TH;
					     TO = cr[WS(rs, 6)];
					     TR = ci[WS(rs, 6)];
					     T1u = FNMS(TJ, TH, T1t);
					     TL = FMA(TJ, TK, TI);
					     TN = W[10];
					     TQ = W[11];
					}
				   }
				   T1x = T1u - T1w;
				   T1F = T1w + T1u;
				   TM = TF - TL;
				   T16 = TF + TL;
				   T1q = TN * TR;
				   TP = TN * TO;
				   T1r = FNMS(TQ, TO, T1q);
				   TS = FMA(TQ, TR, TP);
			      }
			      {
				   E T1l, Te, T1e, Tx, Tn, Tq, Tp, T1j, Tk, T1f, To;
				   {
					E Tt, Tw, Tv, T1d, Tu;
					{
					     E Ta, Td, T9, Tc, T1k, Tb, Ts;
					     Ta = cr[WS(rs, 2)];
					     Td = ci[WS(rs, 2)];
					     {
						  E T1G, T1s, TZ, T17;
						  T1G = T1r + T1p;
						  T1s = T1p - T1r;
						  TZ = TS - TY;
						  T17 = TS + TY;
						  T1y = T1s - T1x;
						  T1P = T1x + T1s;
						  T25 = T1F + T1G;
						  T1H = T1F - T1G;
						  T2b = T16 - T17;
						  T18 = T16 + T17;
						  T10 = TM + TZ;
						  T1Y = TZ - TM;
						  T9 = W[2];
					     }
					     Tc = W[3];
					     Tt = cr[WS(rs, 3)];
					     Tw = ci[WS(rs, 3)];
					     T1k = T9 * Td;
					     Tb = T9 * Ta;
					     Ts = W[4];
					     Tv = W[5];
					     T1l = FNMS(Tc, Ta, T1k);
					     Te = FMA(Tc, Td, Tb);
					     T1d = Ts * Tw;
					     Tu = Ts * Tt;
					}
					{
					     E Tg, Tj, Tf, Ti, T1i, Th, Tm;
					     Tg = cr[WS(rs, 7)];
					     Tj = ci[WS(rs, 7)];
					     T1e = FNMS(Tv, Tt, T1d);
					     Tx = FMA(Tv, Tw, Tu);
					     Tf = W[12];
					     Ti = W[13];
					     Tn = cr[WS(rs, 8)];
					     Tq = ci[WS(rs, 8)];
					     T1i = Tf * Tj;
					     Th = Tf * Tg;
					     Tm = W[14];
					     Tp = W[15];
					     T1j = FNMS(Ti, Tg, T1i);
					     Tk = FMA(Ti, Tj, Th);
					     T1f = Tm * Tq;
					     To = Tm * Tn;
					}
				   }
				   {
					E T1m, T1g, Tr, T1h;
					T1m = T1j - T1l;
					T1I = T1l + T1j;
					Tl = Te - Tk;
					T13 = Te + Tk;
					T1g = FNMS(Tp, Tn, T1f);
					Tr = FMA(Tp, Tq, To);
					T1J = T1g + T1e;
					T1h = T1e - T1g;
					Ty = Tr - Tx;
					T14 = Tr + Tx;
					T1n = T1h - T1m;
					T1O = T1m + T1h;
				   }
			      }
			 }
		    }
		    T24 = T1I + T1J;
		    T1K = T1I - T1J;
		    {
			 E T2a, T15, Tz, T1Z;
			 T2a = T13 - T14;
			 T15 = T13 + T14;
			 Tz = Tl + Ty;
			 T1Z = Ty - Tl;
			 {
			      E T1L, T1N, T1E, T1M;
			      {
				   E T19, T1D, T1C, T11, T1b;
				   T19 = T15 + T18;
				   T1D = T15 - T18;
				   T11 = Tz + T10;
				   T1b = Tz - T10;
				   {
					E T1B, T1z, T1a, T1A, T1c;
					T1B = FNMS(KP618033988, T1n, T1y);
					T1z = FMA(KP618033988, T1y, T1n);
					ci[WS(rs, 4)] = T8 + T11;
					T1a = FNMS(KP250000000, T11, T8);
					T1A = FNMS(KP559016994, T1b, T1a);
					T1c = FMA(KP559016994, T1b, T1a);
					T1C = FNMS(KP250000000, T19, T12);
					T1L = FNMS(KP618033988, T1K, T1H);
					T1N = FMA(KP618033988, T1H, T1K);
					cr[WS(rs, 1)] = FMA(KP951056516, T1z, T1c);
					ci[0] = FNMS(KP951056516, T1z, T1c);
					cr[WS(rs, 3)] = FMA(KP951056516, T1B, T1A);
					ci[WS(rs, 2)] = FNMS(KP951056516, T1B, T1A);
				   }
				   cr[0] = T12 + T19;
				   T1E = FNMS(KP559016994, T1D, T1C);
				   T1M = FMA(KP559016994, T1D, T1C);
			      }
			      {
				   E T1X, T21, T20, T22, T1Q, T1W, T1V, T26, T28, T27;
				   T1Q = T1O + T1P;
				   T1W = T1P - T1O;
				   ci[WS(rs, 3)] = FMA(KP951056516, T1N, T1M);
				   cr[WS(rs, 4)] = FNMS(KP951056516, T1N, T1M);
				   ci[WS(rs, 1)] = FMA(KP951056516, T1L, T1E);
				   cr[WS(rs, 2)] = FNMS(KP951056516, T1L, T1E);
				   T1V = FMA(KP250000000, T1Q, T1U);
				   cr[WS(rs, 5)] = T1Q - T1U;
				   T1X = FNMS(KP559016994, T1W, T1V);
				   T21 = FMA(KP559016994, T1W, T1V);
				   T20 = FNMS(KP618033988, T1Z, T1Y);
				   T22 = FMA(KP618033988, T1Y, T1Z);
				   T26 = T24 + T25;
				   T28 = T24 - T25;
				   ci[WS(rs, 8)] = FMA(KP951056516, T22, T21);
				   cr[WS(rs, 9)] = FMS(KP951056516, T22, T21);
				   ci[WS(rs, 6)] = FMA(KP951056516, T20, T1X);
				   cr[WS(rs, 7)] = FMS(KP951056516, T20, T1X);
				   T27 = FNMS(KP250000000, T26, T23);
				   ci[WS(rs, 9)] = T26 + T23;
				   T29 = FMA(KP559016994, T28, T27);
				   T2d = FNMS(KP559016994, T28, T27);
				   T2c = FMA(KP618033988, T2b, T2a);
				   T2e = FNMS(KP618033988, T2a, T2b);
			      }
			 }
		    }
	       }
	       ci[WS(rs, 7)] = FMA(KP951056516, T2e, T2d);
	       cr[WS(rs, 8)] = FMS(KP951056516, T2e, T2d);
	       ci[WS(rs, 5)] = FMA(KP951056516, T2c, T29);
	       cr[WS(rs, 6)] = FMS(KP951056516, T2c, T29);
	  }
     }
}

static const tw_instr twinstr[] = {
     {TW_FULL, 1, 10},
     {TW_NEXT, 1, 0}
};

static const hc2hc_desc desc = { 10, "hf_10", twinstr, &GENUS, {48, 18, 54, 0} };

void X(codelet_hf_10) (planner *p) {
     X(khc2hc_register) (p, hf_10, &desc);
}
#else				/* HAVE_FMA */

/* Generated by: ../../../genfft/gen_hc2hc.native -compact -variables 4 -pipeline-latency 4 -n 10 -dit -name hf_10 -include hf.h */

/*
 * This function contains 102 FP additions, 60 FP multiplications,
 * (or, 72 additions, 30 multiplications, 30 fused multiply/add),
 * 45 stack variables, 4 constants, and 40 memory accesses
 */
#include "hf.h"

static void hf_10(R *cr, R *ci, const R *W, stride rs, INT mb, INT me, INT ms)
{
     DK(KP587785252, +0.587785252292473129168705954639072768597652438);
     DK(KP951056516, +0.951056516295153572116439333379382143405698634);
     DK(KP250000000, +0.250000000000000000000000000000000000000000000);
     DK(KP559016994, +0.559016994374947424102293417182819058860154590);
     {
	  INT m;
	  for (m = mb, W = W + ((mb - 1) * 18); m < me; m = m + 1, cr = cr + ms, ci = ci - ms, W = W + 18, MAKE_VOLATILE_STRIDE(20, rs)) {
	       E T7, T1R, TT, T1C, TF, TQ, TR, T1o, T1p, T1P, TX, TY, TZ, T1d, T1g;
	       E T1x, Ti, Tt, Tu, T1r, T1s, T1O, TU, TV, TW, T16, T19, T1y;
	       {
		    E T1, T1A, T6, T1B;
		    T1 = cr[0];
		    T1A = ci[0];
		    {
			 E T3, T5, T2, T4;
			 T3 = cr[WS(rs, 5)];
			 T5 = ci[WS(rs, 5)];
			 T2 = W[8];
			 T4 = W[9];
			 T6 = FMA(T2, T3, T4 * T5);
			 T1B = FNMS(T4, T3, T2 * T5);
		    }
		    T7 = T1 - T6;
		    T1R = T1B + T1A;
		    TT = T1 + T6;
		    T1C = T1A - T1B;
	       }
	       {
		    E Tz, T1b, TP, T1e, TE, T1c, TK, T1f;
		    {
			 E Tw, Ty, Tv, Tx;
			 Tw = cr[WS(rs, 4)];
			 Ty = ci[WS(rs, 4)];
			 Tv = W[6];
			 Tx = W[7];
			 Tz = FMA(Tv, Tw, Tx * Ty);
			 T1b = FNMS(Tx, Tw, Tv * Ty);
		    }
		    {
			 E TM, TO, TL, TN;
			 TM = cr[WS(rs, 1)];
			 TO = ci[WS(rs, 1)];
			 TL = W[0];
			 TN = W[1];
			 TP = FMA(TL, TM, TN * TO);
			 T1e = FNMS(TN, TM, TL * TO);
		    }
		    {
			 E TB, TD, TA, TC;
			 TB = cr[WS(rs, 9)];
			 TD = ci[WS(rs, 9)];
			 TA = W[16];
			 TC = W[17];
			 TE = FMA(TA, TB, TC * TD);
			 T1c = FNMS(TC, TB, TA * TD);
		    }
		    {
			 E TH, TJ, TG, TI;
			 TH = cr[WS(rs, 6)];
			 TJ = ci[WS(rs, 6)];
			 TG = W[10];
			 TI = W[11];
			 TK = FMA(TG, TH, TI * TJ);
			 T1f = FNMS(TI, TH, TG * TJ);
		    }
		    TF = Tz - TE;
		    TQ = TK - TP;
		    TR = TF + TQ;
		    T1o = T1b + T1c;
		    T1p = T1f + T1e;
		    T1P = T1o + T1p;
		    TX = Tz + TE;
		    TY = TK + TP;
		    TZ = TX + TY;
		    T1d = T1b - T1c;
		    T1g = T1e - T1f;
		    T1x = T1g - T1d;
	       }
	       {
		    E Tc, T14, Ts, T18, Th, T15, Tn, T17;
		    {
			 E T9, Tb, T8, Ta;
			 T9 = cr[WS(rs, 2)];
			 Tb = ci[WS(rs, 2)];
			 T8 = W[2];
			 Ta = W[3];
			 Tc = FMA(T8, T9, Ta * Tb);
			 T14 = FNMS(Ta, T9, T8 * Tb);
		    }
		    {
			 E Tp, Tr, To, Tq;
			 Tp = cr[WS(rs, 3)];
			 Tr = ci[WS(rs, 3)];
			 To = W[4];
			 Tq = W[5];
			 Ts = FMA(To, Tp, Tq * Tr);
			 T18 = FNMS(Tq, Tp, To * Tr);
		    }
		    {
			 E Te, Tg, Td, Tf;
			 Te = cr[WS(rs, 7)];
			 Tg = ci[WS(rs, 7)];
			 Td = W[12];
			 Tf = W[13];
			 Th = FMA(Td, Te, Tf * Tg);
			 T15 = FNMS(Tf, Te, Td * Tg);
		    }
		    {
			 E Tk, Tm, Tj, Tl;
			 Tk = cr[WS(rs, 8)];
			 Tm = ci[WS(rs, 8)];
			 Tj = W[14];
			 Tl = W[15];
			 Tn = FMA(Tj, Tk, Tl * Tm);
			 T17 = FNMS(Tl, Tk, Tj * Tm);
		    }
		    Ti = Tc - Th;
		    Tt = Tn - Ts;
		    Tu = Ti + Tt;
		    T1r = T14 + T15;
		    T1s = T17 + T18;
		    T1O = T1r + T1s;
		    TU = Tc + Th;
		    TV = Tn + Ts;
		    TW = TU + TV;
		    T16 = T14 - T15;
		    T19 = T17 - T18;
		    T1y = T16 + T19;
	       }
	       {
		    E T11, TS, T12, T1i, T1k, T1a, T1h, T1j, T13;
		    T11 = KP559016994 * (Tu - TR);
		    TS = Tu + TR;
		    T12 = FNMS(KP250000000, TS, T7);
		    T1a = T16 - T19;
		    T1h = T1d + T1g;
		    T1i = FMA(KP951056516, T1a, KP587785252 * T1h);
		    T1k = FNMS(KP587785252, T1a, KP951056516 * T1h);
		    ci[WS(rs, 4)] = T7 + TS;
		    T1j = T12 - T11;
		    ci[WS(rs, 2)] = T1j - T1k;
		    cr[WS(rs, 3)] = T1j + T1k;
		    T13 = T11 + T12;
		    ci[0] = T13 - T1i;
		    cr[WS(rs, 1)] = T13 + T1i;
	       }
	       {
		    E T1m, T10, T1l, T1u, T1w, T1q, T1t, T1v, T1n;
		    T1m = KP559016994 * (TW - TZ);
		    T10 = TW + TZ;
		    T1l = FNMS(KP250000000, T10, TT);
		    T1q = T1o - T1p;
		    T1t = T1r - T1s;
		    T1u = FNMS(KP587785252, T1t, KP951056516 * T1q);
		    T1w = FMA(KP951056516, T1t, KP587785252 * T1q);
		    cr[0] = TT + T10;
		    T1v = T1m + T1l;
		    cr[WS(rs, 4)] = T1v - T1w;
		    ci[WS(rs, 3)] = T1v + T1w;
		    T1n = T1l - T1m;
		    cr[WS(rs, 2)] = T1n - T1u;
		    ci[WS(rs, 1)] = T1n + T1u;
	       }
	       {
		    E T1H, T1z, T1G, T1F, T1J, T1D, T1E, T1K, T1I;
		    T1H = KP559016994 * (T1y + T1x);
		    T1z = T1x - T1y;
		    T1G = FMA(KP250000000, T1z, T1C);
		    T1D = Ti - Tt;
		    T1E = TQ - TF;
		    T1F = FMA(KP587785252, T1D, KP951056516 * T1E);
		    T1J = FNMS(KP951056516, T1D, KP587785252 * T1E);
		    cr[WS(rs, 5)] = T1z - T1C;
		    T1K = T1H + T1G;
		    cr[WS(rs, 9)] = T1J - T1K;
		    ci[WS(rs, 8)] = T1J + T1K;
		    T1I = T1G - T1H;
		    cr[WS(rs, 7)] = T1F - T1I;
		    ci[WS(rs, 6)] = T1F + T1I;
	       }
	       {
		    E T1Q, T1S, T1T, T1N, T1V, T1L, T1M, T1W, T1U;
		    T1Q = KP559016994 * (T1O - T1P);
		    T1S = T1O + T1P;
		    T1T = FNMS(KP250000000, T1S, T1R);
		    T1L = TU - TV;
		    T1M = TX - TY;
		    T1N = FMA(KP951056516, T1L, KP587785252 * T1M);
		    T1V = FNMS(KP587785252, T1L, KP951056516 * T1M);
		    ci[WS(rs, 9)] = T1S + T1R;
		    T1W = T1T - T1Q;
		    cr[WS(rs, 8)] = T1V - T1W;
		    ci[WS(rs, 7)] = T1V + T1W;
		    T1U = T1Q + T1T;
		    cr[WS(rs, 6)] = T1N - T1U;
		    ci[WS(rs, 5)] = T1N + T1U;
	       }
	  }
     }
}

static const tw_instr twinstr[] = {
     {TW_FULL, 1, 10},
     {TW_NEXT, 1, 0}
};

static const hc2hc_desc desc = { 10, "hf_10", twinstr, &GENUS, {72, 30, 30, 0} };

void X(codelet_hf_10) (planner *p) {
     X(khc2hc_register) (p, hf_10, &desc);
}
#endif				/* HAVE_FMA */
