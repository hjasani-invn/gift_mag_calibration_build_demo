#include "imm_math.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

static bool_t ludcmp(double *A, uint32_t n, int32_t *indx, double *d);
static void lubksb (const double *A, uint32_t n, const int32_t *indx, double *b);

void 
eye(double *mat, uint32_t n)
{
	uint32_t i;

	for (i = 0;i < n; i++) 
		mat[i+i*n] = 1.0;
}


void 
matcpy (double *dest, const double *src, uint32_t n, uint32_t m)
{
	memcpy (dest, src, sizeof (double) * n * m);
}



/* multiply matrix -----------------------------------------------------------*/
void 
matmul (const char *tr, uint32_t n, uint32_t k, uint32_t m, double alpha,
	    const double *A, const double *B, double beta, double *C) 
{
	double d;
	uint32_t i,j,x;
	char f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

	for (i = 0; i < n; i++) { 
		for (j = 0; j < k; j++) {
			d = 0.0;
			switch (f) {
			case 1: 
				for (x = 0; x < m; x++) 
					d += A[i+x*n] * B[x+j*m]; 
				break;
			case 2: 
				for (x = 0;x < m;x++) 
					d += A[i+x*n] * B[j+x*k]; 
				break;
			case 3: 
				for (x = 0; x < m; x++) 
					d += A[x+i*m] * B[x+j*m]; 
				break;
			case 4:
				for (x = 0; x < m; x++) 
					d += A[x+i*m] * B[j+x*k]; 
				break;
			}

			if (beta == 0.0) 
				C[i+j*n] = alpha * d; 
			else 
				C[i+j*n] = alpha * d + beta * C[i+j*n];
		}
	}
}

void 
	matadd (double alpha, const double *A, double beta, const double *B, uint32_t n, uint32_t m, double *C) 
{
	uint32_t i,j;

	for (i = 0;i < n;i++)
		for (j = 0;j < m;j++)
			C[i+j*n] = alpha * A[i+j*n] + beta * B[i+j*n];

}

/* LU decomposition ----------------------------------------------------------*/
static bool_t 
	ludcmp(double *A, uint32_t n, int32_t *indx, double *d) 
{
	double big, s, tmp;
	double vv[4] = {0};
	uint32_t i, imax = 0, j, k;

	*d = 1.0;

	for (i = 0; i < n; i++) {
		big = 0.0; 
		for (j = 0; j < n; j++) 
			if ((tmp = ABS (A[i+j*n])) > big) 
				big = tmp;
		if (big > 0.0) 
			vv[i] = 1.0/big; 
		else 
			return FALSE;
	}

	for (j = 0; j < n; j++) {
		for (i = 0; i < j; i++) {
			s = A[i+j*n]; 
			for (k = 0;k < i; k++) 
				s -= A[i+k*n] * A[k+j*n]; 
			A[i+j*n] = s;
		}
		big = 0.0;
		for (i = j; i < n; i++) {
			s = A[i+j*n]; 
			for (k = 0; k < j; k++) 
				s -= A[i+k*n] * A[k+j*n]; 
			A[i+j*n]=s;			
			if ((tmp = vv[i] * ABS(s)) >= big) {
				big = tmp; 
				imax = i;
			}
		}

		if (j != imax) {
			for (k = 0;k < n; k++) {
				tmp = A[imax+k*n]; 
				A[imax+k*n] = A[j+k*n]; 
				A[j+k*n] = tmp;
			}
			*d=-(*d); 
			vv[imax] = vv[j];
		}

		indx[j] = (int32_t)imax;

		if (A[j+j*n] == 0.0) 
			return FALSE;

		if (j != n-1) {
			tmp =1.0 / A[j+j*n]; 
			for (i = j+1; i < n; i++) 
				A[i+j*n] *= tmp;
		}
	}

	return 0;
}

/* LU back-substitution ------------------------------------------------------*/
static void 
	lubksb (const double *A, uint32_t n, const int32_t *indx, double *b) 
{
	double s;
	int32_t ii=-1, i, ip, j;

	for (i = 0; i < (int32_t)n; i++) {
		ip = indx[i]; 
		s = b[ip]; 
		b[ip] = b[i];
		if (ii >= 0) 
			for (j = ii; j < i; j++) 
				s -= A[i+j*n] * b[j]; 
		else if (s) 
			ii= (int32_t) i;

		b[i] = s;
	}
	for (i = (int32_t)n - 1; i >= 0; i--) {
		s = b[i]; 
		for (j = i + 1; j < (int32_t)n; j++) 
			s -= A[i+j*n] * b[j]; 
		b[i] = s / A[i+i*n];
	}
}

/* inverse of matrix ---------------------------------------------------------*/
bool_t 
	matinv (double *A, uint32_t n) 
{
	double d;
	double B[16] = {0};
	uint32_t i,j;
	int32_t indx[16] = {0};

	matcpy (B,A,n,n);

	if (ludcmp (B, n, indx, &d)) {
		return FALSE;
	}

	for (j = 0;j < n;j++) {
		for (i = 0;i < n;i++) 
			A[i+j*n] = 0.0; 

		A[j+j*n] = 1.0;
		lubksb (B, n, indx, A + j * n);
	}

	return TRUE;
}

void 
	matscale (double *A, double scale, uint32_t m, uint32_t n)
{
	uint32_t i,j;
	for (i = 0; i < m; i++) {
		for (j = 0; j < n; j++) {
			A[i + j * m] *= scale; 
		}
	}
}

void 
matprint(double *A, uint32_t m, uint32_t n) {
		uint32_t i,j;

		for(i = 0; i < m; i++) {
			printf("Mat = ");
			for(j = 0; j < n; j++)
				printf("%g ", A[i + j * m]);
			printf("\n");
		}
		printf("\n");
}

void 
kalman_filter_predition(double* F, double* G, double* P, uint32_t state_num, float time) 
{

	double EYE[16] = {0}; 
	double T[16] = {0};
	double Q[16] = {0};
	double tmpP1[16] = {0};
	double tmpP2[16] = {0};
	double G2[16] = {0};
	double tmpQ1[16] = {0};
	double tmpQ2[16] = {0};

	eye (EYE, 4);

	matadd (1.0, EYE, time, F, state_num,state_num, T);
	matmul ("NT",state_num,state_num,state_num,1.0,G,G,0.0,G2);
	matmul ("NN",state_num,state_num,state_num,1.0,T,G2,0.0,tmpQ1); 
	matmul ("NT",state_num,state_num,state_num,1.0,tmpQ1,T,0.0,tmpQ2); 
	matadd (1.0, tmpQ2, 1.0, G2, state_num,state_num, Q);
	matscale (Q, time/2.0, state_num, state_num);

	matmul ("NN",state_num,state_num,state_num,1.0,T,P,0.0,tmpP1);  
	matmul ("NT",state_num,state_num,state_num,1.0,tmpP1,T,0.0,tmpP2); 
	matadd (1.0, tmpP2, 1.0, Q, state_num,state_num, P);

}

void 
kalman_filter_update (double* dX, double *P, double *H, double *R, double *Z, uint32_t nrow, uint32_t state_num) 
{
	double K[8] = {0};
	double P_Ht[8] = {0};
	double Ht_H[4] = {0};
	double Cv[4] = {0};
	double Cv_inv[4] = {0};
	double EYE[16] = {0};
	double tmpP1[16] = {0};
	double tmpP2[16] = {0};
	double tmpP3[16] = {0};

	eye(EYE, 4);

	matmul ("NT",state_num,nrow,state_num,1.0,P,H,0.0,P_Ht);
	matmul ("NN",nrow,nrow,state_num,1.0,H,P_Ht,0.0,Ht_H);
	matadd (1.0, Ht_H, 1.0, R, nrow, nrow,Cv);	
	matcpy (Cv_inv,Cv,nrow,nrow);
	matinv (Cv_inv,nrow);
	matmul ("NN",state_num,nrow,nrow,1.0,P_Ht,Cv_inv,0.0,K); 
	matmul ("NN",state_num,1,nrow,1.0,K,Z,0.0,dX); 

	matmul ("NN",state_num,state_num,nrow,1.0,K,H,0.0,tmpP1); 
	matadd (1.0, EYE, -1.0, tmpP1, state_num,state_num,tmpP2);
	matcpy (tmpP3,P,state_num,state_num);
	matmul ("NN",state_num,state_num,state_num,1.0,tmpP2,tmpP3,0.0,P); 
}


void 
eigenvector_2x2 (double *eigenvalues, double *eigenvector1, double *eigenvector2, const double *A)
{
	double a, b, c, d, T, D, B, n;

	a = A[0]; b = A[2];
	c = A[1]; d = A[3];

	T = a + d;
	D = a * d - b * c;

	B = sqrt (T * T / 4.0 - D);
	eigenvalues[0] = T / 2.0 + B;
	eigenvalues[1] = T / 2.0 - B;


	if (NEAR_ZERO (b) && NEAR_ZERO(c)) {
		eigenvector1[0] = 1.0;
		eigenvector1[1] = 0.0;

		eigenvector2[0] = 0.0;
		eigenvector2[1] = 1.0;
	} else if (!NEAR_ZERO (c)) {
		eigenvector1[0] = eigenvalues[0] - d;
		eigenvector1[1] = c;

		n = sqrt (eigenvector1[0] * eigenvector1[0] + eigenvector1[1] * eigenvector1[1]);
		eigenvector1[0] /= n;
		eigenvector1[1] /= n;


		eigenvector2[0] = eigenvalues[1] - d;
		eigenvector2[1] = c;
		n = sqrt (eigenvector2[0] * eigenvector2[0] + eigenvector2[1] * eigenvector2[1]);
		eigenvector2[0] /= n;
		eigenvector2[1] /= n;
	} else if (!NEAR_ZERO (b)) {
		eigenvector1[0] = b;
		eigenvector1[1] = eigenvalues[0] - a;
		n = sqrt (eigenvector1[0] * eigenvector1[0] + eigenvector1[1] * eigenvector1[1]);
		eigenvector1[0] /= n;
		eigenvector1[1] /= n;

		eigenvector2[0] = b;
		eigenvector2[1] = eigenvalues[1] - a;
		n = sqrt (eigenvector2[0] * eigenvector2[0] + eigenvector2[1] * eigenvector2[1]);
		eigenvector2[0] /= n;
		eigenvector2[1] /= n;
	}

}


int32_t 
float_round_int32 (float val)
{
	if ((val < 0)) 
		return (int32_t)(val - 0.5);
	else 
		return (int32_t)(val + 0.5);
}