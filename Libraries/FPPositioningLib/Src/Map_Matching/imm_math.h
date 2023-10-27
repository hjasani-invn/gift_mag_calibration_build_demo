#ifndef _IMM_MATH_H_
#define _IMM_MATH_H_

#include "imm_types.h"

IMM_BEGIN_DECLS

// set square matrix A to identity matrix with the rows of n
void eye (double *A, uint32_t n);

// matrix multiplication: C = alpha .*(A * B) + beta .* C 
void matmul (const char *tr, uint32_t n, uint32_t k, uint32_t m, double alpha,
	         const double *A, const double *B, double beta, double *C);

// matrix add: C = alpha * A + beta * B 
void matadd (double alpha,const double *A, double beta, const double *B, uint32_t n, uint32_t m,  double *C);

// matrix scale A = scale .* A
void matscale (double *A, double scale, uint32_t m, uint32_t n);

// matrix inverse. The original matrix is overwritten by the inverse matrix
bool matinv (double *A, uint32_t n);

// matrix copy: dest = src
void matcpy (double *dest, const double *src, uint32_t n, uint32_t m);

//matrix transpose
void mattranspose(double *dest, double *src, uint32_t n, uint32_t m);

// print the matrix p for debug
void matprint (double *p, uint32_t m, uint32_t n);

// get the eigen vector and eigenvalue for 2x2 covariance matrix
void eigenvector_2x2 (double *eigenvalues, double *eigenvector1, double *eigenvector2, const double *A);

int float_round_int32 (float val);


IMM_END_DECLS

#endif