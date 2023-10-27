/*****************************************************************************
*    Copyright (c) 2005 Spirit Corp.
******************************************************************************/
/**
*    @project                PDR project
*    @brief                  matrix operations
*    @file                   matrix.h
*    @author                 A. Baloyan, M. Zhokhova
*    @date                   23.04.2013
*    @version                2.1
*
******************************************************************************/
#ifndef _MATRIX_H_
#define _MATRIX_H_

#pragma once

#ifdef _MSC_VER
 #if  _MSC_VER < 1600
  #include "stdint.h"
 #else 
  #include <stdint.h>
 #endif
#else
 #include <stdint.h>
#endif

#ifdef  __cplusplus
#define FPPE_BEGIN_DECLS extern "C" {
#define FPPE_END_DECLS    }
#else
#define FPPE_BEGIN_DECLS
#define FPPE_END_DECLS
#endif

FPPE_BEGIN_DECLS

#define MATRIX_MAX_N     8
#define MATRIX_MAX_M     8

typedef struct
{ 
  double Matr[MATRIX_MAX_N][MATRIX_MAX_M];
  int8_t mRaw, mCol; // row, column
}
Matrix;

// ------------------------------------------------------------------------
///  @brief     Matrix initialization
///
///  @param[in] A - pointer to instance
///  @param[in] N - row
///  @param[in] M - column
// ------------------------------------------------------------------------
extern void CreateMatrix(Matrix *A, int8_t N, int8_t M);

// ------------------------------------------------------------------------
///  @brief     Free matrix dynamic allocation memory
///
///  @param[in] A - pointer to instance
// ------------------------------------------------------------------------
extern void DestroyMatrix(Matrix* A);

// ------------------------------------------------------------------------
///  @brief     Matrix copy A = B
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
// ------------------------------------------------------------------------
extern void CopyMatrix(Matrix *A, const Matrix *B);

// ------------------------------------------------------------------------
///  @brief     Make unitary Matrix A = I(N,N)
///
///  @param[in] A - pointer to instance
///  @param[in] N - row, column
// ------------------------------------------------------------------------
extern void    UnitaryMatrix(Matrix *A, int8_t N);

// ------------------------------------------------------------------------
///  @brief     QR-decomposition of the matrix A
///             Resultant matrices are contained in R and Q correspondly
///
///  @param[in] A - pointer to instance
///  @param[in] Q - pointer to instance
///  @param[in] R - pointer to instance
///  @return 0 - error, 1 - successfully
// ------------------------------------------------------------------------
extern int8_t QR_Decomposition(Matrix *A, Matrix *Q, Matrix *R);

// ------------------------------------------------------------------------
///  @brief     Multiplication of matrices A = T(B)*C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
extern void    MultMatrixT_(Matrix *A, const Matrix *B, Matrix *C);

// ------------------------------------------------------------------------
///  @brief     Multiplication of matrices A = B*T(C)
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
extern void    MultMatrix2T_(Matrix *A, Matrix *B, Matrix *C);

// ------------------------------------------------------------------------
///  @brief     Multiplication of matrices A = B*C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
extern void    MultMatrix(Matrix *A, Matrix *B, Matrix *C);

// ------------------------------------------------------------------------
///  @brief     Addition of matrices A = B+C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
extern void    SumMatrix(Matrix *A, Matrix *B, Matrix *C);

// ------------------------------------------------------------------------
///  @brief     Difference of matrices A = B-C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
extern void    DiffMatrix(Matrix *A, Matrix *B, Matrix *C);

// ------------------------------------------------------------------------
///  @brief     Solution by linear system X=R*Y (i.e. Y=Inv(R)*X)
///             where R is an upper triangular matrix
///
///  @param[in] Y - pointer to instance
///  @param[in] R - pointer to instance
///  @param[in] X - pointer to instance
///  @return 0 - error, 1 - successfully
// ------------------------------------------------------------------------
extern int8_t  UppTriSolution(Matrix *Y, Matrix *R, Matrix *X);

// --------------------------------------------------------------------------------------
///  @brief     Computation of a matrix G=Inv(R)*Q where R is an upper triangular matrix
///
///  @param[in] G - pointer to instance
///  @param[in] R - pointer to instance
///  @param[in] Q - pointer to instance
// ---------------------------------------------------------------------------------------
extern void    UppTriSolModif(Matrix *G, Matrix *R, Matrix *Q);

// ------------------------------------------------------------------------
///  @brief     Computation of the inverse Matrix A = Inv(B)
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @return 0 - error, 1 - successfully
// ------------------------------------------------------------------------
extern int8_t   InverseMatrix(Matrix *A, Matrix *B);

// --------------------------------------------------------------------------------------
///  @brief    Rough computation of a matrix expm(A) where A is a square matrix (m x m)
///		    RoughExpM (A) = I(m x m) + A + 1/2 * A * A 
///				where I(m x m) is an identity square matrix (m x m)
///
///  @param[in] A - pointer to instance
// ---------------------------------------------------------------------------------------
extern void RoughExpM(Matrix *E, Matrix *A);

// --------------------------------------------------------------------------------------
///  @brief    Creates skew-symmetric matrix K, where K is a square matrix (3 x 3)
///
///  @param[in] K - pointer to instance
///  @param[in] vec_x - vector x component
///  @param[in] vec_y - vector y component
///  @param[in] vec_z - vector z component
// ---------------------------------------------------------------------------------------
extern void Skew_Symmetric(Matrix *K, double vec_x, double vec_y, double vec_z);

FPPE_END_DECLS
#endif // _MATRIX_H_