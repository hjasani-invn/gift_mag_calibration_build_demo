/*****************************************************************************
*    Copyright (c) 2005 Spirit Corp.
******************************************************************************/
/**
*    @project                PDR project
*    @brief                  matrix operations
*    @file                   matrix.cpp
*    @author                 A. Baloyan, M. Zhokhova
*    @date                   23.04.2013
*    @version                2.1
*
******************************************************************************/

#include <math.h>
#include <memory.h>
#include <stdlib.h>
#include "matrix.h"

// ------------------------------------------------------------------------
///  @brief     Matrix initialization
///
///  @param[in] A - pointer to instance
///  @param[in] N - row
///  @param[in] M - column
// ------------------------------------------------------------------------
void CreateMatrix(Matrix *A, int8_t N, int8_t M)
{ 
  A->mRaw = N;
  A->mCol = M;
  memset(A->Matr, 0, sizeof(A->Matr));
  return;
}

// ------------------------------------------------------------------------
///  @brief     Free matrix dynamic allocation memory
///
///  @param[in] A - pointer to instance
// ------------------------------------------------------------------------
void DestroyMatrix(Matrix* A)
{ 
  A->mRaw = 0;
  A->mCol = 0;
  
  return;
}

// ------------------------------------------------------------------------
///  @brief     Matrix copy A = B
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
// ------------------------------------------------------------------------
void CopyMatrix(Matrix *A, const Matrix *B)
{ 
  int8_t i, j, N = B->mRaw, M = B->mCol;
  
  if(A->mRaw != N || A->mCol != M) 
    CreateMatrix(A,N,M);
  
  if(A != B)
    for(i = 0; i < N; i++)
      for(j = 0; j < M; j++)
        A->Matr[i][j] = B->Matr[i][j];
 	
  return;
}

// ------------------------------------------------------------------------
///  @brief     Make unitary Matrix A = I(N,N)
///
///  @param[in] A - pointer to instance
///  @param[in] N - row, column
// ------------------------------------------------------------------------
void UnitaryMatrix(Matrix *A, int8_t N)
{ 
  int8_t i;

  CreateMatrix(A,N,N);
	memset(A->Matr, 0, sizeof(A->Matr));

  for(i = 0; i < N; i++) 
    A->Matr[i][i] = 1.;
  
  return;
}

// ------------------------------------------------------------------------
///  @brief     QR-decomposition of the matrix A
///             Resultant matrices are contained in R and Q correspondly
///
///  @param[in] A - pointer to instance
///  @param[in] Q - pointer to instance
///  @param[in] R - pointer to instance
///  @return 0 - error, 1 - successfully
// ------------------------------------------------------------------------
int8_t QR_Decomposition(Matrix *A, Matrix *Q, Matrix *R)
{ 
  int8_t i, j, k;
  int8_t N = A->mRaw, M = A->mCol;
  double temp;
  
  CopyMatrix(Q,A); 
  CreateMatrix(R,M,M);
  
  for(k = 0; k < M; k++)
  { 
    for(i = 0, temp = 0.; i < N; i++) 
      temp += Q->Matr[i][k] * Q->Matr[i][k];
		
    R->Matr[k][k] = sqrt(temp);
    if( R->Matr[k][k] != 0)
      temp = 1./R->Matr[k][k];
    else
    {
      DestroyMatrix(Q);
      DestroyMatrix(R);
      return 0;
    }
		
    for(i = 0; i < N; i++) 
      Q->Matr[i][k] *= temp;
		
    for(j = k + 1; j < M; j++)
    {	
      for(i = 0, temp = 0.; i < N; i++) 
        temp += Q->Matr[i][k] * Q->Matr[i][j];
			
      for(i = 0; i < N; i++) 
        Q->Matr[i][j] -= temp * Q->Matr[i][k];
			
      R->Matr[k][j] = temp;
    }
  }
	return 1;
}

// ------------------------------------------------------------------------
///  @brief     Multiplication of matrices A = B*C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
void MultMatrix(Matrix *A, Matrix *B, Matrix *C)
{ 
  double K;
  int8_t i, j, l;
  Matrix D, *Ptr;
  
  D.mCol = 0;
  D.mRaw = 0;
  
  if(B->mCol != C->mRaw)
  {
    DestroyMatrix(A);
    return;
  }
  
  Ptr = ((A == B) || (A == C)) ? &D : A;
  CreateMatrix(Ptr,B->mRaw,C->mCol);
  for(i = 0; i < B->mRaw; i++)
    for(j = 0; j < C->mCol; j++)
    { 
      for(l = 0, K = 0; l < B->mCol; l++)
	    K += B->Matr[i][l] * C->Matr[l][j];

	  Ptr->Matr[i][j] = K;
    }

  if((A == B) || (A == C)) 
  { 
    CopyMatrix(A,&D);
    DestroyMatrix(&D); 
  }

  return;
}

// ------------------------------------------------------------------------
///  @brief     Multiplication of matrices A = T(B)*C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
void MultMatrixT_(Matrix *A, const Matrix *B, Matrix *C)
{ 
  double K;
  int8_t i, j, l;
  Matrix D, *Ptr;
  
  D.mCol = 0;
  D.mRaw = 0;
  
  if(B->mRaw != C->mRaw)
  {
    DestroyMatrix(A);
    return;
  }

  Ptr = ((A == B) || (A == C)) ? &D : A;
  CreateMatrix(Ptr,B->mCol,C->mCol);

  for(i = 0; i < B->mCol; i++)
    for(j = 0; j < C->mCol; j++)
    { 
      for(l = 0, K = 0; l < B->mRaw; l++)
      K += B->Matr[l][i] * C->Matr[l][j];
	    
      Ptr->Matr[i][j] = K;
    }
  
  if((A == B) || (A == C)) 
  { 
    CopyMatrix(A,&D);
    DestroyMatrix(&D); 
  }
  
  return;
}

// ------------------------------------------------------------------------
///  @brief     Multiplication of matrices A = B*T(C)
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
void    MultMatrix2T_(Matrix *A, Matrix *B, Matrix *C)
{
  double K;
  int8_t i, j, l;
  Matrix D, *Ptr;
  
  D.mCol = 0;
  D.mRaw = 0;
  
  if(B->mCol != C->mCol)
  {
    DestroyMatrix(A);
    return;
  }

  Ptr = ((A == B) || (A == C)) ? &D : A;
  CreateMatrix(Ptr, B->mRaw, C->mRaw);

  for(i = 0; i < B->mRaw; i++)
  for(j = 0; j < C->mRaw; j++)
  { 
    for(l = 0, K = 0; l < B->mCol; l++)
      K += B->Matr[i][l] * C->Matr[j][l];
	    
    Ptr->Matr[i][j] = K;
  }
  
  if((A == B) || (A == C)) 
  { 
    CopyMatrix(A,&D);
    DestroyMatrix(&D); 
  }
  
  return;
}

// ------------------------------------------------------------------------
///  @brief     Addition of matrices A = B+C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
void SumMatrix(Matrix *A, Matrix *B, Matrix *C)
{
  int8_t i, j;
  Matrix D, *Ptr;
  
  D.mCol = 0;
  D.mRaw = 0;
  
  if(B->mCol != C->mCol)
  {
    DestroyMatrix(A);
    return;
  }
  if(B->mRaw != C->mRaw)
  {
    DestroyMatrix(A);
    return;
  }

  Ptr = ((A == B) || (A == C)) ? &D : A;
  CreateMatrix(Ptr, B->mRaw, B->mCol);

  for(i = 0; i < B->mRaw; i++)
    for(j = 0; j < B->mCol; j++)
			Ptr->Matr[i][j] = B->Matr[i][j] + C->Matr[i][j];

  if((A == B) || (A == C)) 
  { 
    CopyMatrix(A,&D);
    DestroyMatrix(&D); 
  }
  
  return;
}

// ------------------------------------------------------------------------
///  @brief     Difference of matrices A = B-C
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @param[in] C - pointer to instance
// ------------------------------------------------------------------------
void DiffMatrix(Matrix *A, Matrix *B, Matrix *C)
{
  int8_t i, j;
  Matrix D, *Ptr;
  
  D.mCol = 0;
  D.mRaw = 0;
  
  if(B->mCol != C->mCol)
  {
    DestroyMatrix(A);
    return;
  }
  if(B->mRaw != C->mRaw)
  {
    DestroyMatrix(A);
    return;
  }

  Ptr = ((A == B) || (A == C)) ? &D : A;
  CreateMatrix(Ptr, B->mRaw, B->mCol);

  for(i = 0; i < B->mRaw; i++)
    for(j = 0; j < B->mCol; j++)
      Ptr->Matr[i][j] = B->Matr[i][j] - C->Matr[i][j];

  if((A == B) || (A == C)) 
  { 
    CopyMatrix(A,&D);
    DestroyMatrix(&D); 
  }
  
  return;
}


// ------------------------------------------------------------------------
///  @brief     Computation of the inverse Matrix A = Inv(B)
///
///  @param[in] A - pointer to instance
///  @param[in] B - pointer to instance
///  @return 0 - error, 1 - successfully
// ------------------------------------------------------------------------
int8_t InverseMatrix(Matrix *A, Matrix *B)
{ 
  int8_t i, j, l = 0, q, N = B->mRaw;
  double C[MATRIX_MAX_M][MATRIX_MAX_M], D[MATRIX_MAX_M][MATRIX_MAX_M];
  double K, S;
  

  if(B->mRaw != B->mCol)
  {
    DestroyMatrix(A);
    return 0;
  }
  
  for(i = 0; i < N; i++)
  { 
    for(j = 0; j < N; j++)
    { 
      C[i][j] = B->Matr[j][i];
      D[i][j] = 0;
    } 
    
    D[i][i] = 1;
  }

  for(i = 0; i < N - 1; i++)
  { 
    for(j = i, K = 0; j < N; j++)
	{ 
      S = fabs(C[i][j]);
      if(S > K) 
      { 
        l = j; 
        K = S;
      }
    }
    if(!K)
    {
      DestroyMatrix(A);
      return 0;
    }
    if(l != i)
	  for(q = 0; q < N; q++)
	  { 
        S = C[q][i];
        C[q][i] = C[q][l];
        C[q][l] = S;
	      S = D[q][i];
        D[q][i] = D[q][l];
        D[q][l] = S;
	  }

    for(j = i + 1 ; j < N; j++)
    { 
      K = C[i][j] / C[i][i];
	  if(K)
	  { 
      for(l = i; l < N; l++) 
        C[l][j] -= K * C[l][i];
	      
      for(l = 0; l < N; l++) 
        D[l][j] -= K * D[l][i];
	  }
    }
  }

  for(i = N - 1; i > 0; i--)
  { 
    for(j = i - 1; j >= 0; j--)
    { 
      K = C[i][j] / C[i][i];
	  if(K)
	  { 
        C[i][j] -= K * C[i][i];
	      
        for(l = 0; l < N; l++) 
          D[l][j] -= K * D[l][i];
	  }
    }
  }

  for(i = 0; i < N; i++)
  { 
    K = C[i][i];
    for(j = 0; j < N; j++)
      D[j][i] /= K;
  }
  
  CreateMatrix(A,N,N);
  
  for(i = 0; i < N; i++)
  { 
    for(j = 0; j < N; j++)
      A->Matr[j][i] = D[i][j];
  }
  
  return 1;
}

// ------------------------------------------------------------------------
///  @brief     Solution by linear system X=R*Y (i.e. Y=Inv(R)*X)
///             where R is an upper triangular matrix
///
///  @param[in] Y - pointer to instance
///  @param[in] R - pointer to instance
///  @param[in] X - pointer to instance
///  @return 0 - error, 1 - successfully
// ------------------------------------------------------------------------
int8_t UppTriSolution(Matrix *Y, Matrix *R, Matrix *X)
{ 
  int8_t i, j;
  double temp;

  if(R->mRaw != R->mCol)
  {
    DestroyMatrix(Y);
    return 0;
  }
  
  if(R->mRaw != X->mRaw)
  {
    DestroyMatrix(Y);
    return 0;
  }
  
  CopyMatrix(Y,X);
	
  for(i = Y->mRaw - 1; i >= 0; i--)
  { 
    for(j = Y->mRaw - 1,temp = 0.; j > i; j--)
    {
      temp += R->Matr[i][j] * Y->Matr[j][0];
    }
    Y->Matr[i][0] -= temp; 
    if(R->Matr[i][i] != 0)
      Y->Matr[i][0] /= R->Matr[i][i];
    else
    {
      DestroyMatrix(Y);
      return 0;
    }
  }
  return 1;
}

// --------------------------------------------------------------------------------------
///  @brief     Computation of a matrix G=Inv(R)*Q where R is an upper triangular matrix
///
///  @param[in] G - pointer to instance
///  @param[in] R - pointer to instance
///  @param[in] Q - pointer to instance
// ---------------------------------------------------------------------------------------
void UppTriSolModif(Matrix *G, Matrix *R, Matrix *Q)
{ 
  int8_t i, j, k;
  double temp;

  if(R->mRaw != R->mCol)
  {
    DestroyMatrix(G);
    return;
  }
  if(R->mRaw != Q->mRaw)
  {
    DestroyMatrix(G);
    return;
  }

  CopyMatrix(G,Q);
  for(k = 0; k < G->mCol; k++)
  { 
    for (i = G->mRaw - 1; i >= 0; i--)
    { 
      for (j = G->mRaw - 1, temp = 0.; j > i; j--)
      {
        temp += R->Matr[i][j] * G->Matr[j][k];
      }
      G->Matr[i][k] -= temp;
      G->Matr[i][k] /= R->Matr[i][i];
	}
  } 
  return;
}

// --------------------------------------------------------------------------------------
///  @brief    Rough computation of a matrix expm(A) where A is a square matrix (m x m)
///		    RoughExpM (A) = I(m x m) + A + 1/2 * A * A 
///				where I(m x m) is an identity square matrix (m x m)
///
///  @param[in] E - pointer to instance
///  @param[in] A - pointer to instance
// ---------------------------------------------------------------------------------------
void RoughExpM(Matrix *E, Matrix *A)
{ 
	int8_t i, j, N = A->mRaw;

	Matrix A_sqr;
	Matrix I;
	
	A_sqr.mCol = N;
	A_sqr.mRaw = N;

	I.mCol = N;
	I.mRaw = N;
  
  if(A->mRaw != A->mCol)
  {
    DestroyMatrix(A);
    return;
  }

	if(E->mRaw != E->mCol)
  {
    DestroyMatrix(E);
    return;
  }

	CreateMatrix(&A_sqr, N, N);
	MultMatrix(&A_sqr, A, A);

	UnitaryMatrix(&I, A->mRaw);
		 
  for(i = 0; i < N; i++)
  { 
		for(j = 0; j < N; j++)
    { 
			E->Matr[i][j] = I.Matr[i][j] + A->Matr[i][j] + 0.5 * A_sqr.Matr[i][j];
		} 
	}
	return;
}

// --------------------------------------------------------------------------------------
///  @brief    Creates skew-symmetric matrix K, where K is a square matrix (3 x 3)
///
///  @param[in] K - pointer to instance
///  @param[in] vec_x - vector x component
///  @param[in] vec_y - vector y component
///  @param[in] vec_z - vector z component
// ---------------------------------------------------------------------------------------
void Skew_Symmetric(Matrix *K, double vec_x, double vec_y, double vec_z)
{ 
	//int8_t i, j, N = K->mRaw;

	if ((K->mRaw != 3) || ( K->mCol != 3))
	{
		DestroyMatrix(K);
		return;
	}
	
	K->Matr[0][0] = 0; K->Matr[0][1] = -vec_z; K->Matr[0][2] = vec_y; 
	K->Matr[1][0] = vec_z; K->Matr[1][1] = 0; K->Matr[1][2] = -vec_x; 
	K->Matr[2][0] = -vec_y; K->Matr[2][1] = vec_x; K->Matr[2][2] = 0; 
}
