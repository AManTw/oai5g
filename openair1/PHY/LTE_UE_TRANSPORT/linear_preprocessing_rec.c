/*  These functions compute linear preprocessing for
    the UE using LAPACKE and CBLAS modules of
    LAPACK libraries.
    MMSE and MMSE whitening filters are available.
    Functions are using RowMajor storage of the
    matrices, like in conventional C. Traditional
    Fortran functions of LAPACK employ ColumnMajor
    data storage. */

#include<stdio.h>
#include<math.h>
#include<complex.h>
#include <stdlib.h>
#include <cblas.h>
#include <string.h>
#include <linux/version.h>
#if RHEL_RELEASE_CODE >= 1796
    #include <lapacke/lapacke_utils.h>
    #include <lapacke/lapacke.h>
#else
    #include <lapacke_utils.h>
    #include <lapacke.h>
#endif
//#define DEBUG_PREPROC


void transpose(int N, float complex *A, float complex *Result)
{
    // COnputes C := alpha*op(A)*op(B) + beta*C,
    enum CBLAS_TRANSPOSE transa = CblasTrans;
    enum CBLAS_TRANSPOSE transb = CblasNoTrans;
    int rows_opA = N; // number of rows in op(A) and in C
    int col_opB = N; //number of columns of op(B) and in C
    int col_opA = N; //number of columns in op(A) and rows in op(B)
    int col_B; //number of columns in B
    float complex alpha = 1.0 + I * 0;
    int lda  = rows_opA;
    float complex beta = 0.0 + I * 0;
    int ldc = rows_opA;
    int i;
    float complex *B;

    int ldb = col_opB;

    if(transb == CblasNoTrans)
    {
        B = (float complex *)calloc(ldb * col_opB, sizeof(float complex));
        col_B = col_opB;
    }
    else
    {
        B = (float complex *)calloc(ldb * col_opA, sizeof(float complex));
        col_B = col_opA;
    }
    float complex *C = (float complex *)malloc(ldc * col_opB * sizeof(float complex));

    for(i = 0; i < lda * col_B; i += N + 1)
    {
        B[i] = 1.0 + I * 0;
    }

    cblas_cgemm(CblasRowMajor, transa, transb, rows_opA, col_opB, col_opA, &alpha, A, lda, B, ldb, &beta, C, ldc);

    memcpy(Result, C, N * N * sizeof(float complex));

    free(B);
    free(C);
}


void conjugate_transpose(int N, float complex *A, float complex *Result)
{
    // Computes C := alpha*op(A)*op(B) + beta*C,
    enum CBLAS_TRANSPOSE transa = CblasConjTrans;
    enum CBLAS_TRANSPOSE transb = CblasNoTrans;
    int rows_opA = N; // number of rows in op(A) and in C
    int col_opB = N; //number of columns of op(B) and in C
    int col_opA = N; //number of columns in op(A) and rows in op(B)
    int col_B; //number of columns in B
    float complex alpha = 1.0 + I * 0;
    int lda  = rows_opA;
    float complex beta = 0.0 + I * 0;
    int ldc = rows_opA;
    int i;
    float complex *B;
    int ldb = col_opB;

    if(transb == CblasNoTrans)
    {
        B = (float complex *)calloc(ldb * col_opB, sizeof(float complex));
        col_B = col_opB;
    }
    else
    {
        B = (float complex *)calloc(ldb * col_opA, sizeof(float complex));
        col_B = col_opA;
    }
    float complex *C = (float complex *)malloc(ldc * col_opB * sizeof(float complex));

    for(i = 0; i < lda * col_B; i += N + 1)
    {
        B[i] = 1.0 + I * 0;
    }

    cblas_cgemm(CblasRowMajor, transa, transb, rows_opA, col_opB, col_opA, &alpha, A, lda, B, ldb, &beta, C, ldc);

    memcpy(Result, C, N * N * sizeof(float complex));

    free(B);
    free(C);
}

void H_hermH_plus_sigma2I(int N, int M, float complex *A, float sigma2, float complex *Result)
{
    //C := alpha*op(A)*op(B) + beta*C,
    enum CBLAS_TRANSPOSE transa = CblasConjTrans;
    enum CBLAS_TRANSPOSE transb = CblasNoTrans;
    int rows_opA = N; // number of rows in op(A) and in C
    int col_opB = N; //number of columns of op(B) and in C
    int col_opA = N; //number of columns in op(A) and rows in op(B)
    int col_C = N; //number of columns in B
    float complex alpha = 1.0 + I * 0;
    int lda  = col_opA;
    float complex beta = 1.0 + I * 0;
    int ldc = col_opA;
    int i;

    float complex *C = (float complex *)calloc(ldc * col_opB, sizeof(float complex));

    for(i = 0; i < lda * col_C; i += N + 1)
    {
        C[i] = sigma2 * (1.0 + I * 0);
    }

    cblas_cgemm(CblasRowMajor, transa, transb, rows_opA, col_opB, col_opA, &alpha, A, lda, A, lda, &beta, C, ldc);

    memcpy(Result, C, N * M * sizeof(float complex));
    free(C);

}

void HH_herm_plus_sigma2I(int M, int N, float complex *A, float sigma2, float complex *Result)
{
    //C := alpha*op(A)*op(B) + beta*C,
    enum CBLAS_TRANSPOSE transa = CblasNoTrans;
    enum CBLAS_TRANSPOSE transb = CblasConjTrans;
    int k = N; //number of columns in op(A) and rows in op(B),k
    float complex alpha = 1.0 + I * 0;
    int lda  = N;
    int ldb  = N;
    int ldc = M;
    int i;

    float complex *C = (float complex *)calloc(M * M, sizeof(float complex));

    for(i = 0; i < M * M; i += M + 1)
    {
        C[i] = 1.0 + I * 0;
    }

    cblas_cgemm(CblasRowMajor, transa, transb, M, M, k, &alpha, A, lda, A, ldb, &sigma2, C, ldc);

    memcpy(Result, C, M * M * sizeof(float complex));
    free(C);

}

void eigen_vectors_values(int N, float complex *A, float complex *Vectors, float *Values_Matrix)
{
    // This function computes ORTHONORMAL eigenvectors and eigenvalues of matrix A,
    // where Values_Matrix is a diagonal matrix of eigenvalues.
    // A=Vectors*Values_Matrix*Vectors'
    char jobz = 'V';
    char uplo = 'U';
    int order_A = N;
    int lda = N;
    int i;
    float *Values = (float *)malloc(sizeof(float) * 1 * N);

    LAPACKE_cheev(LAPACK_ROW_MAJOR, jobz, uplo, order_A, A, lda, Values);

    memcpy(Vectors, A, N * N * sizeof(float complex));

    for(i = 0; i < lda; i += 1)
    {
        Values_Matrix[i * (lda + 1)] = Values[i];
    }

    free(Values);
}

void lin_eq_solver(int N, float complex *A, float complex *B, float complex *Result)
{
    int n = N;
    int lda = N;
    int ldb = N;
    int nrhs = N;

    char transa = 'N';
    int *IPIV = malloc(N * N * sizeof(int));

    // Compute LU-factorization
    LAPACKE_cgetrf(LAPACK_ROW_MAJOR, n, nrhs, A, lda, IPIV);

    // Solve AX=B
    LAPACKE_cgetrs(LAPACK_ROW_MAJOR, transa, n, nrhs, A, lda, IPIV, B, ldb);

    // cgetrs( "N", N, 4, A, lda, IPIV, B, ldb, INFO )

    memcpy(Result, B, N * N * sizeof(float complex));

    free(IPIV);

}

void mutl_matrix_matrix_row_based(float complex *M0, float complex *M1, int rows_M0, int col_M0, int rows_M1, int col_M1, float complex *Result)
{
    enum CBLAS_TRANSPOSE transa = CblasNoTrans;
    enum CBLAS_TRANSPOSE transb = CblasNoTrans;
    int rows_opA = rows_M0; // number of rows in op(A) and in C
    int col_opB = col_M1; //number of columns of op(B) and in C
    int col_opA = col_M0; //number of columns in op(A) and rows in op(B)
    float complex alpha = 1.0;
    int lda  = col_M0;
    float complex beta = 0.0;
    int ldc = col_M1;
    int ldb = col_M1;

#ifdef DEBUG_PREPROC
    int i = 0;
    printf("rows_M0 %d, col_M0 %d, rows_M1 %d, col_M1 %d\n", rows_M0, col_M0, rows_M1, col_M1);

    for(i = 0; i < rows_M0 * col_M0; ++i)
    {
        printf(" rows_opA = %d, col_opB = %d, W_MMSE[%d] = (%f + i%f)\n", rows_opA, col_opB,  i, creal(M0[i]), cimag(M0[i]));
    }

    for(i = 0; i < rows_M1 * col_M1; ++i)
    {
        printf(" M1[%d] = (%f + i%f)\n", i, creal(M1[i]), cimag(M1[i]));
    }
#endif

    cblas_cgemm(CblasRowMajor, transa, transb, rows_opA, col_opB, col_opA, &alpha, M0, lda, M1, ldb, &beta, Result, ldc);

#ifdef DEBUG_PREPROC
    for(i = 0; i < rows_opA * col_opB; ++i)
    {
        printf(" result[%d] = (%f + i%f)\n", i, creal(Result[i]), cimag(Result[i]));
    }
#endif

}
void mutl_matrix_matrix_col_based(float complex *M0, float complex *M1, int rows_M0, int col_M0, int rows_M1, int col_M1, float complex *Result)
{
    enum CBLAS_TRANSPOSE transa = CblasNoTrans;
    enum CBLAS_TRANSPOSE transb = CblasNoTrans;
    int rows_opA = rows_M0; // number of rows in op(A) and in C
    int col_opB = col_M1; //number of columns of op(B) and in C
    int col_opA = col_M0; //number of columns in op(A) and rows in op(B)
    float complex alpha = 1.0;
    int lda  = col_M0;
    float complex beta = 0.0;
    int ldc = rows_M1;
    int ldb = rows_M1;

#ifdef DEBUG_PREPROC
    int i = 0;
    printf("rows_M0 %d, col_M0 %d, rows_M1 %d, col_M1 %d\n", rows_M0, col_M0, rows_M1, col_M1);

    for(i = 0; i < rows_M0 * col_M0; ++i)
    {
        printf(" rows_opA = %d, col_opB = %d, W_MMSE[%d] = (%f + i%f)\n", rows_opA, col_opB,  i, creal(M0[i]), cimag(M0[i]));
    }


    for(i = 0; i < rows_M1 * col_M1; ++i)
    {
        printf(" M1[%d] = (%f + i%f)\n", i, creal(M1[i]), cimag(M1[i]));
    }
#endif

    cblas_cgemm(CblasColMajor, transa, transb, rows_opA, col_opB, col_opA, &alpha, M0, lda, M1, ldb, &beta, Result, ldc);

#ifdef DEBUG_PREPROC
    for(i = 0; i < rows_opA * col_opB; ++i)
    {
        printf(" result[%d] = (%f + i%f)\n", i, creal(Result[i]), cimag(Result[i]));
    }
#endif
}


/*FILTERS */
void compute_MMSE(float complex *H, int order_H, float sigma2, float complex *W_MMSE)
{
    int N = order_H;
    float complex *H_hermH_sigmaI = malloc(N * N * sizeof(float complex));
    float complex *H_herm =  malloc(N * N * sizeof(float complex));

    H_hermH_plus_sigma2I(N, N, H, sigma2, H_hermH_sigmaI);

#ifdef DEBUG_PREPROC
    int i = 0;
    for(i = 0; i < N * N; i++)
    {
        printf(" H_hermH_sigmaI[%d] = (%f + i%f)\n", i, creal(H_hermH_sigmaI[i]), cimag(H_hermH_sigmaI[i]));
    }
#endif

    conjugate_transpose(N, H, H_herm);  //equals H_herm

#ifdef DEBUG_PREPROC
    for(i = 0; i < N * N; i++)
    {
        printf(" H_herm[%d] = (%f + i%f)\n", i, creal(H_herm[i]), cimag(H_herm[i]));
    }
#endif

    lin_eq_solver(N, H_hermH_sigmaI, H_herm, W_MMSE);

#ifdef DEBUG_PREPROC
    for(i = 0; i < N * N; i++)
    {
        printf(" W_MMSE[%d] = (%f + i%f)\n", i, creal(W_MMSE[i]), cimag(W_MMSE[i]));
    }
#endif

    free(H_hermH_sigmaI);
    free(H_herm);
}


float sqrt_float(float x, float sqrt_x)
{
    sqrt_x = (float)(sqrt((double)(x)));
    return sqrt_x;
}
