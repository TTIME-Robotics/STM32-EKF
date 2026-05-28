/**
 * @file matrix_maths.cpp
 * @brief Contains implementations of EKF-related matrix maths
 * @author Daniel Dew
 */

#include "matrix_maths.hpp"
#include <cstdint>
using namespace EKF;

template<int ROWS, int COLS>
Matrix<ROWS,COLS> mat_add(const Matrix<ROWS,COLS>& mat_a, const Matrix<ROWS,COLS>& mat_b){
	Matrix<ROWS,COLS> mat_out;
	for (uint32_t r=0; r<ROWS; ++r) {
		for (uint32_t c=0; c<COLS; ++c) {
			mat_out(r,c) = mat_a(r,c) + mat_b(r,c);
		}
	}
	return mat_out;
}

template<int M, int N, int P>
Matrix<M, P> mat_mult(const Matrix<M, N>& mat_a, const Matrix<N, P>& mat_b)
{
    Matrix<M, P> mat_out;

    for (int i = 0; i < M; i++) {
        for (int j = 0; j < P; j++) {
            float sum = 0.0f;
            for (int k = 0; k < N; k++) {
                sum += mat_a(i, k) * mat_b(k, j);
            }
            mat_out(i, j) = sum;
        }
    }

    return mat_out;
}

template<int ROWS, int COLS>
Matrix<COLS,ROWS> mat_transpose(Matrix<ROWS,COLS>& mat) {
	Matrix<COLS, ROWS> mat_out;
	for (uint32_t r=0; r<ROWS; ++r) {
		for (uint32_t c=0; c<COLS; ++c) {
			mat_out(c,r) = mat(r,c);
		}
	}
	return mat_out;
}

template<int N>
SquareMatrix<N> cholesky(const SquareMatrix<N>& A)
{
    SquareMatrix<N> L = {};

    for (int i = 0; i < N; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0f;
            for (int k = 0; k < j; k++)
                sum += L(i,k) * L(j,k);

            if (i == j)
                L(i,j) = sqrtf(A(i,i) - sum);
            else
                L(i,j) = (A(i,j) - sum) / L(j,j);
        }
    }
    return L;
}

template<int N>
Matrix<N,1> forward_sub(const SquareMatrix<N>& L, const Matrix<N,1>& b)
{
    Matrix<N,1> y = {};
    for (int i = 0; i < N; i++) {
        float sum = 0.0f;
        for (int k = 0; k < i; k++)
            sum += L(i,k) * y(k,0);
        y(i,0) = (b(i,0) - sum) / L(i,i);
    }
    return y;
}

template<int N>
Matrix<N,1> back_sub(const SquareMatrix<N>& L, const Matrix<N,1>& y)
{
    Matrix<N,1> x = {};
    for (int i = N-1; i >= 0; i--) {
        float sum = 0.0f;
        for (int k = i+1; k < N; k++)
            sum += L(k,i) * x(k,0);  // L^T(i,k) = L(k,i)
        x(i,0) = (y(i,0) - sum) / L(i,i);
    }
    return x;
}

template<int N>
SquareMatrix<N> mat_inv_spd(const SquareMatrix<N>& A)
{
    SquareMatrix<N> L = cholesky(A);
    SquareMatrix<N> inv = {};

    for (int col = 0; col < N; col++) {
        // Build unit vector e_col
        Matrix<N,1> e = {};
        e(col, 0) = 1.0f;

        // Two triangular solves
        Matrix<N,1> y = forward_sub(L, e);
        Matrix<N,1> x = back_sub(L, y);

        // x is column col of the inverse
        for (int row = 0; row < N; row++)
            inv(row, col) = x(row, 0);
    }

    return inv;
}
