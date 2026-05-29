/**
 * @file matrix_maths.hpp
 * @brief contains definitions for EKF-related matrix maths
 * @author Daniel Dew
 * @version 0.1
 * @date 28.05.26
 */

#ifndef EKF_MAT_MATHS_HPP
#define EKF_MAT_MATHS_HPP

namespace EKF{
template<int ROWS, int COLS>
struct Matrix {
	float data[ROWS*COLS] = {};

	float& operator()(int r, int c) { return data[r * COLS + c]; }
	float  operator()(int r, int c) const { return data[r * COLS + c]; }
};
template<int N>
using SquareMatrix = Matrix<N,N>;
template<int N>
struct Vector : public Matrix<N,1> {
	float& operator()(int n) { return data[n]; }
	float  operator()(int n) const { return data[n]; }
};


template<int ROWS, int COLS>
Matrix<ROWS,COLS> mat_add(const Matrix<ROWS,COLS>& mat_a, const Matrix<ROWS,COLS>& mat_b);

template<int ROWS, int COLS>
Matrix<ROWS,COLS> mat_sub(const Matrix<ROWS,COLS>& mat_a, const Matrix<ROWS,COLS>& mat_b);

template<int M, int N, int P>
Matrix<M,P> mat_mult(const Matrix<M,N>& mat_a, const Matrix<N,P>& mat_b);

template<int ROWS, int COLS>
Matrix<COLS,ROWS> mat_transpose(const Matrix<ROWS,COLS>& mat);

template<int N>
SquareMatrix<N> cholesky(const SquareMatrix<N>& A);

template<int N>
Matrix<N,1> forward_sub(const SquareMatrix<N>& L, const Matrix<N,1>& b);

template<int N>
Matrix<N,1> back_sub(const SquareMatrix<N>& L, const Matrix<N,1>& y);

template<int N>
SquareMatrix<N> mat_inv_spd(const SquareMatrix<N>& A);


template<int M, int N>
SquareMatrix<M> propagate_covariance(const Matrix<M,N>& jac, const SquareMatrix<N>& cov);

template<int N>
SquareMatrix<N> Identity();

}
#endif // EKF_MAT_MATHS_HPP
