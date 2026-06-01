#include "error.h"

using namespace EKF;

template<int MEAS_DIM>
int32_t EK_filter::update(
		const Vector<MEAS_DIM>& innovation,
		const Matrix<MEAS_DIM,STATE_N>& jac_H,
		const SquareMatrix<MEAS_DIM>& sensor_noise,
		uint32_t timestamp
) {
	SquareMatrix<MEAS_DIM> innov_cov = mat_add(propagate_covariance(jac_H, state_covariance), sensor_noise);
	Matrix<STATE_N, MEAS_DIM> kalman_gain = compute_kalman_gain(state_covariance, innov_cov, jac_H);

	Matrix<STATE_N,1> state_mat = get_state_mat();
	state_mat = mat_add(state_mat, mat_mult(kalman_gain, innovation));

	SquareMatrix<STATE_N> new_cov =
			mat_mult(mat_sub(Identity<STATE_N>(), mat_mult(kalman_gain, jac_H)), state_covariance);

	set_state(state_mat, new_cov);
	estimate_timestamp = timestamp;
	last_update_timestamp = timestamp;

	return EKF_SUCCESS;
}