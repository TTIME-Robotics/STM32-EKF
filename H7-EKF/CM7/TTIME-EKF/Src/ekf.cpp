/**
 * @file ekf.cpp
 * @brief Contains implementations for the generic EKF class
 * @author Daniel Dew
 * @version 0.1
 * @date 28.05.26
 */

#include "ekf.hpp"
#include "types.hpp"
using namespace EKF;

EK_filter::EK_filter(State_t init_state, const SquareMatrix<STATE_N>& init_cov)
	: state_estimate(init_state), state_covariance(init_cov) {}

int32_t predict(
		const State_t new_state,
		const SquareMatrix<STATE_N>& jac_F,
		const SquareMatrix<STATE_N>& process_noise,
		uint32_t timestamp
) {

}
