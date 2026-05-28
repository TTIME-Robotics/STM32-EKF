/**
 * @file ekf.hpp
 * @brief contains definitions for the generic EKF class
 * @author Daniel Dew
 * @version 0.1
 * @date 28.05.26
 */

#ifndef EKF_EKF_HPP
#define EKF_EKF_HPP

#include "types.hpp"
#include "matrix_maths.hpp"
#include <cstdint>

namespace EKF {

class EK_filter {
public:
	/**
	 * @brief Constructs the filter with initial values
 	 * @param init_state The initial estimate
	 * @param init_cov The initial covariance (uncertainty in initial state)
	 */
	EK_filter(State_t init_state, const SquareMatrix<STATE_N>& init_cov);
	/**
	 * @brief Trivial destructor
	 */
	~EK_filter() = default;

	/**
	 * @brief Change state for prediction and propogate covariance
	 * @param new_state the new state to update to
	 * @param jac_F The square jacobian matrix to propogate covariance
	 * @param process_noise The square noise matrix to add to covariance estimate
	 * @param timestamp The current tick to be stored as estimate timestamp
	 * @return EKF_SUCCESS if successful, EKF_ERR if not
	 */
	int32_t predict(
			const State_t new_state,
			const SquareMatrix<STATE_N>& jac_F,
			const SquareMatrix<STATE_N>& process_noise,
			uint32_t timestamp
	);
	/**
	 * @brief Update the state from a measurement
	 * @tparam MEAS_DIM The dimension of measurement matrix (Z)
	 * @param innovation The innovation from the update (y)
	 * @param jac_H The jacobian to propogate noise (H)
	 * @param sensor_cov The noise to add to the covariance update (R)
	 * @param timestamp The current tick to be stored as estimate timestep
	 * @return EKF_SUCCESS if successful, EKF_ERR if not
	 */
	template<int MEAS_DIM>
	int32_t update(
			const Matrix<MEAS_DIM, 1>& innovation,
			const Matrix<MEAS_DIM,STATE_N>& jac_H,
			const SquareMatrix<MEAS_DIM>& sensor_noise,
			uint32_t timestamp
	);

	/**
	 * @brief Set the state of the filter
	 * @param state State to set to
	 * @param state_cov The covariance of this state
	 */
	void set_state(const State_t state, const SquareMatrix<STATE_N>& state_cov);

	/**
	 * @brief Get the state
	 * @return The state
	 */
	State_t get_state() const;
	/**
	 * @brief Get the pose (position, heading)
	 * @return The pose
	 */
	Pose_t get_pose() const;
private:
	State_t state_estimate; // x, current state estimate
	SquareMatrix<STATE_N> state_covariance; // P, current state covariance
	uint32_t estimate_timestamp; // The tick when estimate was last updated/predicted
};

}

#endif // EKF_EKF_HPP
