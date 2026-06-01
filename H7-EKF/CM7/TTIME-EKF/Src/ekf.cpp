/**
 * @file ekf.cpp
 * @brief Contains implementations for the generic EKF class
 * @author Daniel Dew
 * @version 0.1
 * @date 28.05.26
 */

#include "ekf.hpp"
#include "types.hpp"
#include "matrix_maths.hpp"
#include "error.h"
#include <math.h>
using namespace EKF;

EK_filter::EK_filter(State_t init_state, const SquareMatrix<STATE_N>& init_cov, uint32_t time)
	: state_estimate(init_state), state_covariance(init_cov),
	  last_predict_timestamp(time), last_update_timestamp(time),
	  estimate_timestamp (time) {}

int32_t EK_filter::predict(
		const State_t new_state,
		const SquareMatrix<STATE_N>& jac_F,
		const SquareMatrix<STATE_N>& process_noise,
		uint32_t timestamp
) {
	state_estimate = new_state;
	const SquareMatrix<STATE_N> propagated_cov = propagate_covariance<STATE_N,STATE_N>(jac_F, state_covariance);
	state_covariance = mat_add<STATE_N,STATE_N>(propagated_cov, process_noise);
	estimate_timestamp = timestamp;
	last_predict_timestamp = timestamp;

	return EKF_SUCCESS;
}

// EK_filter::update in .tpp file

void EK_filter::set_state(const State_t state, const SquareMatrix<STATE_N>& state_cov) {
	state_estimate = state;
	state_covariance = state_cov;
}
void EK_filter::set_state(const Vector<STATE_N> state, const SquareMatrix<STATE_N>& state_cov) {
	state_estimate.position_x =  state(0,0);
	state_estimate.position_y =  state(1,0);
	state_estimate.velocity_u =  state(2,0);
	state_estimate.velocity_v =  state(3,0);
	state_estimate.angle =       state(4,0);
	state_estimate.angular_vel = state(5,0);
}

State_t EK_filter::get_state() const {
	return state_estimate;
}

float normalise_angle(float angle) {
	angle += PI;
	angle = fmodf(angle, PI*2.0f);
	angle -= PI;
	return angle;
}

Pose_t EK_filter::get_pose() const {
	Pose_t out;
	out.position_x = state_estimate.position_x;
	out.position_y = state_estimate.position_y;

	float angle_rad = normalise_angle(state_estimate.angle);
	float heading_rad = fmodf(2.5f*PI - angle_rad, PI*2.0f);
	out.heading = heading_rad * (180.0f / PI);

	return out;
}

Matrix<STATE_N,1> EK_filter::get_state_mat() const {
	Matrix<STATE_N,1> out;
	out(0,0) = state_estimate.position_x;
	out(1,0) = state_estimate.position_y;
	out(2,0) = state_estimate.velocity_u;
	out(3,0) = state_estimate.velocity_v;
	out(4,0) = state_estimate.angle;
	out(5,0) = state_estimate.angular_vel;
	return out;
}

Pose_t get_pose_from_state(State_t state) {
	Pose_t out = {
		.position_x = state.position_x,
		.position_y = state.position_y,
	};
	float angle_rad = normalise_angle(state.angle);
	float heading_rad = fmodf(2.5f*PI - angle_rad, PI*2.0f);
	out.heading = heading_rad * (180.0f / PI);
	return out;
}

uint32_t EK_filter::get_last_update_timestamp() const {
	return last_update_timestamp;
}

uint32_t EK_filter::get_last_predict_timestamp() const {
	return last_predict_timestamp;
}

uint32_t EK_filter::get_estimate_timestamp() const {
	return estimate_timestamp;
}
