/**
 * @file imu_ekf.hpp
 * @brief Contains implementations for update/predict steps using IMU sensor
 * @author Daniel Dew
 */

#include "imu_ekf.hpp"
#include "ekf.hpp"
#include "matrix_maths.hpp"
#include "types.hpp"

using namespace EKF;


int32_t IMU::predict(const EK_filter* filter, const Vector<3>& imu_dat, uint32_t timestamp) {
	return IMU::predict(filter, imu_dat(0), imu_dat(1), imu_dat(2), timestamp);
}

int32_t IMU::predict(const EK_filter* filter, float ax, float ay, float angular_rate, uint32_t timestamp) {

	State_t new_state;

	State_t current_state = filter->get_state();
	uint32_t tick_step = timestamp - filter->get_last_predict_timestamp();
	float dt = tick_step / 1000.0f;

	// Euler midpoint prediction
	float vlong_mid = current_state.velocity_u + 0.5 * ax * dt;
	float vlat_mid = current_state.velocity_v + 0.5 * ay * dt;
	float angle_mid = current_state.angle + 0.5* angular_rate * dt;

	float cam=cos(angle_mid);
	float sim=sin(angle_mid);

	new_state.position_x  = current_state.position_x + (vlong_mid*cam - vlat_mid*sim)*dt;
	new_state.position_y  = current_state.position_y + (vlong_mid*sim + vlat_mid*cam)*dt;
	new_state.velocity_u  = current_state.velocity_u + ax * dt;
	new_state.velocity_v  = current_state.velocity_v + ay * dt;
	new_state.angle       = current_state.angle + angular_rate*dt;
	new_state.angular_vel = angular_rate;

	// Compute jacobian

	// Compute process noise

	return filter->predict(new_state, jac_F, process_noise, timestamp)
}

int32_t predict_accel_only(const EK_filter& filter, const Vector<2>& accel_dat, uint32_t timestamp);

int32_t predict_accel_only(const EK_filter& filter, float ax, float ay, uint32_t timestamp);

int32_t predict_gyro_only(const EK_filter& filter, float angular_rate, uint32_t timestamp);

int32_t update_gyro(const EK_filter& filter, float angular_rate);

int32_t update_accel(const EK_filter& filter, const Vector<2>& accel_dat);

int32_t update_accel(const EK_filter& filter, float ax, float ay);
