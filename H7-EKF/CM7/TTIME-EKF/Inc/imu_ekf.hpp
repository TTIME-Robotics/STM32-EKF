/**
 * @file imu_ekf.hpp
 * @brief Contains definitions for IMU-specific filtering
 * @author Daniel Dew
 */

#ifndef EKF_IMU_HPP
#define EKF_IMU_HPP

#include "ekf.hpp"
#include "matrix_maths.hpp"
#include <math.h>

namespace EKF::IMU {

	typedef struct {
		float variance_ax;
		float variance_ay;
		float variance_angular_rate;
	} IMU_variances_t;

	/**
	 * @brief Predict using acceleration and angular rate
	 * @param filter reference to the generic EK filter
	 * @param imu_dat a Vector of the longitudinal and lateral accelerations and the angular rate
	 * @param variances The variances of input data
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict(EK_filter* filter, const Vector<3>& imu_dat, const IMU_variances_t& variances, uint32_t timestamp);
	/**
	 * @brief Predict using acceleration and angular rate
	 * @param filter reference to the generic EK filter
	 * @param ax Longitudinal acceleration
	 * @param ay Lateral acceleration
	 * @param angular_rate
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict(EK_filter* filter, float ax, float ay, float angular_rate, const IMU_variances_t& variances, uint32_t timestamp);

	/**
	 * @brief Predict using acceleration only
	 * @param filter reference to the generic EK filter
	 * @param accel_dat Vector of longitudinal and lateral acceleration
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict_accel_only(EK_filter* filter, const Vector<2>& accel_dat, uint32_t timestamp);
	/**
	 * @brief Predict using acceleration only
	 * Assumes constant angular rate
	 * @param filter reference to the generic EK filter
	 * @param ax Longitudinal acceleration
	 * @param ay Longitudinal acceleration
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict_accel_only(EK_filter* filter, float ax, float ay, uint32_t timestamp);

	/**
	 * @brief Predict using the angular rate only
	 * Assumes constant acceleration
	 * @param filter reference to the generic EK filter
	 * @param angular_rate
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict_gyro_only(EK_filter* filter, float angular_rate, uint32_t timestamp);

	/**
	 * @brief Update using the gyro
	 * @param filter reference to the generic EK filter
	 * @param angular_rate
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t update_gyro(EK_filter* filter, float angular_rate);

	/**
	 * @brief update using acceleration from a centripetal model
	 * @param filter reference to the generic EK filter
	 * @param accel_dat A vector of the longitudinal/lateral acceleration data
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t update_accel(EK_filter* filter, const Vector<2>& accel_dat);

	/**
	 * @brief update using acceleration from a centripetal model
	 * @param filter reference to the generic EK filter
	 * @param ax Longitudinal acceleration
	 * @param ay Lateral acceleration
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t update_accel(EK_filter* filter, float ax, float ay);
}

#endif //EKF_IMU_HPP
