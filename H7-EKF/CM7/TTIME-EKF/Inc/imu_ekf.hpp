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

	typedef struct {
		SquareMatrix<3> accel_correction_matrix;
		SquareMatrix<3> gyro_correction_matrix;
		Vector<3> accel_biases;
		Vector<3> gyro_biases;
	} IMU_calibs_t;

	typedef struct {
		float pitch;
		float roll;
		float G_CONST;
	} Gravity_correction_estimate_t;

	int32_t update_g_correction(
			Gravity_correction_estimate_t& estimate,
			float ax, float ay, float az,
			float gx, float gy, float gz,
			float dt, float alpha=0.98f
	);

	void compensate_gravity(float& ax, float& ay, float& az, const Gravity_correction_estimate_t& correction);

	/**
	 * @brief Create a calibration params struct
	 * @param correction_mat3x3 3x3 correction accelerometer matrix (flattened array)
	 * @param accel_biases Biases for accelerometer (flattened array)
	 * @param gryo_biases Biases for gyroscope (flattened array)
	 * @return The calibration params struct
	 */
	IMU_calibs_t create_calib_params(const float* correction_mat3x3, const float* accel_biases, const float* gryo_biases);

	/**
	 * @brief Use calibrated parameters to correct accel data
	 * @param data Data to correct
	 * @param params Calibration params
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t correct_accel(float* data, const IMU_calibs_t* params);
	/**
	 * @brief Use calibrated parameters to correct gyro data
	 * @param data Data to correct
	 * @param params Calibration params
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t correct_gyro(float* data, const IMU_calibs_t* params);

	int32_t calibrate(float* (*getIMUdat)(), IMU_calibs_t* params, Gravity_correction_estimate_t& g_correction, uint32_t sample_time_ms=2000U);

	/**
	 * @brief Predict using acceleration and angular rate
	 * @param filter reference to the generic EK filter
	 * @param imu_dat a Vector of the longitudinal and lateral accelerations and the angular rate
	 * @param variances The variances of input data
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict(EK_filter* filter, const Vector<3>& imu_dat, const IMU_variances_t& variances, const Gravity_correction_estimate_t& g_correction, uint32_t timestamp);
	/**
	 * @brief Predict using acceleration and angular rate
	 * @param filter reference to the generic EK filter
	 * @param ax Longitudinal acceleration
	 * @param ay Lateral acceleration
	 * @param angular_rate
	 * @param timestamp The current tick
	 * @return EKF_SUCCESS if successful, EKF_ERROR if not
	 */
	int32_t predict(EK_filter* filter, float ax, float ay, float angular_rate, const IMU_variances_t& variances, const Gravity_correction_estimate_t& g_correction, uint32_t timestamp);

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
