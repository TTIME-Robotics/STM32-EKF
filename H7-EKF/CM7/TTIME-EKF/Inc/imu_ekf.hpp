/**
 * @file imu_ekf.hpp
 * @brief Contains definitions for IMU-specific filtering
 * @author Daniel Dew
 */

#ifndef EKF_IMU_HPP
#define EKF_IMU_HPP

#include "ekf.hpp"
#include "matrix_maths.hpp"
#include <cmath>

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

    /**
     * @brief Create a calibration params struct
     * @param correction_mat3x3 3x3 correction accelerometer matrix (flattened array)
     * @param accel_biases Biases for accelerometer (flattened array)
     * @param gyro_biases Biases for gyroscope (flattened array)
     * @return The calibration params struct
     */
    IMU_calibs_t create_calib_params(const float* correction_mat3x3, const float* accel_biases, const float* gyro_biases);

    /**
     * @brief Use calibrated parameters to correct accel data
     */
    int32_t correct_accel(float* data, const IMU_calibs_t* params);

    /**
     * @brief Use calibrated parameters to correct gyro data
     */
    int32_t correct_gyro(float* data, const IMU_calibs_t* params);

    int32_t calibrate(float* (*getIMUdat)(), IMU_calibs_t* params, uint32_t sample_time_ms = 2000U);

    /**
     * @brief Predict using acceleration and angular rate
     */
    int32_t predict(EK_filter* filter, const Vector<3>& imu_dat, const IMU_variances_t& variances, uint32_t timestamp);

    int32_t predict(EK_filter* filter, float ax, float ay, float angular_rate, const IMU_variances_t& variances, uint32_t timestamp);

    int32_t predict_accel_only(EK_filter* filter, const Vector<2>& accel_dat, uint32_t timestamp);
    int32_t predict_accel_only(EK_filter* filter, float ax, float ay, uint32_t timestamp);

    int32_t predict_gyro_only(EK_filter* filter, float angular_rate, uint32_t timestamp);

    int32_t update_gyro(EK_filter* filter, float angular_rate);
    int32_t update_accel(EK_filter* filter, const Vector<2>& accel_dat);
    int32_t update_accel(EK_filter* filter, float ax, float ay);
}

#endif // EKF_IMU_HPP
