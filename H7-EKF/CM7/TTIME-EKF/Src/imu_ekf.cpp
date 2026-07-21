/**
 * @file imu_ekf.cpp
 * @brief Contains implementations for update/predict steps using IMU sensor
 * @author Daniel Dew
 */

#include "imu_ekf.hpp"
#include "ekf.hpp"
#include "matrix_maths.hpp"
#include "types.hpp"
#include "main.h"
#include <cstring>
#include <cmath>

using namespace EKF;

IMU::IMU_calibs_t IMU::create_calib_params(const float* correction_mat3x3, const float* accel_biases, const float* gyro_biases) {
    IMU_calibs_t params;
    std::memcpy(&params.accel_correction_matrix.data, correction_mat3x3, sizeof(float) * 9U);
    std::memcpy(&params.accel_biases.data, accel_biases, sizeof(float) * 3U);
    std::memcpy(&params.gyro_biases.data, gyro_biases, sizeof(float) * 3U);
    params.gyro_correction_matrix = Identity<3>();
    return params;
}

int32_t IMU::correct_accel(float* data, const IMU_calibs_t* params) {
    if (!data || !params) return EKF_ERR;
    Vector<3> data_vec;
    std::memcpy(&data_vec.data, data, sizeof(float) * 3U);
    data_vec = mat_mult(params->accel_correction_matrix, mat_sub(data_vec, params->accel_biases));
    std::memcpy(data, &data_vec.data, sizeof(float) * 3U);
    return EKF_SUCCESS;
}

int32_t IMU::correct_gyro(float* data, const IMU_calibs_t* params) {
    if (!data || !params) return EKF_ERR;
    Vector<3> data_vec;
    std::memcpy(&data_vec.data, data, sizeof(float) * 3U);
    data_vec = mat_sub(data_vec, params->gyro_biases);
    std::memcpy(data, &data_vec.data, sizeof(float) * 3U);
    return EKF_SUCCESS;
}

int32_t IMU::calibrate(float* (*getIMUdat)(), IMU_calibs_t* params, uint32_t sample_time_ms) {
    if (getIMUdat == nullptr || params == nullptr || sample_time_ms == 0U) {
        return EKF_ERR;
    }

    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    uint32_t samples = 0;
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < sample_time_ms) {
        float* raw_data = getIMUdat();
        if (raw_data != nullptr) {
            ax_sum += raw_data[0];
            ay_sum += raw_data[1];
            az_sum += raw_data[2];
            gx_sum += raw_data[3];
            gy_sum += raw_data[4];
            gz_sum += raw_data[5];
            samples++;
        }
        HAL_Delay(10U);
    }

    if (samples == 0U) return EKF_ERR;

    // Calculate sample averages
    const float inv_samples = 1.0f / static_cast<float>(samples);
    float ax_avg = ax_sum * inv_samples;
    float ay_avg = ay_sum * inv_samples;
    float az_avg = az_sum * inv_samples;
    float gx_avg = gx_sum * inv_samples;
    float gy_avg = gy_sum * inv_samples;
    float gz_avg = gz_sum * inv_samples;

    // 1. Accumulate Gyroscope Biases
    params->gyro_biases(0, 0) += gx_avg;
    params->gyro_biases(1, 0) += gy_avg;
    params->gyro_biases(2, 0) += gz_avg;

    // 2. Compute Pitch and Roll Angles from Static Gravity Vector
    float roll  = std::atan2(ay_avg, az_avg);
    float pitch = std::atan2(-ax_avg, std::sqrt(ay_avg * ay_avg + az_avg * az_avg));

    float cr = std::cos(roll);
    float sr = std::sin(roll);
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);

    // 3. Construct Leveling Matrix
    float level_mat[9U] = {
        cp,   sp * sr,  sp * cr,
        0.0f, cr,      -sr,
        -sp,  cp * sr,  cp * cr
    };

    SquareMatrix<3> levelling;
    std::memcpy(levelling.data, level_mat, sizeof(float) * 9U);

    // 4. Apply Leveling Matrix Alignment
    params->accel_correction_matrix = mat_mult(levelling, params->accel_correction_matrix);
    std::memcpy(params->gyro_correction_matrix.data, level_mat, sizeof(float) * 9U);

    return EKF_SUCCESS;
}

int32_t IMU::predict(EK_filter* filter, const Vector<3>& imu_dat, const IMU_variances_t& variances, uint32_t timestamp) {
    return IMU::predict(filter, imu_dat(0, 0), imu_dat(1, 0), imu_dat(2, 0), variances, timestamp);
}

int32_t IMU::predict(EK_filter* filter, float ax, float ay, float angular_rate, const IMU_variances_t& variances, uint32_t timestamp) {
    if (!filter) return EKF_ERR;

    State_t new_state;
    State_t current_state = filter->get_state();
    uint32_t tick_step = timestamp - filter->get_last_predict_timestamp();
    float dt = static_cast<float>(tick_step) / 1000.0f;

    // 1. Euler midpoint kinematic prediction
    float vlong_mid = current_state.velocity_u + 0.5f * ax * dt;
    float vlat_mid  = current_state.velocity_v + 0.5f * ay * dt;
    float angle_mid = current_state.angle + 0.5f * angular_rate * dt;

    float cam = std::cos(angle_mid);
    float sim = std::sin(angle_mid);

    new_state.position_x  = current_state.position_x + (vlong_mid * cam - vlat_mid * sim) * dt;
    new_state.position_y  = current_state.position_y + (vlong_mid * sim + vlat_mid * cam) * dt;
    new_state.velocity_u  = current_state.velocity_u + ax * dt;
    new_state.velocity_v  = current_state.velocity_v + ay * dt;
    new_state.angle       = current_state.angle + angular_rate * dt;
    new_state.angular_vel = angular_rate;

    // 2. State Transition Jacobian (jac_F)
    SquareMatrix<STATE_N> jac_F = Identity<STATE_N>();

    float d_posx_d_angle = (-vlong_mid * sim - vlat_mid * cam) * dt;
    float d_posy_d_angle = ( vlong_mid * cam - vlat_mid * sim) * dt;

    jac_F(0, 2) = cam * dt;
    jac_F(0, 3) = -sim * dt;
    jac_F(0, 4) = d_posx_d_angle;

    jac_F(1, 2) = sim * dt;
    jac_F(1, 3) = cam * dt;
    jac_F(1, 4) = d_posy_d_angle;

    jac_F(5, 5) = 0.0f; // Reset self-dependence since angular_rate overwrites omega

    // 3. Input Noise Transformation Jacobian (jac_G)
    Matrix<STATE_N, 3> jac_G = Zero<STATE_N, 3>;

    jac_G(0, 0) = 0.5f * dt * dt * cam;
    jac_G(0, 1) = -0.5f * dt * dt * sim;
    jac_G(0, 2) = 0.5f * dt * d_posx_d_angle;

    jac_G(1, 0) = 0.5f * dt * dt * sim;
    jac_G(1, 1) = 0.5f * dt * dt * cam;
    jac_G(1, 2) = 0.5f * dt * d_posy_d_angle;

    jac_G(2, 0) = dt;
    jac_G(3, 1) = dt;

    jac_G(4, 2) = dt;
    jac_G(5, 2) = 1.0f;

    // 4. Construct Noise Covariance Matrix (W)
    SquareMatrix<3> variances_mat = Zero<3, 3>;
    variances_mat(0, 0) = variances.variance_ax;
    variances_mat(1, 1) = variances.variance_ay;
    variances_mat(2, 2) = variances.variance_angular_rate;

    // 5. Compute Process Noise Matrix: Q = G * W * G^T
    SquareMatrix<STATE_N> process_noise = mat_mult<STATE_N, 3, STATE_N>(
        mat_mult(jac_G, variances_mat),
        mat_transpose(jac_G)
    );

    return filter->predict(new_state, jac_F, process_noise, timestamp);
}
