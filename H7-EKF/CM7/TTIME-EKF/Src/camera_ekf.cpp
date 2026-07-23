/**
 * @file camera_ekf.cpp
 * @brief Contains all implementations for april tag based EKF updates
 * @author Daniel Dew
 */

#include "camera_ekf.hpp"
#include "ekf.hpp"
#include "matrix_maths.hpp"
#include "types.hpp"

#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace EKF;
using namespace EKF::Camera;


Tag_candidates_t EKF::Camera::lookup_tag(uint8_t id) {
    auto lo = std::lower_bound(std::begin(tag_table), std::end(tag_table), id,
        [](const Tag_entry_t& e, uint8_t v) { return e.id < v; });
    auto hi = std::upper_bound(std::begin(tag_table), std::end(tag_table), id,
        [](uint8_t v, const Tag_entry_t& e) { return v < e.id; });
    return { lo, hi };
}

int32_t update_with_tag(EK_filter* filter,
			const Tag_frame_t& frame,
			const Camera_orientation_t& orientation,
			uint32_t timestamp) {
	int32_t result = EKF_ERR;
	if (filter != NULL) {
		// (Crudely) integrate to current time
		filter->integrate_to_now(timestamp);
		// tag_frame_to_robot()
		Pose_t new_pose; Vector<3> innov; SquareMatrix<3> noise;
		int32_t infer_res = tag_frame_to_robot(new_pose, innov, noise, frame, orientation, filter);
		if (infer_res > 0) {
			// Calculate jacobian
			Matrix<3,6> jac_H = {
					.data {
						1, 0, 0, 0, 0, 0,
						0, 1, 0, 0, 0, 0,
						0, 0, 0, 0, 1, 0
					}
			};
			filter->update<3>(innov, jac_H, noise, timestamp);
		}
		result = EKF_SUCCESS;
	}
	return result;
}

int32_t tag_frame_to_robot(Pose_t& pose_out,
		Vector<3>& innov_out,
		SquareMatrix<3>& noise_out,
		const Tag_frame_t& frame,
		const Camera_orientation_t& cam_ori,
		const EK_filter* filter){
	Tag_candidates_t all_tags = lookup_tag((uint8_t)frame.ID);
	if (all_tags.count() < 1) return 0;
	float lowest_d2 = infinityf();
	Tag_entry_t most_likely_tag;
	for (size_t i=0; i<all_tags.count(); ++i) {
		Pose_t tmp_pose;
		robot_pose_from_tag(frame, cam_ori, all_tags[i].pose, tmp_pose);
		// For each, calculate poses and mahalanobis distances (using area)
		SquareMatrix<3> inferred_pose_cov = tag_area_to_cov(frame.area_px);
		Vector<3> tmp_innov; SquareMatrix<3> tmp_noise;
		float new_d2 = mahalanobis2_dist(filter,
				tmp_pose,
				inferred_pose_cov,
				tmp_innov,
				tmp_noise);
		if (new_d2 < lowest_d2) {
			lowest_d2 = new_d2;
			most_likely_tag = all_tags[i];
			innov_out = tmp_innov;
			noise_out = tmp_noise;
			pose_out = tmp_pose;
		}
	}
	// Select best and return
	return EKF_SUCCESS;
}

int32_t robot_pose_from_tag(const Tag_frame_t& obsvd_frame,
			const Camera_orientation_t& cam_ori,
			const Tag_pose_t& obsvd_tag_pose,
			Pose_t& pose_out) {
	/*
	 * -- Notation --
	 * rotation_object_frame (3x3 matrix)
	 * position_object_frame (1x3 vector)
	 * vector_from_to_frame (1x3 vector)
	 */
	// Get robot rotation matrix from relative rotations
	SquareMatrix<3> rotation_tag_world =
			euler_to_trans(obsvd_tag_pose.pitch,
					obsvd_tag_pose.yaw,
					obsvd_tag_pose.roll);
	SquareMatrix<3> rotation_tag_cam =
			euler_to_trans(obsvd_frame.pitch, obsvd_frame.yaw, obsvd_frame.roll);
	SquareMatrix<3> rotation_cam_robot =
			euler_to_trans(cam_ori.pitch, cam_ori.yaw, cam_ori.roll);

	SquareMatrix<3> rotation_tag_cam_T = mat_transpose(rotation_tag_cam);
	SquareMatrix<3> rotation_cam_robot_T = mat_transpose(rotation_cam_robot);

	SquareMatrix<3> rotation_robot_tag = mat_mult(rotation_tag_cam_T, rotation_cam_robot_T);
	SquareMatrix<3> rotation_robot_world = mat_mult(rotation_tag_world, rotation_robot_tag); // Desired

	// Get world-relative robot position
	Vector<3> vector_cam_tag_cam = {
			.data {
				obsvd_frame.vec_x,
				obsvd_frame.vec_y,
				obsvd_frame.vec_z
			}
	};
	Vector<3> vector_cam_tag_world =
			mat_mult(mat_mult(rotation_robot_world, rotation_cam_robot), vector_cam_tag_cam);
	Vector<3> vector_robot_cam_robot = {
			.data {
				cam_ori.position_x,
				cam_ori.position_y,
				cam_ori.position_z
			}
	};
	Vector<3> vector_robot_cam_world =
			mat_mult(rotation_robot_world, vector_robot_cam_robot);

	Vector<3> vector_robot_tag_world = mat_add(vector_robot_cam_world, vector_cam_tag_world);
	Vector<3> tag_position = {
			.data {
				obsvd_tag_pose.pos_x,
				obsvd_tag_pose.pos_y,
				obsvd_tag_pose.pos_z
			}
	};
	Vector<3> robot_position = mat_sub(tag_position, vector_robot_tag_world);

	pose_out.position_x = robot_position(0,0);
	pose_out.position_y = robot_position(1,0);
	pose_out.heading = atan2f(rotation_robot_world(1,0), rotation_robot_world(0,0));

	return EKF_SUCCESS;
}

float mahalanobis2_dist(const EK_filter* filter,
		const Pose_t& proposed_pose,
		const SquareMatrix<3>& pose_cov,
		Vector<3>& innov_out,
		SquareMatrix<3>& noise_out) {
	Pose_t pose_est = filter->get_pose();
	Vector<3> innov = {
			.data {
				proposed_pose.position_x - pose_est.position_x,
				proposed_pose.position_y - pose_est.position_y,
				proposed_pose.heading - pose_est.heading,
			}
	};

	while (innov(2,0) > PI) innov(2,0) -= 2.0f * PI;
	while (innov(2,0) < -PI) innov(2,0) += 2.0f*PI;

	innov_out = innov;

	SquareMatrix<6> state_cov = filter->get_state_cov();
	SquareMatrix<3> extracted_state_cov = {
			.data {
				state_cov(0,0), state_cov(0,1), state_cov(0,4),
				state_cov(1,0), state_cov(1,1), state_cov(1,4),
				state_cov(4,0), state_cov(4,1), state_cov(4,4)
			}
	};

	SquareMatrix<3> S = mat_add(extracted_state_cov, pose_cov);
	noise_out = S;
	SquareMatrix<3> S_inv = mat_inv_spd(S);

	Matrix<1,3> innov_T = mat_transpose(innov);
	Matrix<1,1> d2 = mat_mult(mat_mult(innov_T, S_inv), innov);

	return d2(0,0);
}

// This definitely needs tuning.
SquareMatrix<3> tag_area_to_cov(float area_px) {
	// Clamp area
	const float MIN_AREA = 25.0f; // 25 px
	if (area_px < MIN_AREA) area_px = MIN_AREA;

	// Assuming reliable tag detection (large px area)
	// Whats the min std. dev. of translation
	const float SIGMA_POS_FLOOR = 0.25f;  // inches
	const float SIGMA_YAW_FLOOR = 0.01f;  // radians
	// Scaling std dev with shrinking tag areas
	const float K_POS = 40.0f;
	const float K_YAW = 1.8f;

	float inv_sqrt_area = 1.0f / sqrtf(area_px);

	float sigma_pos = SIGMA_POS_FLOOR + K_POS * inv_sqrt_area;
	float sigma_yaw = SIGMA_YAW_FLOOR + K_YAW * inv_sqrt_area;

	SquareMatrix<3> R_pos = {
			.data {
				sigma_pos*sigma_pos, 0, 0,
				0, sigma_pos*sigma_pos, 0,
				0, 0, sigma_yaw*sigma_yaw
			}
	};

	return R_pos;
}



