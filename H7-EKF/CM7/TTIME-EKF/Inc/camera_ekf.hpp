/**
 * @file camera_ekf.hpp
 * @brief Contains definitions for camera/apriltag EKF functionality
 * @author Daniel Dew
 */

#ifndef CAMERA_EKF_HPP
#define CAMERA_EKF_HPP

#include "error.h"
#include "ekf.hpp"
#include "types.hpp"
#include "matrix_maths.hpp"

#include <iostream>
#include <unordered_map>

namespace EKF::Camera {

	// Tag registry and types

	typedef struct {
		float position_x;
		float position_y;
		float position_z;
		float pitch;
		float yaw;
		float roll;
	} Camera_orientation_t;

	typedef struct {
		float vec_x;
		float vec_y;
		float vec_z;
		float pitch;
		float yaw;
		float roll;
		int ID;
		float area_px;
	} Tag_frame_t;

	typedef struct {
	    float pos_x;
	    float pos_y;
	    float pos_z;
	    float pitch;
	    float yaw;
	    float roll;
	} Tag_pose_t;

	typedef struct {
	    uint8_t id;
	    Tag_pose_t pose;
	} Tag_entry_t;

	// Sorted by id — required for the binary-search lookup below.
	// Multiple entries with the same id are fine (mirrored/duplicate tags).
	static constexpr Tag_entry_t tag_table[] = {
	    {  1, {  0.90f,  1.83f, 0.30f,  0.0f,   0.0f,   0.0f } },
	    {  1, { -0.90f, -1.83f, 0.30f,  0.0f, 180.0f,   0.0f } },  // mirrored side
	    {  2, {  1.20f,  1.83f, 0.30f,  0.0f,   0.0f,   0.0f } },
	    // ...
	};

	struct Tag_candidates_t {
	    const Tag_entry_t* begin;
	    const Tag_entry_t* end;
	    size_t count() const { return end - begin; }
	};

	Tag_candidates_t lookup_tag(uint8_t id);

	// --

	/**
	 * @brief Use a tag frame to update the EKF
	 * @param filter
	 * @param frame
	 * @param orientation
	 * @param timestamp
	 * @return
	 */
	int32_t update_with_tag(EK_filter* filter,
			const Tag_frame_t& frame,
			const Camera_orientation_t& orientation,
			uint32_t timestamp);

	/**
	 * @brief Get robot pose from a specific tag
	 * @param obsvd_frame
	 * @param cam_ori
	 * @param obsvd_tag_pose
	 * @param pose_out
	 * @return
	 */
	int32_t robot_pose_from_tag(const Tag_frame_t& obsvd_frame,
			const Camera_orientation_t& cam_ori,
			const Tag_pose_t& obsvd_tag_pose,
			Pose_t& pose_out);

	/**
	 * @brief Calculated the square mahalanobis distance
	 * How many std deviations from the expected pose and distrubution
	 * @param filter
	 * @param proposed_pose
	 * @param pose_cov
	 * @return
	 */
	float mahalanobis2_dist(const EK_filter* filter,
			const Pose_t& proposed_pose,
			const SquareMatrix<3>& pose_cov);

	// -- Helper functions

	/**
	 * @brief Convert area of tag to pose covariance
	 * @param area_px
	 * @return
	 */
	SquareMatrix<3> tag_area_to_cov(float area_px);

}

#endif // CAMERA_EKF_HPP
