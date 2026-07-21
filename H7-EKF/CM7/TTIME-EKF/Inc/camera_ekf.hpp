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
		float certainty;
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

	Pose_t tag_to_robot_frame(const Tag_frame_t* frame, const Camera_orientation_t* cam_ori);

	 // For one candidate goal, compute Mahalanobis distance given current filter state
	float mahalanobis_distance(const EK_filter* filter,
								 const Tag_pose_t& candidate,
								 const Pose_t& observed_robot_frame_pose,
								 Matrix& out_H,   // stash H, S, R etc if selected — avoids recompute
								 Matrix& out_S);

	// Returns index into candidates, or -1 if rejected (no confident association)
	int select_best_goal(const EK_filter* filter,
						  const Tag_candidates_t& candidates,
						  const Pose_t& observed_robot_frame_pose,
						  float chi2_gate,
						  float min_gap);

	int32_t update_tag(EK_filter* filter,
			const Tag_frame_t* frame,
			Camera_orientation_t* orientation,
			int32_t timestamp);

}

#endif // CAMERA_EKF_HPP
