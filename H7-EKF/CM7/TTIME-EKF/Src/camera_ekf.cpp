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


Tag_candidates_t lookup_tag(uint8_t id) {
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
	int32_t result = EKF_SUCCESS
	if (filter == NULL) {
		result = EKF_ERR;
	}
	// tag_frame_to_robot()
	// calculate jacobian and noise
	// filter->update(innovation, jac_H, sensor_noise, timestamp)
	return result;
}

Pose_t tag_frame_to_robot(const Tag_frame_t& frame, const Camera_orientation_t* cam_ori){
	// Find candidates
	// For each, calculate poses and mahalanobis distances (using area)
	// Select best and return
}

float mahalanobis2_dist(const EK_filter* filter,
		const Pose_t& proposed_pose,
		const SquareMatrix<3>& pose_cov) {

}

SquareMatrix<3> tag_area_to_cov(float area_px) {
}



