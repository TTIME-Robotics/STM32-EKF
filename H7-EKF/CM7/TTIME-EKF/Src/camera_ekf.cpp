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

using namespace EKF;
using namespace EKF::Camera;


// Individual axis rotations, radians
Matrix<3,3> rot_x(float a) {
    Matrix<3,3> m = {};
    m(0,0) = 1.0f;
    m(1,1) =  cosf(a); m(1,2) = -sinf(a);
    m(2,1) =  sinf(a); m(2,2) =  cosf(a);
    return m;
}

Matrix<3,3> rot_y(float a) {
    Matrix<3,3> m = {};
    m(0,0) =  cosf(a); m(0,2) =  sinf(a);
    m(1,1) = 1.0f;
    m(2,0) = -sinf(a); m(2,2) =  cosf(a);
    return m;
}

Matrix<3,3> rot_z(float a) {
    Matrix<3,3> m = {};
    m(0,0) =  cosf(a); m(0,1) = -sinf(a);
    m(1,0) =  sinf(a); m(1,1) =  cosf(a);
    m(2,2) = 1.0f;
    return m;
}

Matrix<3,3> euler_to_rot(float pitch, float yaw, float roll) {
    // intrinsic Z-Y-X — confirm this matches your pose estimation convention
    return mat_mult<3,3,3>(mat_mult<3,3,3>(rot_z(yaw), rot_y(pitch)), rot_x(roll));
}
void rot_to_euler(const Matrix<3,3>& R, float& pitch, float& yaw, float& roll) {

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)  — matches euler_to_rot's convention
    //
    // R(2,0) = -sin(pitch)
    // R(1,0) =  cos(pitch)*sin(yaw)
    // R(0,0) =  cos(pitch)*cos(yaw)
    // R(2,1) =  cos(pitch)*sin(roll)
    // R(2,2) =  cos(pitch)*cos(roll)

    float sin_pitch = -R(2,0);

    // Clamp for float rounding before asinf — otherwise a value like
    // 1.0000001f from accumulated error returns NaN instead of ±90°
    if (sin_pitch > 1.0f)  sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;

    pitch = asinf(sin_pitch);

    const float GIMBAL_EPS = 1e-6f;
    float cos_pitch = cosf(pitch);

    if (cos_pitch > GIMBAL_EPS) {
        // Normal case
        yaw  = atan2f(R(1,0), R(0,0));
        roll = atan2f(R(2,1), R(2,2));
    } else {
        // Gimbal lock: pitch = ±90°, yaw and roll become coupled —
        // only their sum/difference is recoverable, so roll is
        // arbitrarily fixed to 0 and yaw absorbs the combined rotation.
        roll = 0.0f;
        if (sin_pitch > 0.0f) {
            // pitch = +90°
            yaw = atan2f(R(0,1), R(1,1));
        } else {
            // pitch = -90°
            yaw = -atan2f(R(0,1), R(1,1));
        }
    }
}



Tag_candidates_t lookup_tag(uint8_t id) {
    auto lo = std::lower_bound(std::begin(tag_table), std::end(tag_table), id,
        [](const Tag_entry_t& e, uint8_t v) { return e.id < v; });
    auto hi = std::upper_bound(std::begin(tag_table), std::end(tag_table), id,
        [](uint8_t v, const Tag_entry_t& e) { return v < e.id; });
    return { lo, hi };
}

Tag_pose_t frame_to_tag_pose(const Tag_frame_t* frame, const Camera_orientation_t* cam_ori) {
	Matrix<3,3> R_robot_cam = euler_to_rot(cam_ori->pitch, cam_ori->yaw, cam_ori->roll);
	Matrix<3,1> t_robot_cam = {};
	t_robot_cam(0,0) = cam_ori->position_x;
	t_robot_cam(1,0) = cam_ori->position_y;
	t_robot_cam(2,0) = cam_ori->position_z;

	Matrix<3,3> R_cam_tag = euler_to_rot(frame->pitch, frame->yaw, frame->roll);
	Matrix<3,1> t_cam_tag = {};
	t_cam_tag(0,0) = frame->vec_x;
	t_cam_tag(1,0) = frame->vec_y;
	t_cam_tag(2,0) = frame->vec_z;

	Matrix<3,3> R_robot_tag = mat_mult<3,3,3>(R_robot_cam, R_cam_tag);
	Matrix<3,1> t_robot_tag = mat_add<3,1>(t_robot_cam, mat_mult<3,3,1>(R_robot_cam, t_cam_tag));

	Tag_pose_t out;
	out.pos_x = t_robot_tag(0,0);
	out.pos_y = t_robot_tag(1,0);
	out.pos_z = t_robot_tag(2,0);
	rot_to_euler(R_robot_tag, out.pitch, out.yaw, out.roll);
	return out;
}

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


