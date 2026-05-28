/**
 * @file types.h
 * @brief Contains all type definitions for TTIME-EKF
 * @author Daniel Dew
 * @version 0.1
 * @date 25.05.26
 */
#ifndef EKF_TYPES_H
#define EKF_TYPES_H

namespace EKF {

constexpr float PI = 3.1415926535f;

/**
 * @brief number of recorded values in state
 */
constexpr int STATE_N = 6;
/**
 * @brief A type containing information about a robot's pose
 */
typedef struct {
	float position_x;     // X position in world frame (m)
	float position_y;      // Y position in world frame (m)
	float velocity_u;    // Longitudinal velocity in body frame (pointing out nose) (m/s)
	float velocity_v;    // Lateral velocity in body frame (pointing out right side) (m/s)
	float angle;         // The angle from the +ve x-axis (radians)
	float angular_vel; // Angular frequency/velocity (radians/s)
} State_t;
/**
 * @brief A type containing position and heading.
 *
 * More intuitive than State_t
 */
typedef struct {
	float position_x;
	float position_y;
	float heading;
} Pose_t;

Pose_t get_pose_from_state(State_t state);

}

#endif // EKF_TYPES_H
