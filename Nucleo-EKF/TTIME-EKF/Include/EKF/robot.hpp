/**
 * @file robot_state.hpp
 * @brief Contains robot state class definitions
 */

#ifndef TTIME_EKF_ROBOT_HPP
#define TTIME_EKF_ROBOT_HPP

namespace EKF {
	typedef struct robot_state {
		float position_x;
		float position_y;
		float heading;
		float linear_vel;
		float angular_vel;
	} robot_state_t;

	class Robot {
		public:
			robot_state_t getState();

			int32_t predict_to_now(); // TODO: figure out stuff

		private:
			robot_state_t state_;

	};
} // namespace EKF

#endif // TTIME_EKF_ROBOT_STATE_HPP
