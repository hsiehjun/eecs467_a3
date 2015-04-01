#ifndef ARM_HPP
#define ARM_HPP

#include <stdint.h>
#include <pthread.h>
#include <array>
#include <vector>
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_command_list_t.hpp"


class Arm {
private:
	dynamixel_status_list_t _list;
	pthread_mutex_t _armMutex;
	Arm();
	static Arm* _instance;
	bool _isMoving;
	std::array<float, 6> _targetPos;
	
	static double distance(double x, double y);
public:
	static Arm* instance();

	void updateServoAngles(const dynamixel_status_list_t* list);

	bool forwardKinematics(std::array<float, 2>& arr);

	static bool inverseKinematics(double to_x, double to_y, double to_z, std::array<float, 6>& arr);

	bool moveArm(const dynamixel_command_list_t* list);

	bool inMotion();
};

#endif /* ARM_HPP */
