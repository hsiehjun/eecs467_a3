#include "Arm.hpp"
#include "Constants.hpp"
#include "LcmHandler.hpp"
#include "math/angle_functions.hpp"
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include <cmath>
#include <iostream>

using namespace eecs467;

Arm* Arm::_instance = new Arm;

Arm::Arm() : _isMoving(false) {
	if (pthread_mutex_init(&_armMutex, NULL)) {
		printf("armMutex not initialized\n");
		exit(1);
	}
}

Arm* Arm::instance() {
	return _instance;
}

void Arm::updateServoAngles(const dynamixel_status_list_t* list) {
	pthread_mutex_lock(&_armMutex);
	_list = *list;
	pthread_mutex_unlock(&_armMutex);
	float error = 0;
	for (int i= 0; i < 6; ++i) {
		error += abs(_list.statuses[i].position_radians -
			_targetPos[i]);
	}

	if (error < ARM_ERROR_THRESH) {
		pthread_mutex_lock(&_armMutex);
		_isMoving = true;
		pthread_mutex_unlock(&_armMutex);
	}
}

bool Arm::forwardKinematics(std::array<float, 2>& arr) {
	pthread_mutex_lock(&_armMutex);
	if (_list.statuses.size() == 0) {
		pthread_mutex_unlock(&_armMutex);
		return false;
	}

	float length = ARM_LENGTH_1 * sin(_list.statuses[1].position_radians) +
		ARM_LENGTH_2 * sin(_list.statuses[1].position_radians + 
			_list.statuses[2].position_radians) +
		(ARM_LENGTH_3 + CLAW_LENGTH) * sin(_list.statuses[1].position_radians + 
			_list.statuses[2].position_radians +
			_list.statuses[3].position_radians);
	float x = -length * cos(_list.statuses[0].position_radians);
	float y = -length * sin(_list.statuses[0].position_radians);
	pthread_mutex_unlock(&_armMutex);

	arr = std::array<float, 2>{{x, y}};
	return true;
}

/*
  Basic distance functions.
*/
double Arm::distance(double x, double y)
{
	return sqrt(x*x + y*y);
}

/*
  The x/y plane is the plane of the surface of the board.
  The z coordinate is the height off of the board.
  Returns a list of the servo angles to set the arm to for it to grab a position.
	 If the function fails, the array will be empty.
  Position (to_x, to_y, to_z) is in the global coordinate frame (meters).
	 It refers to the center of the "palm" surface of the robot.
  Position (base_x, base_y) is in the global coordinate frame (meters).
*/
  
bool Arm::inverseKinematics(double to_x, double to_y, double to_z, std::array<float, 6>& arr)
{
	float R = distance(to_x, to_y);
	if (R == 0) {
		return false;
	}

	float MSQR = R * R + (ARM_LENGTH_3 + to_z - ARM_LENGTH_0) * 
		(ARM_LENGTH_3 - ARM_LENGTH_0);
	float alpha = atan2(ARM_LENGTH_3 + to_z - ARM_LENGTH_0, R);
	float beta = acos((-ARM_LENGTH_2 * ARM_LENGTH_2 + 
		ARM_LENGTH_1 * ARM_LENGTH_1 + MSQR) / 
		(2 * ARM_LENGTH_1 * sqrt(MSQR)));
	float gamma = acos((-MSQR + ARM_LENGTH_1 * ARM_LENGTH_1 + 
		ARM_LENGTH_2 * ARM_LENGTH_2) / 
		(2 * ARM_LENGTH_1 * ARM_LENGTH_2));
	if (isnan(beta) || isnan(gamma)) {
		return false;
	}

	arr[0] = wrap_to_pi(M_PI + atan2(to_y, to_x));
	arr[1] = angle_diff(angle_diff(M_PI / 2, alpha), beta);
	arr[2] = angle_diff(M_PI, gamma);
	arr[3] = angle_diff(angle_diff(M_PI, arr[2]), arr[1]);
	arr[4] = M_PI/2.;
	arr[5] = M_PI/2.;

	return true;
}

bool Arm::moveArm(const dynamixel_command_list_t* list) {
	pthread_mutex_lock(&_armMutex);
	/*if (!inverseKinematics(x, y, 0.05, _targetPos)) {
		pthread_mutex_unlock(&_armMutex);
		return false;
	}*/
	_isMoving = true;

	dynamixel_command_list_t cmdList;
	for (int i = 0; i < 6; ++i) {
		dynamixel_command_t cmd;
		cmd.position_radians = list->commands[i].position_radians;
		cmd.speed = ARM_SPEED;
		cmd.max_torque = ARM_MAX_TORQUE;
		cmdList.commands.push_back(cmd);
	}
	cmdList.len = cmdList.commands.size();

	LcmHandler::instance()->getLcm()->publish(ARM_CHANNEL_NAME, &cmdList);

	pthread_mutex_unlock(&_armMutex);
	return true;
}

bool Arm::inMotion() {
	pthread_mutex_lock(&_armMutex);
	bool ret = _isMoving;
	pthread_mutex_unlock(&_armMutex);
	return ret;
}