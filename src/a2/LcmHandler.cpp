#include "LcmHandler.hpp"
#include "Arm.hpp"

LcmHandler* LcmHandler::_instance = new LcmHandler;

LcmHandler* LcmHandler::instance() {
	return _instance;
}

LcmHandler::LcmHandler() {
	_lcm.subscribe("ARM_STATUS", &LcmHandler::handleStatusMessage, this);

	if (pthread_mutex_init(&_statusMutex, NULL)) {
		printf("status Mutex not initialized\n");
		exit(1);
	}
}

void LcmHandler::handleStatusMessage(const lcm::ReceiveBuffer* rbuf,
	const std::string& chan, 
	const dynamixel_status_list_t* msg) {
	Arm::instance()->updateServoAngles(msg);
}

void LcmHandler::launchThreads() {
	pthread_create(&_lcmThreadPid, NULL, &LcmHandler::lcmHandle, this);
}

lcm::LCM* LcmHandler::getLcm() {
	return &_lcm;
}

void* LcmHandler::lcmHandle(void* args) {
	LcmHandler* state = (LcmHandler*) args;
	while (1) {
		state->_lcm.handle();
	}
	return NULL;
}

