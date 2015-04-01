#include "a2/Arm.hpp"
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"

#include <stdio.h>

class Handler
{
	public:
		~Handler() {}
		void handleMessage(const lcm::ReceiveBuffer* rbuf,
			const std::string& chan,
			const dynamixel_command_list_t* msg)
		{
			std::cout << "handling" << std::endl;
			Arm::instance()->moveArm(msg);
		}
};

int main() {
	lcm::LCM lcm;


	Handler handlerObject;
	lcm.subscribe("ARM", &Handler::handleMessage, &handlerObject);

	while(0 == lcm.handle());
	return 0;
	//Arm::instance()->moveArm(Pos);

}




