#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"
#include <vector>
#include <iostream>

int main(int argc, char ** argv)
{
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	dynamixel_command_list_t my_data;
	my_data.len = 6;
	my_data.commands.reserve(6);
	double x;
	while(1)
	{
		for(int i = 0; i < 6; i++)
		{
			std::cout << "Pos[" << i << "]: ";
			std::cin >> x;
			my_data.commands[i].position_radians = x;		
		}
		lcm.publish("ARM", &my_data);
	}
	return 0;
}
