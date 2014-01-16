/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/**
 * @author Alexander Entinger, MSc
 * @brief main file for highpower motor controller
 * @file main.cpp
 * @license MPL 2.0
 */

#include <iostream>
#include <cstdlib>

#include "lxr_hp_motor_control.h"

/**
 * @brief error handler function
 */
void error_handler(size_t const err_code) {
	std::cout << lxr_hp_motor_control::convert_to_string(err_code);
	exit(1);
}

int main(int argc, char **argv) {

	unsigned char const serial_motor_driver_id = 128;

	lxr_hp_motor_control mc("/dev/ttyACM0", serial_motor_driver_id);
	mc.register_error_callback(&error_handler);

	char cmd = 0;
	do {
		std::cout << "LXRobotics Highpower Motorshield Control Menu" << std::endl << std::endl;
		std::cout << "[0]\tset speed" << std::endl;
		std::cout << "[1]\tset direction" << std::endl;
		std::cout << "[q]\tquit" << std::endl;
		std::cout << ">> "; std::cin >> cmd;

		switch(cmd) {
		case '0': {
			size_t speed = 0;
			std::cout << "Enter speed" << std::endl << ">> ";
			std::cin >> speed;
			if(speed > 255) {
				std::cout << "Error, speed is in range of 0 to 255" << std::endl;
			} else {
				mc.set_speed(static_cast<unsigned char>(speed));
			}
		} break;
		case '1': {
			char dir = 0;
			std::cout << "Select direction" << std::endl;
			std::cout << "[0]\tFWD" << std::endl;
			std::cout << "[1]\tBWD" << std::endl << ">> " << std::endl;
			std::cin >> dir;
			switch(dir) {
			case '0': mc.set_direction(E_FWD); break;
			case '1': mc.set_direction(E_BWD); break;
			default: std::cout << "Error, only 0 and 1 are possible selections" << std::endl; break;
			}
		} break;
		case 'q': {
			std::cout << "Exiting now." << std::endl;
		} break;
		default: {
			std::cout << "Wrong command, try again" << std::endl;
		} break;
		}

	} while(cmd != 'q');

	return EXIT_SUCCESS;
}
