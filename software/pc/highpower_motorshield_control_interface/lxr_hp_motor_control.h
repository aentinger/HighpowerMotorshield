/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/**
 * @author Alexander Entinger, MSc / LXRobotics GmbH
 * @brief this module enables a control of the LXRobotics Highpower Motorshield when flashed with the serial_motor_driver.ino sketch
 * @file lxr_hp_motor_control.h
 * @license MPL 2.0
 */

#ifndef LXR_HP_MOTOR_CONTROL_H_
#define LXR_HP_MOTOR_CONTROL_H_

#define DEBUG_OUTPUT_ENABLED		(true)

#include <string>
#include <boost/thread.hpp>

#include "serial.h"

// function pointer for registering an error callback function
static size_t const NO_ERROR = 0;
static size_t const ID_WRONG = 1;
static size_t const STATUS_WRONG = 2;
static size_t const CS_WRONG = 4;
typedef void(*error_callback)(size_t const err_code);

// typedef for motor direction
typedef enum {E_BWD = 0, E_FWD = 1} E_MOTOR_DIRECTION;

class lxr_hp_motor_control {
public:
	/**
	 * @brief Constructor
	 * @param devNode string of the device node where the arduino is connected with the pc
	 * @param id the id which is programmed in the connected arduino, value of the define SERIAL_MOTOR_DRIVER_ID in serial_motor_driver.ino
	 */
	lxr_hp_motor_control(std::string const &devNode, unsigned char const id);

	/**
	 * @brief Destructor
	 */
	~lxr_hp_motor_control();

	/**
	 * @brief sets the speed of the motor
	 */
	void set_speed(unsigned char const speed);

	/**
	 * @brief sets the direction of the motor
	 */
	void set_direction(E_MOTOR_DIRECTION const dir);

	/**
	 * @brief register a function which is to be called in case of an error
	 */
	void register_error_callback(error_callback cb);

	/**
	 * @brief converts the error code into a string
	 */
	static std::string convert_to_string(size_t const err_code);

protected:
	static size_t const m_baudrate = 115200;
	static size_t const m_com_thread_sleep_ms = 100;

private:
	serial m_serial;

	unsigned char m_id;
	unsigned char m_speed;
	E_MOTOR_DIRECTION m_direction;

	bool m_error_flag;

	boost::mutex m_mutex;

	boost::thread m_com_thread;

	error_callback m_error_cb_func;

	/**
	 * @brief this is the function executed by the communication thread
	 */
	void com_thread_func();
};

#endif /* LXR_HP_MOTOR_CONTROL_H_ */
