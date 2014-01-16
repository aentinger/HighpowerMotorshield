/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/**
 * @author Alexander Entinger, MSc / LXRobotics GmbH
 * @brief this module enables a control of the LXRobotics Highpower Motorshield when flashed with the serial_motor_driver.ino sketch
 * @file lxr_hp_motor_control.cpp
 * @license MPL 2.0
 */

#include "lxr_hp_motor_control.h"
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>

/**
 * @brief Constructor
 * @param devNode string of the device node where the arduino is connected with the pc
 * @param id the id which is programmed in the connected arduino, value of the define SERIAL_MOTOR_DRIVER_ID in serial_motor_driver.ino
 */
lxr_hp_motor_control::lxr_hp_motor_control(std::string const &devNode, unsigned char const id) : m_serial(devNode, m_baudrate), m_id(id), m_speed(0), m_direction(E_FWD), m_error_flag(false), m_com_thread(boost::bind(&lxr_hp_motor_control::com_thread_func, this)), m_error_cb_func(0) {

}

/**
 * @brief Destructor
 */
lxr_hp_motor_control::~lxr_hp_motor_control() {

}

/**
 * @brief sets the speed of the motor
 */
void lxr_hp_motor_control::set_speed(unsigned char const speed) {
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_speed = speed;
}

/**
 * @brief sets the direction of the motor
 */
void lxr_hp_motor_control::set_direction(E_MOTOR_DIRECTION const dir) {
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_direction = dir;
}

/**
 * @brief register a function which is to be called in case of an error
 */
void lxr_hp_motor_control::register_error_callback(error_callback cb) {
	boost::lock_guard<boost::mutex> lock(m_mutex);
	if(cb != 0) m_error_cb_func = cb;
}

/**
 * @brief converts the error code into a string
 */
std::string lxr_hp_motor_control::convert_to_string(size_t const err_code) {
	std::stringstream ss;

	if(err_code & ID_WRONG) ss << "Wrong ID received" << std::endl;
	if(err_code & STATUS_WRONG) ss << "Error status received" << std::endl;
	if(err_code & CS_WRONG) ss << "Checksum error" << std::endl;

	return ss.str();
}

#define STATUS_ERROR                (0)
#define STATUS_OK                   (1)

/**
 * @brief this is the function executed by the communication thread
 */
void lxr_hp_motor_control::com_thread_func() {

	sleep(2); // delay two seconds to allow serial device to be fully initialized

	for(;;) {
		// build the message for sending down
		size_t const msg_size = 4;
		unsigned char msg_buf[4] = {0};

		enum {E_MSG_ID = 0, E_MSG_DIR = 1, E_MSG_SPEED = 2, E_MSG_CS = 3};

		msg_buf[E_MSG_ID] = m_id;
		{
			boost::lock_guard<boost::mutex> lock(m_mutex);
			msg_buf[E_MSG_DIR] = static_cast<unsigned char>(m_direction);
			msg_buf[E_MSG_SPEED] = m_speed;
		}
		msg_buf[E_MSG_CS] = msg_buf[E_MSG_ID] ^ msg_buf[E_MSG_DIR] ^ msg_buf[E_MSG_SPEED];

		//for(size_t i=0; i<msg_size; i++) std::cout << std::hex << "msg_buf[" << i << "] = 0x" << static_cast<size_t>(msg_buf[i]) << std::endl;

		// send the message
		m_serial.writeToSerial(msg_buf, msg_size);

		// receive the reply
		size_t const reply_size = 3;
		boost::shared_ptr<unsigned char> reply = m_serial.readFromSerial(reply_size);

		enum {E_REP_ID = 0, E_REP_STATUS = 1, E_REP_CS = 2};

		// evaluate the reply
		size_t err_code = NO_ERROR;
		if(reply.get()[E_REP_ID] != m_id) err_code |= ID_WRONG;
		if(reply.get()[E_REP_STATUS] != STATUS_OK) err_code |= STATUS_ERROR;
		if((reply.get()[E_REP_ID] ^ reply.get()[E_REP_STATUS]) != reply.get()[E_REP_CS]) err_code |= CS_WRONG;

		// in case of error call the registered error callback function
		if(err_code != NO_ERROR) {
			if(m_error_cb_func != 0) m_error_cb_func(err_code);
		}

		//for(size_t i=0; i<reply_size; i++) std::cout << std::hex << "reply[" << i << "] = 0x" << static_cast<size_t>(reply.get()[i]) << std::endl;

		boost::this_thread::sleep(boost::posix_time::milliseconds(m_com_thread_sleep_ms));
	}
}
