/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/**
 * @file serial.cpp
 * @author Alexander Entinger, BSc
 * @brief this file implements a class for serial communication
 * @license MPL2.0
 */

#include "serial.h"

/**
 * @brief Constructor
 */
serial::serial(std::string const &devNode, unsigned int const baudRate) :
                m_devNode(devNode), m_baudRate(baudRate), m_io_service(), m_serial_port(
                                m_io_service, m_devNode) {

        m_serial_port.set_option(
                        boost::asio::serial_port_base::baud_rate(m_baudRate));
        m_serial_port.set_option(boost::asio::serial_port_base::character_size(8));
        m_serial_port.set_option(
                        boost::asio::serial_port_base::flow_control(
                                        boost::asio::serial_port_base::flow_control::none));
        m_serial_port.set_option(
                        boost::asio::serial_port_base::parity(
                                        boost::asio::serial_port_base::parity::none));
        m_serial_port.set_option(
                        boost::asio::serial_port_base::stop_bits(
                                        boost::asio::serial_port_base::stop_bits::one));
}

/**
 * @brief Destructor
 */
serial::~serial() {

}

/**
 * @brief write data to the serial port
 */
void serial::writeToSerial(unsigned char const *buf, unsigned int const size) {
        boost::asio::write(m_serial_port, boost::asio::buffer(buf, size));
}

/**
 * @brief read data from the serial port
 */
boost::shared_ptr<unsigned char> serial::readFromSerial(unsigned int const size) {
        boost::shared_ptr<unsigned char> buf(new unsigned char[size]);

        boost::asio::read(m_serial_port, boost::asio::buffer(buf.get(), size));

        return buf;
}
