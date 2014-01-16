/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/**
 * @file serial.h
 * @author Alexander Entinger, BSc
 * @brief this file implements a class for serial communication
 * @license MPL2.0
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <string>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

class serial {
public:
        /**
         * @brief Constructor
         */
        serial(std::string const &devNode, unsigned int const baudRate);

        /**
         * @brief Destructor
         */
        ~serial();

        /**
         * @brief write data to the serial port
         */
        void writeToSerial(unsigned char const *buf, unsigned int const size);

        /**
         * @brief read data from the serial port
         */
        boost::shared_ptr<unsigned char> readFromSerial(unsigned int const size);

private:
        std::string m_devNode;
        unsigned int m_baudRate;
        boost::asio::io_service m_io_service;
        boost::asio::serial_port m_serial_port;
};

#endif /* SERIAL_H_ */
