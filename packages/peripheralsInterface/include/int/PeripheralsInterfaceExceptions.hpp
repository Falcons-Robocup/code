// Copyright 2016 Prabhu Mani (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PeripheralsInterfaceExceptions.hpp
 *
 *  Created on: Feb 24, 2016
 *      Author: Prabhu Mani
 */

#ifndef PERIPHERALSINTERFACEEXCEPTIONS_HPP_
#define PERIPHERALSINTERFACEEXCEPTIONS_HPP_

#include <exception>
#include <string>
#include <boost/format.hpp>


namespace deviceManagerException{
	/**
	 * Exception for the case firmware version is not the latest
	 */
	class InvalidFirmwareVersionException : public std::runtime_error {
	 public:
		InvalidFirmwareVersionException(const std::string& error_text) :
		std::runtime_error(error_text) {}

	};

	/**
	 * Exception for the case two devices with equal deviceId are connected.
	 */
	class InvalidDeviceIdException : public std::runtime_error {
	 public:
		InvalidDeviceIdException(const std::string& error_text) :
		std::runtime_error(error_text) {}

	};
}

namespace peripheralsInterfaceDataException{
//TODO:
}
namespace peripheralsInterfaceMotionException{
//TODO:
}

namespace motorControllerException{

	/**
	 * Exception for the case where no application id is obtained from the board
	 */
	class NoApplicationId : public std::runtime_error {
	 public:
		NoApplicationId(const std::string& portname) :
		std::runtime_error{std::string{"No application id received from "} + portname} {}
	};

} /* end of namespace motorController */

namespace communicationException{
	/**
	 * Exception for the case where far end receive buffer size is not > 255
	 */
	class FarEndBufferFull : public std::runtime_error {
	 public:
		FarEndBufferFull(const size_t& farEndBufferSize) :
		std::runtime_error{(boost::format("Error: no space in far end receive buffer. buffer has size %d") % farEndBufferSize).str()} {}

	};
	class zeroBytesFromFarEnd : public std::runtime_error {
	 public:
		zeroBytesFromFarEnd(const size_t& numBytes) :
		std::runtime_error{(boost::format("Error: no data read from far end. bytes read:%d") % numBytes).str()} {}

	};
}

namespace serialException{
	/**
	 * Exception for the case where reading a byte returns error
	 */
	class ReadBytesError : public std::runtime_error {
	 public:
		ReadBytesError(const std::string& errorNum, const std::string& portName) :
		std::runtime_error{(boost::format("Serial error: reading %s in port %s failed.") % errorNum % portName).str()} {}
	};

	/**
	 * Exception for the case where writing does not complete
	 */
	class WriteBytesError : public std::runtime_error {
	 public:
		WriteBytesError(const std::string& errorNum, const std::string& portName) :
		std::runtime_error{(boost::format("Serial error: writing %s in port %s failed.") % errorNum % portName).str()} {}
	};

	/**
	 * Exception for the case where serial port opening fails
	 */
	class PortOpenFailed : public std::runtime_error {
	 public:
		PortOpenFailed(const std::string& portName) :
		std::runtime_error{(boost::format("Serial error: opening port (%s) failed.")% portName).str()} {}
	};
}

#endif /* PERIPHERALSINTERFACEEXCEPTIONS_HPP_ */
