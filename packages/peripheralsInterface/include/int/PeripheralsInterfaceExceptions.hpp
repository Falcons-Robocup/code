 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
