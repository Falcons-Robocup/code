 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTrackerConfigurator.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLTRACKERCONFIGURATOR_HPP_
#define BALLTRACKERCONFIGURATOR_HPP_

#include <map>

#include "int/types/configurators/ballTrackerConfiguratorTypes.hpp"

class ballTrackerConfigurator
{
	public:
		/*!
		 * \brief Obtain an instance of this singleton class
		 */
		static ballTrackerConfigurator& getInstance()
		{
			static ballTrackerConfigurator instance;
			return instance;
		}

		/*!
		 * \brief Set a configuration parameter
		 */
		void set(const ballTrackerConfiguratorBool key, const bool value);
		void set(const ballTrackerConfiguratorIntegers key, const int value);
		void set(const ballTrackerConfiguratorFloats key, const float value);

		/*!
		 * \brief Set value based on string key, trying to deduce the type
		 */
		bool set(const std::string key, const std::string value);
		
		/*!
		 * \brief Get the value of a configuration parameter
		 * 
		 * Note: for template type deduction, the type must be explicitly provided 
		 */
		bool get(const ballTrackerConfiguratorBool key) const;
		int get(const ballTrackerConfiguratorIntegers key) const;
		float get(const ballTrackerConfiguratorFloats key) const;

		/*!
		 * \brief Get the string from enum value
		 */
		std::string enum2str(const ballTrackerConfiguratorBool key) const;
		std::string enum2str(const ballTrackerConfiguratorIntegers key) const;
		std::string enum2str(const ballTrackerConfiguratorFloats key) const;

		/*!
		 * \brief Trace all (key,value) pairs
		 */
		void traceAll();
        
		/*!
		 * \brief Reset all configuration parameters
		 */
		void reset();
        
	private:
	    std::map<ballTrackerConfiguratorBool, bool> _dataBool;
	    std::map<ballTrackerConfiguratorIntegers, int> _dataInteger;
	    std::map<ballTrackerConfiguratorFloats, float> _dataFloat;
	    std::map<ballTrackerConfiguratorBool, std::string> _enum2strBool;
	    std::map<ballTrackerConfiguratorIntegers, std::string> _enum2strInteger;
	    std::map<ballTrackerConfiguratorFloats, std::string> _enum2strFloat;

		ballTrackerConfigurator();
		~ballTrackerConfigurator();
};

#endif /* BALLTRACKERCONFIGURATOR_HPP_ */
