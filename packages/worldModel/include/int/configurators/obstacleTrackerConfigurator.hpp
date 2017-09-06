 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleTrackerConfigurator.hpp
 *
 *  Created on: Nov 20, 2016
 *      Author: Jan Feitsma
 */

#ifndef OBSTACLETRACKERCONFIGURATOR_HPP_
#define OBSTACLETRACKERCONFIGURATOR_HPP_

#include <map>

#include "int/types/configurators/obstacleTrackerConfiguratorTypes.hpp"

// TODO factor out a base class, now this is duplicate w.r.t. ballTrackerConfigurator

class obstacleTrackerConfigurator
{
	public:
		/*!
		 * \brief Obtain an instance of this singleton class
		 */
		static obstacleTrackerConfigurator& getInstance()
		{
			static obstacleTrackerConfigurator instance;
			return instance;
		}

		/*!
		 * \brief Set a configuration parameter
		 */
		void set(const obstacleTrackerConfiguratorBool key, const bool value);
		void set(const obstacleTrackerConfiguratorIntegers key, const int value);
		void set(const obstacleTrackerConfiguratorFloats key, const float value);

		/*!
		 * \brief Set value based on string key, trying to deduce the type
		 */
		bool set(const std::string key, const std::string value);
		
		/*!
		 * \brief Get the value of a configuration parameter
		 * 
		 * Note: for template type deduction, the type must be explicitly provided 
		 */
		bool get(const obstacleTrackerConfiguratorBool key) const;
		int get(const obstacleTrackerConfiguratorIntegers key) const;
		float get(const obstacleTrackerConfiguratorFloats key) const;

		/*!
		 * \brief Get the string from enum value
		 */
		std::string enum2str(const obstacleTrackerConfiguratorBool key) const;
		std::string enum2str(const obstacleTrackerConfiguratorIntegers key) const;
		std::string enum2str(const obstacleTrackerConfiguratorFloats key) const;

		/*!
		 * \brief Trace all (key,value) pairs
		 */
		void traceAll();
        
		/*!
		 * \brief Reset all configuration parameters
		 */
		void reset();
        
	private:
	    std::map<obstacleTrackerConfiguratorBool, bool> _dataBool;
	    std::map<obstacleTrackerConfiguratorIntegers, int> _dataInteger;
	    std::map<obstacleTrackerConfiguratorFloats, float> _dataFloat;
	    std::map<obstacleTrackerConfiguratorBool, std::string> _enum2strBool;
	    std::map<obstacleTrackerConfiguratorIntegers, std::string> _enum2strInteger;
	    std::map<obstacleTrackerConfiguratorFloats, std::string> _enum2strFloat;

		obstacleTrackerConfigurator();
		~obstacleTrackerConfigurator();
};

#endif /* OBSTACLETRACKERCONFIGURATOR_HPP_ */
