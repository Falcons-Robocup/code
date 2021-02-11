// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cEnvironmentRobot.hpp
 *
 *  Created on: Jan 3, 2016
 *      Author: Michel Koenen
 */



#ifndef CENVIRONMENTROBOT_HPP_
#define CENVIRONMENTROBOT_HPP_


class cEnvironmentRobot
{
	public:
			static cEnvironmentRobot& getInstance()
			{
				static cEnvironmentRobot instance; // Guaranteed to be destroyed.
											          // Instantiated on first use.
				return instance;
			}

			/*
			 * Getter functionality for use by anybody who needs to know
			 */
			float getRadius();
			float getRadiusMargin();

	private:
			cEnvironmentRobot();  //constructor definition
			~cEnvironmentRobot();
			cEnvironmentRobot(cEnvironmentRobot const&); // Don't Implement
			void operator=(cEnvironmentRobot const&);	   // Don't implement
			void getConfig();
			float _radius, _radiusMargin;
};

#endif /* CCENVIRONMENTROBOT_HPP_ */

