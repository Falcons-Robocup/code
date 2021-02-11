// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cEnvironmentBall.hpp
 *
 *  Created on: Jan 3, 2016
 *      Author: Michel Koenen
 */



#ifndef CENVIRONMENTBALL_HPP_
#define CENVIRONMENTBALL_HPP_


class cEnvironmentBall
{
	public:
			static cEnvironmentBall& getInstance()
			{
				static cEnvironmentBall instance; // Guaranteed to be destroyed.
											          // Instantiated on first use.
				return instance;
			}

			/*
			 * Getter functionality for use by anybody who needs to know
			 */
			float getRadius();


	private:
			cEnvironmentBall();  //constructor definition
			~cEnvironmentBall();
			cEnvironmentBall(cEnvironmentBall const&); // Don't Implement
			void operator=(cEnvironmentBall const&);	   // Don't implement
			void getConfig();
			float _radius;
};

#endif /* CCENVIRONMENTBALL_HPP_ */

