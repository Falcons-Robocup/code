// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleType.hpp
 *
 *  Created on: Aug 16, 2016
 *      Author: robocup
 */

#ifndef OBSTACLETYPE_HPP_
#define OBSTACLETYPE_HPP_

#include <stddef.h>
#include "int/types/object/objectResultType.hpp"

class obstacleClass_t: public objectResultType
{
	public:
		obstacleClass_t();
		obstacleClass_t(objectResultType const &o) : objectResultType(o) {};
		~obstacleClass_t();

		/*
		 * Function needed for std::sort to sort vectors
		 */
		bool operator<(const obstacleClass_t& obj) const
		{
			return this->getConfidence() > obj.getConfidence();
		}

		// assignment operator
        obstacleClass_t& operator= (const obstacleClass_t& d)
        {
            objectResultType::operator=(d);
            return *this;
        }

		void setConfidence(const float confidence);
		float getConfidence() const;

	private:
		float _confidence;
		// TODO try to determine obstacle orientation?
};

#endif /* OBSTACLETYPE_HPP_ */
