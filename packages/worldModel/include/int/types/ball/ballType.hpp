// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballType.hpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLTYPE_HPP_
#define BALLTYPE_HPP_

#include <stddef.h>
#include "int/types/object/objectResultType.hpp"

class ballClass_t: public objectResultType
{
	public:
		ballClass_t();
		ballClass_t(objectResultType const &o) : objectResultType(o) {};
		~ballClass_t();

		/*
		 * Function needed for std::sort to sort vectors
		 */
		bool operator<(const ballClass_t& obj) const
		{
			return this->_confidence > obj._confidence;
		}
		
		// assignment operator
        ballClass_t& operator= (const ballClass_t& d)
        {
            objectResultType::operator=(d);
            _confidence = d.getConfidence();
            _isValid = d.getIsValid();
            return *this;
        }

		void setConfidence(const float confidence);
		void setIsValid(const bool isValid);

		float getConfidence() const;
		bool getIsValid() const;

	private:
		float _confidence;
		bool _isValid;
};

#endif /* BALLTYPE_HPP_ */
