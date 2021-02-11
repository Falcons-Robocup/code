// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTranslateFeedback.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CTRANSLATEFEEDBACK_HPP_
#define CTRANSLATEFEEDBACK_HPP_

#include "int/cAbstractVelocityTransform.hpp"

class cTranslateFeedback : public cAbstractVelocityTransform
{
    public:
        cTranslateFeedback(cVelocityTransformMain* main) : cAbstractVelocityTransform(main) { };
        ~cTranslateFeedback() { };

        void execute();
};

#endif /* CTRANSLATEFEEDBACK_HPP_ */
