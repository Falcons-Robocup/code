// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTranslateTarget.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CTRANSLATETARGET_HPP_
#define CTRANSLATETARGET_HPP_

#include "int/cAbstractVelocityTransform.hpp"

class cTranslateTarget : public cAbstractVelocityTransform
{
    public:
        cTranslateTarget(cVelocityTransformMain* main) : cAbstractVelocityTransform(main) { };
        ~cTranslateTarget() { };

        void execute();
};

#endif /* CTRANSLATETARGET_HPP_ */
