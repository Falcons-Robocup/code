// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPublishTarget.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CPUBLISHTARGET_HPP_
#define CPUBLISHTARGET_HPP_

#include "int/cAbstractVelocityTransform.hpp"

class cPublishTarget : public cAbstractVelocityTransform
{
    public:
        cPublishTarget(cVelocityTransformMain* main) : cAbstractVelocityTransform(main) { };
        ~cPublishTarget() { };

        void execute();
};

#endif /* CPUBLISHTARGET_HPP_ */
