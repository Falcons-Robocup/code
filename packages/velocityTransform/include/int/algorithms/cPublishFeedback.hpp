// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPublishFeedback.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CPUBLISHFEEDBACK_HPP_
#define CPUBLISHFEEDBACK_HPP_

#include "int/cAbstractVelocityTransform.hpp"

class cPublishFeedback : public cAbstractVelocityTransform
{
    public:
        cPublishFeedback(cVelocityTransformMain* main) : cAbstractVelocityTransform(main) { };
        ~cPublishFeedback() { };

        void execute();
};

#endif /* CPUBLISHFEEDBACK_HPP_ */
