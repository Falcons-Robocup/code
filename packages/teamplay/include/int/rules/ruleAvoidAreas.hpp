// Copyright 2017-2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleAvoidAreas.hpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

#ifndef RULEAVOIDAREAS_HPP_
#define RULEAVOIDAREAS_HPP_

#include "polygon2D.hpp"
#include "position2d.hpp"
#include "int/utilities/timer.hpp"

#include <boost/shared_ptr.hpp>
#include <vector>

class ruleAvoidAreas
{
public:
    ruleAvoidAreas(boost::shared_ptr<teamplay::timer>);
    virtual ~ruleAvoidAreas();

    virtual bool isCurrentPositionValid() const;
    virtual std::vector<polygon2D> getForbiddenAreas() const;

private:
    virtual bool isCurrentPositionInOwnPenaltyAreaValid() const;
    virtual bool isCurrentPositionInOppPenaltyAreaValid() const;
    virtual bool isCurrentPositionInOwnGoalAreaValid() const;
    virtual bool isCurrentPositionInOppGoalAreaValid() const;

    boost::shared_ptr<teamplay::timer> _timer;
};

#endif /* RULEAVOIDAREAS_HPP_ */
