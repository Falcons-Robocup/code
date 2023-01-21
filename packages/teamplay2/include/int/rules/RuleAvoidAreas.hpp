// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleAvoidAreas.hpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

#ifndef RULEAVOIDAREAS_HPP_
#define RULEAVOIDAREAS_HPP_

#include "polygon2D.hpp"
#include "int/utilities/Timer.hpp"

#include <boost/shared_ptr.hpp>
#include <vector>

class RuleAvoidAreas
{
public:
    RuleAvoidAreas(boost::shared_ptr<teamplay::Timer>);
    virtual ~RuleAvoidAreas();

    virtual bool isCurrentPositionValid() const;
    virtual std::vector<polygon2D> getForbiddenAreas() const;

private:
    virtual bool isCurrentPositionInOwnPenaltyAreaValid() const;
    virtual bool isCurrentPositionInOppPenaltyAreaValid() const;
    virtual bool isCurrentPositionInOwnGoalAreaValid() const;
    virtual bool isCurrentPositionInOppGoalAreaValid() const;

    boost::shared_ptr<teamplay::Timer> _timer;
};

#endif /* RULEAVOIDAREAS_HPP_ */
