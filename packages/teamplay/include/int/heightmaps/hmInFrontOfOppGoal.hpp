// Copyright 2017-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmInFrontOfOppGoal.hpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HMINFRONTOFOPPGOAL_HPP_
#define HMINFRONTOFOPPGOAL_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmInFrontOfOppGoal : public abstractHeightMap
{
public:
    hmInFrontOfOppGoal();
    virtual ~hmInFrontOfOppGoal();

    virtual void precalculate();
    virtual abstractHeightMap refine(const parameterMap_t&);
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

private:
    /* This heightmap does not depend on worldmodel input,
    ** hence it does not need to be recalculated every cycle */
    bool _calculation_finished;

    /* This heightmap applies to the left side of the field only,
     * the right side only, or both. The refine() function is used
     * to select the correct one */
    heightMap_t _hmBothSides;
    heightMap_t _hmLeftSide;
    heightMap_t _hmRightSide;
};

} /* namespace teamplay */

#endif /* HMINFRONTOFOPPGOAL_HPP_ */
