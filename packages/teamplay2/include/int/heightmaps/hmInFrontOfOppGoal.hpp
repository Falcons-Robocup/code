// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmInFrontOfOppGoal.hpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HMINFRONTOFOPPGOAL_HPP_
#define HMINFRONTOFOPPGOAL_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmInFrontOfOppGoal : public AbstractHeightMap
{
public:
    hmInFrontOfOppGoal();
    virtual ~hmInFrontOfOppGoal();

    virtual void precalculate();
    virtual AbstractHeightMap refine();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

private:
    /* This heightmap does not depend on worldmodel input,
    ** hence it does not need to be recalculated every cycle */
    bool _calculation_finished;

    /* This heightmap applies to the left side of the field only,
     * the right side only, or both. The refine() function is used
     * to select the correct one */
    HeightMap_t _hmBothSides;
    HeightMap_t _hmLeftSide;
    HeightMap_t _hmRightSide;
};

} /* namespace teamplay */

#endif /* HMINFRONTOFOPPGOAL_HPP_ */
