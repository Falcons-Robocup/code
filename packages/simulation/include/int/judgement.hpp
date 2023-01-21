// Copyright 2018-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * judgement.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#ifndef JUDGEMENT_HPP_
#define JUDGEMENT_HPP_

#include "setpieceEnum.hpp"
#include "teamID.hpp"
#include "vector2d.hpp"
#include "int/simulation_generated_enum2str.hpp" // for enums of simulation package
#include "generated_enum2str.hpp" // for enums of facilities package

struct Judgement {
    Point2D ballPosition;
    SetpieceEnum setpiece;
    TeamID teamID;
};

inline std::ostream& operator<<(std::ostream & os, const Judgement& j)
{
    return os << "Judgement{ballPos = (" << j.ballPosition.x << ", " << j.ballPosition.y << \
                "), setpiece = " << j.setpiece << ", team = " << j.teamID << "}";
}

#endif /* JUDGEMENT_HPP_ */
