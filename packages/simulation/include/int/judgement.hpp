// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * judgement.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#ifndef JUDGEMENT_HPP_
#define JUDGEMENT_HPP_

#include "setpiece.hpp"
#include "teamID.hpp"
#include "vector2d.hpp"

struct Judgement {
    Point2D ballPosition;
    Setpiece setpiece;
    TeamID teamID;
};

#endif /* JUDGEMENT_HPP_ */
