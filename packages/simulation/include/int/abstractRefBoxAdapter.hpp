// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractRefBoxAdapter.hpp
 *
 *  Created on: Jan 16, 2019
 *      Author: Coen Tempelaars
 */

#ifndef ABSTRACTREFBOXADAPTER_HPP_
#define ABSTRACTREFBOXADAPTER_HPP_

#include "setpieceEnum.hpp"
#include "teamID.hpp"

class AbstractRefBoxAdapter {
public:
    virtual ~AbstractRefBoxAdapter() {}

    virtual void republish() const = 0;
    virtual void sendStart() = 0;
    virtual void sendStop() = 0;
    virtual void sendSetpiece (const SetpieceEnum&, const TeamID&) = 0;
    virtual void registerGoal (const TeamID&) = 0;
};

#endif /* ABSTRACTREFBOXADAPTER_HPP_ */
