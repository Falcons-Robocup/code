// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBrefboxAdapter.hpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#ifndef RTDBREFBOXADAPTER_HPP_
#define RTDBREFBOXADAPTER_HPP_

#include "abstractRefBoxAdapter.hpp"
#include <string>

class RTDBRefBoxAdapter : public AbstractRefBoxAdapter {
public:
    RTDBRefBoxAdapter();

    virtual void republish() const;
    virtual void sendStart();
    virtual void sendStop();
    virtual void sendSetpiece (const Setpiece&, const TeamID&);
    virtual void registerGoal (const TeamID&);

private:
    void sendCommand () const;

    std::string _command_for_team_A;
    std::string _command_for_team_B;

    int _score_for_team_A;
    int _score_for_team_B;
};

#endif /* RTDBREFBOXADAPTER_HPP_ */
