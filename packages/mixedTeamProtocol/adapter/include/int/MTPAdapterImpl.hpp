// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MTPADAPTER_IMPLEMENTATION_HPP_
#define _INCLUDED_MTPADAPTER_IMPLEMENTATION_HPP_

// headers from this package
#include "ext/MTPAdapter.hpp"

// headers from MSL-MTP package
#include "MixedTeamProtocol.hpp"


class MixedTeamProtocolAdapterImpl: public MixedTeamProtocolAdapterInterface
{
public:
    MixedTeamProtocolAdapterImpl(mtp::PlayerId const &id);
    ~MixedTeamProtocolAdapterImpl();

    bool good() const;
    bool isMixedTeam() const;
    std::vector<mtp::TeamMember> getTeam(bool ownRobotsOnly) const;
    void getRefboxCommand(bool &changed, std::string &command, mtp::RefereeCommand::Arguments &commandArgs);
    std::string getOwnRole() const;
    void setOwnIntention(std::string intention);
    void setPreferredOwnRole(std::string role, float preference);
    void setOwnPosVel(float x, float y, float Rz, float vx, float vy, float vRz, float confidence);
    void setOwnBall(float x, float y, float z, float vx, float vy, float vz, float confidence);
    void setOwnBallPossession(bool hasBall);
    void tick(rtime const &t);

private:
    // data members
    mtp::PlayerId _id;
    mtp::MixedTeamProtocol _mtp;

}; // end of class MixedTeamProtocolAdapterImpl


#endif
