// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MTPADAPTER_HPP_
#define _INCLUDED_MTPADAPTER_HPP_

// headers from MSL-MTP package
#include "PlayerId.hpp"
#include "TeamMember.hpp"
#include "Referee.hpp"

// headers from RTDB package
#include "rtime.hpp"


mtp::PlayerId createPlayerId(char team = 'A');
mtp::PlayerId createPlayerId(int id, char team = 'A');



class MixedTeamProtocolAdapterInterface
{
public:
    virtual ~MixedTeamProtocolAdapterInterface() {}

    // false in case of errors, in which case robot should remain idle
    virtual bool good() const = 0;

    // false in case all robots are of the same vencor
    virtual bool isMixedTeam() const = 0;

    // list of team members
    // optionally without mixed-team members
    virtual std::vector<mtp::TeamMember> getTeam(bool ownRobotsOnly) const = 0;

    // get refbox command
    virtual void getRefboxCommand(bool &changed, std::string &command, mtp::RefereeCommand::Arguments &commandArgs) = 0;

    // return a readable string, rather than the protocol-internal enum
    // (see Roles.hpp in MTP package for the enum; the string names are literally mapped)
    virtual std::string getOwnRole() const = 0;

    // optional, preference between 0.0 (avoid) and 1.0 (force)
    // use this for instance to signal the desire to be goalkeeper when robot has hardware for it
    // or to claim the attacker-main role in case of a setpiece prepare while being closeby
    virtual void setPreferredOwnRole(std::string role, float preference = 0.5) = 0;

    // optional: set own intention
    virtual void setOwnIntention(std::string intention) = 0;


    // worldModel setters
    virtual void setOwnPosVel(float x, float y, float Rz, float vx, float vy, float vRz, float confidence) = 0;
    virtual void setOwnBall(float x, float y, float z, float vx, float vy, float vz, float confidence) = 0;
    virtual void setOwnBallPossession(bool hasBall) = 0;

    // run one iteration with current (perhaps simulated) timestamp
    virtual void tick(rtime const &t) = 0;

}; // end of class MixedTeamProtocolAdapterInterface



class MixedTeamProtocolAdapter
{
private:
    // smart pointer provides exception safety
    std::unique_ptr<MixedTeamProtocolAdapterInterface> impl;

public:
    MixedTeamProtocolAdapter();
    MixedTeamProtocolAdapter(mtp::PlayerId const &id);

    // pass-through invoker
    MixedTeamProtocolAdapterInterface* operator->() const { return impl.get(); }

}; // end of class MixedTeamProtocolAdapter


#endif

