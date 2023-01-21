// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_HPP_

// standard/system headers
#include <vector>
#include <string>

// headers from this package
#include "TeamMember.hpp"
#include "Object.hpp"
#include "Pose.hpp"
#include "Roles.hpp"
#include "PlayerId.hpp"
#include "Referee.hpp"

// headers from rtdb3 package
#include "rtime.hpp" // rtime is a timestamp

namespace mtp
{

class Interface
{
public:
    virtual ~Interface() {}

    // getters
    virtual bool good() const = 0; // false in case of errors, in which case robot should remain idle
    virtual bool isLeader() const = 0; // there can only be one
    virtual mtp::RoleEnum getOwnRole() const = 0;
    virtual std::vector<mtp::TeamMember> getTeam() const = 0; // get all registered team member data
    virtual std::vector<mtp::Object> getBalls() const = 0; // typically 0 or 1 balls
    //virtual mtp::BallPossession getBallPossession() const = 0; // TODO how to figure out if ball is possessed by OPPONENT of FIELD? bring worldmodel logic into this library?
    virtual std::vector<mtp::Object> getObstacles() const = 0;
    virtual RefereeCommand getLastCommand() const = 0;

    // worldModel setters
    virtual void setOwnPosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence) = 0;
    virtual void setOwnBallPossession(bool hasBall) = 0;
    virtual void setOwnBalls(std::vector<mtp::Object> balls) = 0;
    virtual void setOwnObstacles(std::vector<mtp::Object> obstacles) = 0;
    virtual void setHumanTeamMember(mtp::Pose const &position, mtp::Pose const &velocity, float confidence) = 0;

    // teamplay setter
    virtual void setOwnIntention(std::string intention) = 0;
    virtual void setPreferredOwnRole(mtp::RoleEnum const &role, float preference = 0.5) = 0; // optional, preference between 0.0 (avoid) and 1.0 (force)

    // timestamp setters (all data is timestamped w.r.t. a common t0)
    virtual void setT0(rtime const &t0) = 0;
    virtual void setCurrentTime(rtime const &t) = 0;

    // communication
    virtual void commGet() = 0;   // read from RTDB
    virtual void commPut() = 0;   // write to RTDB
    virtual void commStart() = 0; // start RTDB-comm thread
    virtual void commStop() = 0;  // stop RTDB-comm thread

    // alternatively, user clients may tick explicitly, typically synchronizing with their own WorldModel:
    // - mtp tick
    // - mtp get
    // - wordmodel update
    // - teamplay update
    // - mtp set
    // - (mtp tick -- optional to make data immediately available for other team members)
    // TODO: this story may not be uptodate anymore -- replace it with a reference to a few sequence diagrams
    virtual void tick(rtime const &t) = 0;

}; // end of class Interface

class MixedTeamProtocol
{
private:
    // smart pointer provides exception safety
    std::unique_ptr<Interface> impl;

public:
    // path_encoding causes the database name to be part of the root path:
    // - false: /tmp/rtdb2_storage/<id>/<database>/...
    // - true:  /tmp/rtdb_<database>/<id>/default/...
    MixedTeamProtocol(PlayerId const &id, bool path_encoding = false);

    // pass-through invoker
    Interface* operator->() const { return impl.get(); }

}; // end of class MixedTeamProtocol

} // end of namespace mtp

#endif
