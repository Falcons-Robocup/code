// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_IMPLEMENTATION_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_IMPLEMENTATION_HPP_

// headers from this package
#include "ext/MixedTeamProtocol.hpp"
#include "ext/PlayerId.hpp"
#include "ext/Referee.hpp"
#include "ext/Roles.hpp"
#include "int/PlayerPacket.hpp"
#include "int/Player.hpp"
#include "int/Communication.hpp"
#include "int/RoleAllocation.hpp"

// standard/system headers
#include <random>

namespace mtp
{

class MixedTeamProtocolImpl: public Interface
{
public:
    MixedTeamProtocolImpl(PlayerId const &id, bool path_encoding = false);
    ~MixedTeamProtocolImpl();

    // implementation of the API
    bool good() const;
    bool isLeader() const;
    mtp::RoleEnum getOwnRole() const;
    std::vector<mtp::TeamMember> getTeam() const;
    std::vector<mtp::Object> getBalls() const;
    //mtp::BallPossession getBallPossession() const;
    std::vector<mtp::Object> getObstacles() const;
    RefereeCommand getLastCommand() const;
    void setOwnPosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence);
    void setOwnBalls(std::vector<mtp::Object> balls);
    void setOwnBallPossession(bool hasBall);
    void setOwnObstacles(std::vector<mtp::Object> obstacles);
    void setHumanTeamMember(mtp::Pose const &position, mtp::Pose const &velocity, float confidence);
    void setOwnIntention(std::string intention);
    void setPreferredOwnRole(mtp::RoleEnum const &role, float preference);
    void setT0(rtime const &t0);
    void setCurrentTime(rtime const &t);
    void commGet();
    void commPut();
    void commStart();
    void commStop();
    void tick(rtime const &t);

private:
    // other data members
    bool _good = false;
    PlayerId _id;
    bool _started = false;
    rtime _t0, _tc;
    uint8_t _error = 0;
    RoleAllocation _roleAllocation;
    std::default_random_engine _rng;
    std::shared_ptr<Communication> _communication;
    std::map<PlayerIdHash, Player> _players;

    // functions
    void updatePlayers(std::vector<PlayerPacket> packets);
    void calculateLeader();
    void calculateWorldModel();
    void calculateGood();
    RoleAllocation getCurrentRoleAllocation();
    RoleAllocation getRoleAllocationFromLeader();
    void calculateRoleAllocation(); // only done by leader
    void calculateOwnRole();
    PlayerPacket makePacket() const;
    PosVel toPosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence);

}; // end of class MixedTeamProtocolImpl

} // end of namespace mtp


#endif
