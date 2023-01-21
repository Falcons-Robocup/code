// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/MixedTeamProtocolImpl.hpp"

// headers from this package
#include "ext/Roles.hpp"
#include "int/Errors.hpp"

// standard/system headers
#include <cmath>
#include <algorithm>

#ifndef DEBUG
#define DEBUG false
#endif

using namespace mtp;

MixedTeamProtocolImpl::MixedTeamProtocolImpl(PlayerId const &id, bool path_encoding)
:
    _id(id),
    _communication(std::make_shared<Communication>(id, path_encoding))
{
    _rng.seed(_id.hash());
}

MixedTeamProtocolImpl::~MixedTeamProtocolImpl()
{
}

bool MixedTeamProtocolImpl::good() const
{
    return _communication->getState<bool>("GOOD");
}

bool MixedTeamProtocolImpl::isLeader() const
{
    return _communication->getState<bool>("IS_LEADER");
}

RoleEnum MixedTeamProtocolImpl::getOwnRole() const
{
    return (RoleEnum)_communication->getState<int>("CURRENT_ROLE");
}

std::vector<mtp::TeamMember> MixedTeamProtocolImpl::getTeam() const
{
    std::vector<mtp::TeamMember> result;
    for (auto const &player: _players)
    {
        // convert player packet to TeamMember
        mtp::TeamMember t(player.second.id);
        t.role = roleEnumToString((mtp::RoleEnum)player.second.packet.role);
        if (!player.second.packet.self_loc.empty()) {
            auto const &pv = player.second.packet.self_loc.front();
            t.position.x = pv.x;
            t.position.y = pv.y;
            t.position.rz = pv.rz;
            t.velocity.x = pv.vx;
            t.velocity.y = pv.vy;
            t.velocity.rz = pv.vrz;
            // TODO: what to do with confidence?
        }
        t.hasBall = player.second.packet.has_ball;
        // TODO: set t.intention
        result.push_back(t);
    }
    return result;
}

std::vector<mtp::Object> MixedTeamProtocolImpl::getBalls() const
{
    // TODO
    return std::vector<mtp::Object>();
}

std::vector<mtp::Object> MixedTeamProtocolImpl::getObstacles() const
{
    // TODO
    return std::vector<mtp::Object>();
}

RefereeCommand MixedTeamProtocolImpl::getLastCommand() const
{
    return _communication->getLastCommand();
}

void MixedTeamProtocolImpl::setOwnPosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence)
{
    mtp::PosVel pv;
    pv.x = position.x;
    pv.y = position.y;
    pv.rz = position.rz;
    pv.vx = velocity.x;
    pv.vy = velocity.y;
    pv.vrz = velocity.rz;
    pv.confidence = confidence;
    _communication->setState("OWN_POS_VEL", pv);
}

void MixedTeamProtocolImpl::setOwnBalls(std::vector<mtp::Object> balls)
{
    // TODO
}

void MixedTeamProtocolImpl::setOwnBallPossession(bool hasBall)
{
    _communication->setState("HAS_BALL", hasBall);
}

void MixedTeamProtocolImpl::setOwnObstacles(std::vector<mtp::Object> obstacles)
{
    // TODO
}

void MixedTeamProtocolImpl::setHumanTeamMember(mtp::Pose const &position, mtp::Pose const &velocity, float confidence)
{
    // TODO
}

void MixedTeamProtocolImpl::setOwnIntention(std::string intention)
{
    // TODO
}

void MixedTeamProtocolImpl::setPreferredOwnRole(RoleEnum const &role, float preference)
{
    _communication->setState("PREFERRED_ROLE", (int)role);
    _communication->setState("PREFERENCE_FACTOR", preference);
}

void MixedTeamProtocolImpl::setT0(rtime const &t0)
{
    _t0 = t0;
}

void MixedTeamProtocolImpl::setCurrentTime(rtime const &t)
{
    _tc = t;
}

void MixedTeamProtocolImpl::commGet()
{
    auto k = _communication->getPlayerPackets();
    updatePlayers(k);
}

void MixedTeamProtocolImpl::commPut()
{
    // send data packet
    _communication->setPlayerPacket(makePacket());
}

void MixedTeamProtocolImpl::commStart()
{
    _communication->startThread();
}

void MixedTeamProtocolImpl::commStop()
{
    _communication->stopThread();
}

void MixedTeamProtocolImpl::tick(rtime const &t)
{
    // prepare
    _error = 0;
    // check if started
    //if (!_started) throw std::runtime_error("protocol violation: start() needs to be called first"); // ? TODO cleanup
    // check for new packets
    commGet();
    // determine if robot is the leader and act accordingly
    calculateLeader();
    // worldModel processing (always, regardless of errors)
    calculateWorldModel(); // TODO: discuss: to which extent do we want to bring WM responsibility into here?
    // determine own role, if needed
    calculateOwnRole();
    // calculate _good and _error flags
    calculateGood();
    // send data packet
    commPut();
}

void MixedTeamProtocolImpl::updatePlayers(std::vector<PlayerPacket> packets)
{
    if (DEBUG) tprintf("player %s got %d packets", _id.describe().c_str(), (int)packets.size());
    // process new packets
    for (auto& packet: packets)
    {
        // determine PlayerId and PlayerIdHash
        PlayerId id(packet.vendor_id, packet.shirt_id, packet.team_id);
        if (DEBUG) tprintf("-> %s", id.describe().c_str());
        PlayerIdHash c = id.hash();
        // ignore packet if from different team
        if (packet.team_id != _id.teamId) continue;
        // create if not yet existing
        if (!_players.count(c)) _players.insert(std::make_pair(c, Player(id)));
        // update Player object with packet
        _players.find(c)->second.update(packet);
    }
    // remove timed-out players
    float timeout = 7.0; // TODO make configurable
    for (auto it = _players.begin(); it != _players.end(); )
    {
        // calculate age in seconds, assume t0 synchronized
        rtime lastAlive = _t0 + 1e-3 * it->second.packet.timestamp_ms;
        bool isTimedOut = lastAlive - _tc > timeout;
        if (isTimedOut) it = _players.erase(it);
        else it++;
    }
}

void MixedTeamProtocolImpl::calculateLeader()
{
    // for now: the first player in the map is leader
    // this typically is the player with lowest hash
    bool isLeader = (_players.begin()->first == _id.hash());
    _communication->setState("IS_LEADER", isLeader);
    // perform leader duties
    if (isLeader)
    {
        calculateRoleAllocation(); // for every robot - send at end of tick, so next tick all robots will take their assigned roles
    }
}

void MixedTeamProtocolImpl::calculateWorldModel()
{
    // TODO??? needs careful consideration how far we need/want to go...
}

void MixedTeamProtocolImpl::calculateGood()
{
    bool good = (_error == 0);
    if (good)
    {
        // check if current role allocation is valid
        auto currentRoles = _roleAllocation;
        auto count = roleAllocationToCount(currentRoles);
        good = checkRoleCount(count);
    }
    _communication->setState("GOOD", good);
}

RoleAllocation MixedTeamProtocolImpl::getCurrentRoleAllocation()
{
    RoleAllocation result;
    // count roles from team members
    for (auto const& player: _players)
    {
        result[player.second.id] = RoleEnum(player.second.packet.role);
    }
    // don't forget self
    result[_id] = getOwnRole();
    return result;
}

void MixedTeamProtocolImpl::calculateRoleAllocation()
{
    _roleAllocation.clear();
    // construct input
    RoleEnum preferredRole = RoleEnum(_communication->getState<int>("PREFERRED_ROLE"));
    RoleAllocationAlgorithmInput input(_id);
    input.currentRoles = getCurrentRoleAllocation();
    if (preferredRole != RoleEnum::UNDEFINED)
    {
        input.preferredRoles[_id].role = RoleEnum(preferredRole);
        input.preferredRoles[_id].factor = _communication->getState<float>("PREFERENCE_FACTOR");
    }
    for (auto const& player: _players) // team members (excluding self)
    {
        if (player.second.packet.role_preference.size() == 1)
        {
            input.preferredRoles[player.second.id].role = RoleEnum(player.second.packet.role_preference.at(0).first);
            input.preferredRoles[player.second.id].factor = player.second.packet.role_preference.at(0).second;
        }
    }
    // run the algorithm
    RoleAllocationAlgorithmKuhnMunkres algo(input);
    algo.run();
    if (DEBUG) tprintf("algorithm result:\n%s", algo.describe().c_str());
    // handle result
    _error |= algo.error; 
    if (algo.error == 0)
    {
        _roleAllocation = algo.result;
    }
}

RoleAllocation MixedTeamProtocolImpl::getRoleAllocationFromLeader()
{
    mtp::RoleAllocation result;
    for (auto const& player: _players) // team members (excluding self)
    {
        if (player.second.packet.is_leader)
        {
            for (auto const& rolepair: player.second.packet.role_allocation)
            {
                int playerHash = rolepair.first;
                RoleEnum role = RoleEnum(rolepair.second);
                if (_players.count(playerHash))
                {
                    result[_players.at(playerHash).id] = role;
                }
            }
        }
    }
    return result;
}

void MixedTeamProtocolImpl::calculateOwnRole()
{
    if (!_communication->getState<bool>("IS_LEADER"))
    {
        _roleAllocation = getRoleAllocationFromLeader();
        if (_roleAllocation.size())
        {
            if (DEBUG) tprintf("player %s got role allocation (size %d) from leader", _id.describe().c_str(), _roleAllocation.size());
        }
    } // otherwise _roleAllocation is already set
    if (_roleAllocation.count(_id))
    {
        _communication->setState("CURRENT_ROLE", (int)_roleAllocation.at(_id));
    }
}

PlayerPacket MixedTeamProtocolImpl::makePacket() const
{
    PlayerPacket result;
    result.vendor_id = _id.vendorId;
    result.shirt_id = _id.shirtId;
    result.team_id = _id.teamId;
    result.timestamp_ms = int(round(double(_tc - _t0) * 1000));
    result.self_loc.push_back(_communication->getState<mtp::PosVel>("OWN_POS_VEL"));
    result.has_ball = _communication->getState<bool>("HAS_BALL");
    // TODO: balls, obstacles
    result.is_leader = _communication->getState<bool>("IS_LEADER");
    if (result.is_leader)
    {
        for (auto const& rp: _roleAllocation)
        {
            result.role_allocation.push_back(std::make_pair(rp.first.hash(), (uint8_t)rp.second));
        }
    }
    mtp::PreferredRole pr;
    pr.role = (RoleEnum)_communication->getState<int>("PREFERRED_ROLE");
    pr.factor = _communication->getState<float>("PREFERENCE_FACTOR");
    if ((mtp::RoleEnum)pr.role != mtp::RoleEnum::UNDEFINED)
    {
        result.role_preference.push_back(std::make_pair((uint8_t)pr.role, pr.factor));
    }
    result.role = (uint8_t)_communication->getState<int>("CURRENT_ROLE");
    result.intention = 0; // TODO
    result.error = _error;
    return result;
}
