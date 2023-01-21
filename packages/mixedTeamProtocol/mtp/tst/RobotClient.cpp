// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "RobotClient.hpp"

// standard/system headers
// ...


RobotClient::RobotClient(mtp::PlayerId const &i, rtime const &t0, float frequency, float jitter)
:
    id(i),
    _mtp(std::make_unique<mtp::MixedTeamProtocol>(id, true)),
    _comm(std::make_unique<mtp::Communication>(id, true)),
    _frequency(frequency),
    _jitter(jitter)
{
    (*_mtp)->setT0(t0);
}

RobotClient::~RobotClient()
{
    // no need to delete mtp, because it is a std::unique_ptr
}

void RobotClient::tick(rtime const &t)
{
    _previousRole = mtp::roleEnumToString((*_mtp)->getOwnRole()); // for diagnostics / simulation reporting
    (*_mtp)->setCurrentTime(t);
    (*_mtp)->tick(t);
}

std::string RobotClient::getFrameString() const
{
    return _comm->getFrameString();
}

void RobotClient::setFrameString(std::string const &s)
{
    _comm->setFrameString(s);
}

bool RobotClient::readyToPlay() const
{
    return (*_mtp)->good();
}

mtp::RoleEnum RobotClient::getOwnRole() const
{
    return (*_mtp)->getOwnRole();
}

void RobotClient::setCurrentRole(mtp::RoleEnum const &role)
{
    _comm->setState("CURRENT_ROLE", (int)role);
}

void RobotClient::setPreferredRole(mtp::RoleEnum const &role)
{
    _comm->setState("PREFERRED_ROLE", (int)role);
    _comm->setState("PREFERENCE_FACTOR", 1.0);
}

void RobotClient::setOwnPosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence)
{
    _comm->setState("OWN_POS_VEL", mtp::PosVel(position, velocity, confidence));
}

std::vector<mtp::TeamMember> RobotClient::getTeam() const
{
    return (*_mtp)->getTeam();
}

const char *bool2str(bool b)
{
    return b ? "true" : "false";
}

std::string RobotClient::getPreferredRole() const
{
    mtp::RoleEnum role = (mtp::RoleEnum)_comm->getState<int>("PREFERRED_ROLE");
    if (role != mtp::RoleEnum::UNDEFINED) return mtp::roleEnumToString(role);
    return "";
}

std::string RobotClient::statusReportLong() const
{
    // long version
    char buf[280] = {0};
    sprintf(buf, "%s ready=%5s role=%s preferred=%s", 
        id.describe().c_str(), 
        bool2str(readyToPlay()), 
        mtp::roleEnumToString(getOwnRole()).c_str(),
        getPreferredRole().c_str()
        );
    return std::string(buf);
}

std::string RobotClient::statusReportBrief() const
{
    // short version
    // first abbreviate roles: ATTACKER_ASSIST -> AA, DEFENDER_MAIN -> DM, GOALKEEPER -> GK (TODO: nicer to via Roles.hpp? use case beyond MatchSimulation?)
    auto role = getOwnRole();
    std::string currentRole = mtp::roleEnumToString(role);
    size_t idx = currentRole.find('_');
    if (idx == std::string::npos) idx = 3;
    std::string abbreviatedRole = std::string(1, currentRole[0]) + std::string(1, currentRole[idx+1]);
    // now determine extra info characters
    std::string extraInfo;
    if (!readyToPlay()) extraInfo += "X";
    if ((*_mtp)->isLeader()) extraInfo += "L";
    if (_previousRole != currentRole) extraInfo += "C";
    std::string preferredRole = getPreferredRole();
    if (preferredRole.size())
    {
        if (preferredRole == currentRole) extraInfo += "P";
        if (preferredRole != currentRole) extraInfo += "Q";
    }
    if (extraInfo.size()) return abbreviatedRole + "(" + extraInfo + ")";
    return abbreviatedRole;
}
