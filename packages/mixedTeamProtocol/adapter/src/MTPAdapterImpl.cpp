// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/MTPAdapterImpl.hpp"



MixedTeamProtocolAdapterImpl::MixedTeamProtocolAdapterImpl(mtp::PlayerId const &id)
:
    _id(id),
    _mtp(id, true) // as per suggestion in https://github.com/RoboCup-MSL/MixedTeamProtocol/pull/11
{
}

MixedTeamProtocolAdapterImpl::~MixedTeamProtocolAdapterImpl()
{
}

bool MixedTeamProtocolAdapterImpl::good() const
{
    return _mtp->good();
}

bool MixedTeamProtocolAdapterImpl::isMixedTeam() const
{
    return getTeam(false).size() != getTeam(true).size();
}

std::string MixedTeamProtocolAdapterImpl::getOwnRole() const
{
    return mtp::roleEnumToString(_mtp->getOwnRole());
}

std::vector<mtp::TeamMember> MixedTeamProtocolAdapterImpl::getTeam(bool ownRobotsOnly) const
{
    int FALCONS = 74; // TODO: need enum in MTP or a more common 'TeamInfo' repo?
    std::vector<mtp::TeamMember> result;
    for (auto const &m: _mtp->getTeam())
    {
        if (!ownRobotsOnly || (m.id.vendorId == FALCONS))
        {
            result.push_back(m);
        }
    }
    return result;
}

void MixedTeamProtocolAdapterImpl::getRefboxCommand(bool &changed, std::string &command, mtp::RefereeCommand::Arguments &commandArgs)
{
    static std::string lastCommand = "NONE";
    mtp::RefereeCommand mtpRefboxCommand = _mtp->getLastCommand();
    command = mtp::commandEnumToString(mtpRefboxCommand.command);
    commandArgs = mtpRefboxCommand.arguments;
    if (mtpRefboxCommand.target == mtp::TargetEnum::US)
    {
        command += "_OWN";
    }
    if (mtpRefboxCommand.target == mtp::TargetEnum::THEM)
    {
        command += "_OPP";
    }
    // rewrite/filter certain commands?
    if (command == "DROP_BALL")
    {
        command = "DROPPED_BALL";
    }
    // determine changed
    changed = command != lastCommand;
    lastCommand = command;
}

void MixedTeamProtocolAdapterImpl::setOwnIntention(std::string intention)
{
    _mtp->setOwnIntention(intention);
}

void MixedTeamProtocolAdapterImpl::setPreferredOwnRole(std::string role, float preference)
{
    _mtp->setPreferredOwnRole(mtp::roleStringToEnum(role), preference);
}

void MixedTeamProtocolAdapterImpl::setOwnPosVel(float x, float y, float Rz, float vx, float vy, float vRz, float confidence)
{
    mtp::Pose position = { x, y, Rz };
    mtp::Pose velocity = { vx, vy, vRz };
    _mtp->setOwnPosVel(position, velocity, confidence);
}

void MixedTeamProtocolAdapterImpl::setOwnBall(float x, float y, float z, float vx, float vy, float vz, float confidence)
{
    std::vector<mtp::Object> balls;
    balls.push_back( { x, y, z, vx, vy, vz, confidence } );
    _mtp->setOwnBalls(balls);
}

void MixedTeamProtocolAdapterImpl::setOwnBallPossession(bool hasBall)
{
    _mtp->setOwnBallPossession(hasBall);
}

void MixedTeamProtocolAdapterImpl::tick(rtime const &t)
{
    _mtp->tick(t);
}

