// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/RoleAllocation.hpp"

// headers from other packages
#include "tprintf.hpp"

// system headers
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace mtp;


RoleCount mtp::roleAllocationToCount(RoleAllocation const &roles)
{
    RoleCount result;
    for (auto const& rolePair: roles)
    {
        result[rolePair.second] += 1;
    }
    return result;
}

RoleAllocationAlgorithm::RoleAllocationAlgorithm(RoleAllocationAlgorithmInput const &input)
:
    _input(input)
{
    // may not call run() from constructor
}

bool RoleAllocationAlgorithm::currentIsOk() const
{
    // check own role preference
    if (_input.preferredRoles.at(_input.myId).factor > 0.0)
    {
        if (_input.currentRoles.at(_input.myId) != _input.preferredRoles.at(_input.myId).role) return false;
    }
    // check role count against specification
    auto count = roleAllocationToCount(_input.currentRoles);
    return checkRoleCount(count);
}

void RoleAllocationAlgorithm::run()
{
    // input checks
    checkAndFillInputs();
    // check if current role allocation is good enough
    if (currentIsOk())
    {
        result = _input.currentRoles;
    }
    else
    {
        // run
        error = ERROR_UNINITIALIZED;
        result.clear();
        _run();
    }
    // check result, calculate error code
    checkResult();
}

void RoleAllocationAlgorithm::checkResult()
{
    // check role count against specification
    auto count = roleAllocationToCount(result);
    if (checkRoleCount(count))
    {
        error = 0;
    }
    else
    {
        error = mtp::ERROR_BAD_ROLE;
    }
}

void RoleAllocationAlgorithm::checkAndFillInputs()
{
    if (!_input.myId.valid())
    {
        throw std::runtime_error("RoleAllocationAlgorithm got an invalid player id (self): " + _input.myId.describe());
    }
    if (!_input.currentRoles.count(_input.myId))
    {
        _input.currentRoles[_input.myId] = mtp::RoleEnum::UNDEFINED;
    }
    if (!_input.preferredRoles.count(_input.myId))
    {
        _input.preferredRoles[_input.myId].role = mtp::RoleEnum::UNDEFINED;
        _input.preferredRoles[_input.myId].factor = 0.0;
    }
    float myPreferredRoleFactor = _input.preferredRoles.at(_input.myId).factor;
    if (myPreferredRoleFactor < 0.0 || myPreferredRoleFactor > 1.0)
    {
        throw std::runtime_error("RoleAllocationAlgorithm got an invalid role preference factor: " + std::to_string(myPreferredRoleFactor));
    }
    // check each current role, set preferredRole if not existing
    for (auto const& imap: _input.currentRoles)
    {
        if (!imap.first.valid())
        {
            throw std::runtime_error("RoleAllocationAlgorithm got an invalid player id (current roles): " + imap.first.describe());
        }
        try
        {
            std::string r = roleEnumToString(imap.second);
        }
        catch (...)
        {
            throw std::runtime_error("RoleAllocationAlgorithm got an invalid current role");
        }
        if (!_input.preferredRoles.count(imap.first))
        {
            _input.preferredRoles[imap.first].role = mtp::RoleEnum::UNDEFINED;
            _input.preferredRoles[imap.first].factor = 0.0;
        }
    }
    // check that the sizes are equal, if not, this means that extra players must be present in the preferredRoles map
    if (_input.preferredRoles.size() != _input.currentRoles.size())
    {
        throw std::runtime_error("RoleAllocationAlgorithm got extra player in preferredRoles w.r.t. currentRoles");
    }
}

std::string RoleAllocationAlgorithm::describe() const
{
    std::ostringstream ostr;
    // print algorithm result versus input
    ostr << "Result code: " << (int)error << std::endl;
    ostr << "Result allocation:" << std::endl;
    for (auto const& rolePair: result)
    {
        std::string selfString = "      ";
        if (rolePair.first == _input.myId) selfString = "[self]";
        ostr << "  " << selfString << " " << rolePair.first.describe() << ": " << std::setw(20) << std::left;
        try
        {
            ostr << mtp::roleEnumToString(rolePair.second);
        }
        catch (...)
        {
            ostr << "ERROR(" << (int)rolePair.second << ")";
        }
        std::string detailString = "";
        if (_input.currentRoles.count(rolePair.first) && _input.currentRoles.at(rolePair.first) != mtp::RoleEnum::UNDEFINED)
        {
            detailString = "(current=" + mtp::roleEnumToString(_input.currentRoles.at(rolePair.first)) + ")";
        }
        if (_input.preferredRoles.count(rolePair.first) && _input.preferredRoles.at(rolePair.first).role != mtp::RoleEnum::UNDEFINED)
        {
            detailString += "(preferred=" + mtp::roleEnumToString(_input.preferredRoles.at(rolePair.first).role) + ")";
        }
        ostr << std::setw(20) << std::left << detailString << std::endl;
    }
    return ostr.str();
}
