// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef ROLEASSIGNMENT_HPP_
#define ROLEASSIGNMENT_HPP_

#include <map>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "int/types/Role.hpp"
#include "int/types/Robot.hpp"
#include "int/plays/PlaySelection.hpp"
#include "int/stores/GameStateStore.hpp"

#include "MTPAdapter.hpp"

namespace teamplay
{

typedef std::map<mtp::PlayerId, std::string> RoleAssignmentType;


class RoleAssignment
{
public:
    static RoleAssignment& getInstance()
    {
        static RoleAssignment instance; // Guaranteed to be destroyed.
                                        // Instantiated on first use.
        return instance;
    }

    void setBTFactory(BT::BehaviorTreeFactory& factory);
    void setMTP(MixedTeamProtocolAdapter* mtp);

    void assignRoles(); // the main function, called once per robot tick, writing role(s) to RTDB

private:
    RoleAssignment();
    RoleAssignment(RoleAssignment const&) = delete;
    RoleAssignment(RoleAssignment &&) = delete;
    RoleAssignment operator=(RoleAssignment const&) = delete;
    RoleAssignment operator=(RoleAssignment &&) = delete;
    ~RoleAssignment() = default;

    BT::BehaviorTreeFactory* _factory;
    MixedTeamProtocolAdapter* _mtpAdapter;

    bool _havePreferredRole = false; // always true for keeper, possibly also situationally
    std::string _preferredRole;
    float _preferenceFactor = 0.0;
    mtp::PlayerId _self;

    bool needUpdate(); // should MTP be requested to re-calculate
    RoleAssignmentType calculateTeamplayRoleAssignment() const; // TODO: obsolete with new MTP?
    void determineRolePreference();
    void setRolePreferenceInMTP();
    RoleAssignmentType getRolesFromMTP();
    RoleAssignmentType assignRolesFromMTP();
    RoleAssignmentType assignRolesFromTeamplayLeader();
    void traceRoleAssignment(std::string const &message, RoleAssignmentType const &roles) const;

    GameState _theGameState;
};

}

#endif // ROLEASSIGNMENT_HPP_
