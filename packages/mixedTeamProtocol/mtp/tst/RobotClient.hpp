// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_TST_ROBOTCLIENT_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_TST_ROBOTCLIENT_HPP_

// headers from this package
#include "ext/MixedTeamProtocol.hpp"
#include "int/Communication.hpp"

// standard/system headers
// ...


// a simple simulation of a participating robot, relevant software only
// features/limitations:
// * this simple robot simulation can and will not move - such dynamical simulations are not really relevant for testing the protocol
// * robot will tell if it is ready to play (if not, then for instance it has not yet settled on a role)
// * vendor/shirt/team id cannot be reconfigured dynamically
class RobotClient
{
    public:
        RobotClient(mtp::PlayerId const &i, rtime const &t0, float frequency = 10.0, float jitter = 0.0);
        ~RobotClient();
        mtp::PlayerId id;

    public:
        // poke for an update at some frequency (10-40Hz) by controlling simulator
        void tick(rtime const &t);

        // synchronization
        std::string getFrameString() const;
        void setFrameString(std::string const &s);

        // role allocation and conflict resolution
        bool readyToPlay() const;
        mtp::RoleEnum getOwnRole() const;
        void setCurrentRole(mtp::RoleEnum const &role);
        void setPreferredRole(mtp::RoleEnum const &role);

        // status report
        std::string getPreferredRole() const;
        std::string statusReportLong() const;
        std::string statusReportBrief() const;

        // worldModel i/o
        void setOwnPosVel(mtp::Pose const &position, mtp::Pose const &velocity, float confidence);
        std::vector<mtp::TeamMember> getTeam() const;

    private:
        std::shared_ptr<mtp::MixedTeamProtocol> _mtp;
        std::shared_ptr<mtp::Communication> _comm;
        float _frequency;
        float _jitter;
        std::string _previousRole = "UNDEFINED";

};

#endif
