// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRefboxRelay.hpp
 *
 *  Created on: Oct 20, 2010
 *      Author: Jan Feitsma
 */

#ifndef CREFBOXRELAY_HPP_
#define CREFBOXRELAY_HPP_


#include <string>
#include "int/TCPIP_client.h"
#include "int/RTDBAdapterLogging.hpp"

// RTDB & types
#include "FalconsRTDB.hpp"


class cRefboxRelay
{
public:
    cRefboxRelay();
    ~cRefboxRelay();

    bool tick();
    void run();

private:
    // iteration-related
    double _frequency = 100.0;
    int _subsampleRtdbWrite = 1; // 100Hz
    int _subsampleMSLfeedback = 5; // 20Hz
    int _subsampleAttemptReconnect = 200; // only once in 2 seconds
    int _iteration = 0;

    // RTDB
    RtDB2 *_rtdb = NULL;
    RtDB2 *_rtdb_sim_team_B = NULL;
    T_REFBOX_CONFIG _refboxConfig;
    T_MATCH_STATE _matchState;
    T_MATCH_STATE _matchState_sim_team_B;

    // MSL feedback
    bool _doFeedbackLogging = true;
    RTDBAdapterLogging _logger;

    // refbox client
    CTCPIP_Client *_client = NULL;
    std::string _IPAddress;
    int _port = 28097;

private:
    void refboxCallBack(char const *command);
    void createClient();
    std::string teamColorString();
    std::string IPAddress();
};

#endif

