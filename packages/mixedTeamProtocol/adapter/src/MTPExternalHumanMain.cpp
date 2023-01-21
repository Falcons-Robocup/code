// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// based on MTP ClientSpoofer.cpp

// headers from MTP package
#include "MixedTeamProtocol.hpp"

// headers from Vision - machine learning adapter
#include "int/MTPExternalHumanVision.hpp"

// standard/system headers
#include <iostream>
#include <boost/program_options.hpp>


int main(int argc, char **argv)
{
    // parse options
    namespace po = boost::program_options;
    po::options_description desc("External human client MTP packet generator.");
    desc.add_options()
        ("help,h", "produce help message")
        ("id,i", po::value<int>()->default_value(9), "shirt id")
        ("role,r", po::value<std::string>()->default_value("UNDEFINED"), "preferred role")
        ("frequency,f", po::value<float>()->default_value(1.0), "packet transmit frequency")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }

    // setup
    mtp::PlayerId myId(1, vm.at("id").as<int>());
    mtp::MixedTeamProtocol mtp(myId, true);
    mtp->commStart();

    // loop
    float dt = 1.0 / vm.at("frequency").as<float>();
    VisionDataAdapter v;
    VisionInputData data;
    while (true)
    {
        if (v.get(data))
        {
            // TODO: consider to use setHumanTeamMember? (optional?) need a fundamental discussion on how extensive WM in MTP should be
            mtp->setOwnPosVel(data.human_pos, data.human_vel, data.human_confidence);
            // TODO: call mtp->setOwnBalls, when implemented in MTP
            mtp->setOwnBallPossession(data.ball_possession);
            float preference = 0.5;
            mtp->setPreferredOwnRole(mtp::roleStringToEnum(vm.at("role").as<std::string>()), preference);
            mtp->tick(rtime::now());
        }
        sleep(dt);
    }

    return 0;
}
