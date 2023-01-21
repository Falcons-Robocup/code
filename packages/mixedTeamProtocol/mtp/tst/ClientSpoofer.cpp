// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// headers from this package
#include "ext/MixedTeamProtocol.hpp"

// standard/system headers
#include <iostream>
#include <boost/program_options.hpp>



int main(int argc, char **argv)
{
    // parse options
    namespace po = boost::program_options;
    po::options_description desc("Spoof a client. Useful to insert for example humans without actually runtime detecting them.");
    desc.add_options()
        ("help,h", "produce help message")
        ("id,i", po::value<int>()->default_value(1), "shirt id")
        ("role,r", po::value<std::string>()->default_value("UNDEFINED"), "preferred role")
        ("frequency,f", po::value<float>()->default_value(10.0), "packet transmit frequency")
        ("x", po::value<float>()->default_value(0.0), "x coordinate")
        ("y", po::value<float>()->default_value(0.0), "y coordinate")
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
    while (true)
    {
        float x = vm.at("x").as<float>();
        float y = vm.at("y").as<float>();
        // TODO: consider to use setHumanTeamMember? (optional?)
        mtp->setOwnPosVel(mtp::Pose(x, y), mtp::Pose(), 1.0);
        float preference = 0.5;
        mtp->setPreferredOwnRole(mtp::roleStringToEnum(vm.at("role").as<std::string>()), preference);
        mtp->tick(rtime::now());
        sleep(dt);
    }

    return 0;
}
