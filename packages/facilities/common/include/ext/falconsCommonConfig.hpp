// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_FALCONSCOMMONCONFIG_HPP_
#define _INCLUDED_FALCONSCOMMONCONFIG_HPP_

/* Generic function to load yaml, optionally simulation- or robot-specific
 * Example 1:
 *   when key="heartBeat", it will call load heartBeat.yaml from config folder
 *   in simulation mode it will take heartBeatSim.yaml if existing
 * Example 2:
 *   when key="BallHandlers", and current robot number is 2,
 *   then if BallHandlersR2.yaml exists, it will be loaded
 *
 * This provides for a consistent way to manage configuration without having to rebuild components when switching
 * from generic to robot-specific configuration.
 */
std::string determineConfig(std::string key, std::string ext = ".yaml");

#endif

