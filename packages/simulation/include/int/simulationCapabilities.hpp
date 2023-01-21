// Copyright 2019-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simulationCapabilities.hpp
 *
 *  Created on: Dec 13, 2018
 *      Author: Coen Tempelaars
 */

#ifndef SIMULATION_CAPABILITIES_HPP_
#define SIMULATION_CAPABILITIES_HPP_

#include "vec3d.hpp"
#include "vec2d.hpp"
#include "pose.hpp"
#include "vector3d.hpp"
#include "vector2d.hpp"

template<typename FROM>
vec2d convert_xy(const FROM& from)
{
    vec2d to;
    to.x = from.x;
    to.y = from.y;
    return to;
}

template<typename FROM>
vec3d convert_xyz(const FROM& from)
{
    vec3d to;
    to.x = from.x;
    to.y = from.y;
    to.z = from.z;
    return to;
}

template<typename FROM>
pose convert_xyphi(const FROM& from)
{
    pose to;
    to.x = from.x;
    to.y = from.y;
    to.Rz = from.phi;
    return to;
}



#endif /* SIMULATION_CAPABILITIES_HPP_ */
