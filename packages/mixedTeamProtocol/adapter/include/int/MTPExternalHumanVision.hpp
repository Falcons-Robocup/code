// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MTP_EXTERNAL_HUMAN_VISION_HPP_
#define _INCLUDED_MTP_EXTERNAL_HUMAN_VISION_HPP_


// types from MTP package
#include "Pose.hpp"
#include "Object.hpp"


struct VisionInputData
{
    mtp::Pose human_pos;
    mtp::Pose human_vel; // human_vel by definition zero, maybe WorldModel can calculate it in future?
    float human_confidence = 0.0;
    bool ball_possession = false;
    mtp::Object ball;
};


class VisionDataAdapter
{
public:
    VisionDataAdapter();
    bool get(VisionInputData &data); // return ok
private:
};


#endif
