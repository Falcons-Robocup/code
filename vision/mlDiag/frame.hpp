// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef FRAME_HPP
#define FRAME_HPP

#include <QByteArray>

#define CAMERAS 4
#define ROBOTS 6
#define CAM_WINDOWS 6

class frame
{
public:
    frame();
    QByteArray objects;
    QByteArray statistics;
};

#endif // FRAME_HPP
