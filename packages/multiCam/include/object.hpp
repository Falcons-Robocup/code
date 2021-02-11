// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Jan Feitsma, december 2019

#ifndef OBJECT_STRUCT_HPP
#define OBJECT_STRUCT_HPP

// this was previously ballSt, shared with obstacle detection
struct objectSt
{
    int size; // amount of pixels
    int xClosestBy; // for visualization only
    int yCenter; // for visualization only
    cv::Rect rect; // for visualization only
    double radius; // width (related to pixels)
    double azimuth;
    double elevation;
    bool operator<(const objectSt& val) const
    {
        // sorting this struct is performed on size = amount of pixels (higher is better)
        return size > val.size;
    }
};

#endif

