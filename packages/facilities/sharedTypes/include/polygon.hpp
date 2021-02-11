// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef POLYGON_HPP_
#define POLYGON_HPP_

#include <vector>
#include "vec2d.hpp"

#include "RtDB2.h" // required for serialization


struct polygon
{
    std::vector<vec2d>   points;

    // this magic was found in geometry, polygon2D.cpp
    // (original from internet somewhere??)
    bool isPointInside(vec2d const &p)
    {
        bool retVal = false;
        size_t i, j, nvert = points.size();
        if (nvert > 2)
        {
            for (i = 0, j = nvert - 1; i < nvert; j = i++)
            {
                if (((points[i].y >= p.y ) != (points[j].y >= p.y) ) &&
                    (p.x <= (points[j].x - points[i].x) * (p.y - points[i].y) / (points[j].y - points[i].y) + points[i].x))
                {
                    retVal = !retVal;
                }
            }
        }
        return retVal;
    }

    SERIALIZE_DATA_FIXED(points);
};

#endif

