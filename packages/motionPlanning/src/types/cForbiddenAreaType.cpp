// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cForbiddenAreaType.cpp
 *
 *  Created on: Dec 3, 2017
 *      Author: Jan Feitsma
 */

#include "int/types/cForbiddenAreaType.hpp"
#include <string>

cForbiddenAreaType::cForbiddenAreaType(std::vector<Point2D> const &points)
{
    _points = points;
    calcHash();
}

std::string cForbiddenAreaType::getHash() const
{
    return _hash;
}

std::vector<Point2D> cForbiddenAreaType::getPoints() const
{
    return _points;
}

void cForbiddenAreaType::calcHash()
{
    _hash = "";
    for (auto p = _points.begin(); p != _points.end(); ++p)
    {
        char buf[64] = {0};
        sprintf(buf, "%7.2f %7.2f", p->x, p->y);
        _hash += buf;
    } 
}

bool cForbiddenAreaType::operator==(cForbiddenAreaType const &other) const
{
    return _hash == other.getHash();
}

bool cForbiddenAreaType::operator<(cForbiddenAreaType const &other) const
{
    return _hash < other.getHash();
}

