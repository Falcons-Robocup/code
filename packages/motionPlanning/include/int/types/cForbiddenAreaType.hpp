// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cForbiddenAreaType.hpp
 *
 *  Created on: Dec 3, 2017
 *      Author: Jan Feitsma
 */

#ifndef CFORBIDDENAREATYPE_HPP_
#define CFORBIDDENAREATYPE_HPP_

#include <vector>
#include <string>
#include "falconsCommon.hpp"

class cForbiddenAreaType
{
  public:
    cForbiddenAreaType(std::vector<Point2D> const &points);

    bool operator==(cForbiddenAreaType const &other) const;
    bool operator<(cForbiddenAreaType const &other) const;
    std::string getHash() const;
    std::vector<Point2D> getPoints() const;
    
  private:    
    std::vector<Point2D> _points; // polygon
    std::string _hash;
    void calcHash();
};

#endif /* CFORBIDDENAREATYPE_HPP_ */
