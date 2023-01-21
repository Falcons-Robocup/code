// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapVisualizer.hpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Coen Tempelaars
 */

#ifndef HEIGHTMAPVISUALIZER_HPP_
#define HEIGHTMAPVISUALIZER_HPP_

#include "HeightmapNames.hpp"
#include "vector2d.hpp"
#include <opencv2/opencv.hpp>

class MixedTeamProtocolAdapter;

class HeightmapVisualizer
{
public:
    HeightmapVisualizer();
    cv::Mat visualizeHeightmap (const CompositeHeightmapName& name, const int robotID);
    Point2D getOptimum(const CompositeHeightmapName& name, const int robotID);
private:
    void updateRobotId(const int robotID);
    int _robotID; // TODO: This is ugly, but MTP adapter doesn't have an interface to query the PlayerId it was intantiated with
    MixedTeamProtocolAdapter *_mtp; // TODO: This is ugly, but MTP adapter doesn't have a sane default constructor
};

#endif /* HEIGHTMAPVISUALIZER_HPP_ */
