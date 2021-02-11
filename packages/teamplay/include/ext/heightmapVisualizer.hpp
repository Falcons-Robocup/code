// Copyright 2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapVisualizer.hpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Coen Tempelaars
 */

#ifndef HEIGHTMAPVISUALIZER_HPP_
#define HEIGHTMAPVISUALIZER_HPP_

#include "heightmapNames.hpp"
#include <opencv2/opencv.hpp>

class HeightmapVisualizer
{
public:
    HeightmapVisualizer();
    cv::Mat visualizeHeightmap (const CompositeHeightmapName& name, const int robotID);
};

#endif /* HEIGHTMAPVISUALIZER_HPP_ */
