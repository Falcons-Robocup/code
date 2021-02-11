// Copyright 2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapVisualizerTest.cpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "ext/heightmapVisualizer.hpp"

/* SUT dependencies */
#include <opencv2/opencv.hpp>


/* Testing the heightmap visualizer */

class heightmapVisualizerTest : public TeamplayTest
{
public:
    heightmapVisualizerTest()
    {
    }
};

TEST_F(heightmapVisualizerTest, generateJPG)
{
    auto hmv = HeightmapVisualizer();
    auto image = hmv.visualizeHeightmap(CompositeHeightmapName::DRIBBLE, 1);

    cv::Mat scaledImage(600, 900, CV_8UC3);
    cv::resize(image, scaledImage, scaledImage.size());

    imwrite("/var/tmp/tst_heightmapVisualizer.jpg", scaledImage);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
