// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightmapVisualizerTest.cpp
 *
 *  Created on: Mar 8, 2020
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "ext/HeightmapVisualizer.hpp"

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
    int robotID = 1;

    // Set TURTLE5K_ROBOTNUMBER to robotID, otherwise MixedTeamProtocol will throw an exception
    setenv("TURTLE5K_ROBOTNUMBER", std::to_string(robotID).c_str(), (int)true /*overwrite=true*/);

    auto hmv = HeightmapVisualizer();
    auto image = hmv.visualizeHeightmap(CompositeHeightmapName::DRIBBLE, robotID);

    cv::Mat scaledImage(600, 900, CV_8UC3);
    cv::resize(image, scaledImage, scaledImage.size());

    imwrite("/var/tmp/tst_heightmapVisualizer.jpg", scaledImage);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
