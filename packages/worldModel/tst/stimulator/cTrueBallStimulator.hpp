// Copyright 2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTrueBallStimulator.hpp
 *
 *  Created on: Aug 2020
 *      Author: Lucas Catabriga
 */

#ifndef CTRUEBALLSTIMULATOR_HPP_
#define CTRUEBALLSTIMULATOR_HPP_

#include <string>
#include "cAbstractStimulator.hpp"

#include "opencv2/opencv.hpp"

class RtDB2;

struct Vec3d
{
    double x;
    double y;
    double z;
};

struct VideoFileInfo
{
    std::string file_name;
    double file_time;
};


class cTrueBallStimulator: public cAbstractStimulator
{
public:
    cTrueBallStimulator(int agentId, std::string inputFile, std::string outputFile, int flip);
    ~cTrueBallStimulator();
    
    bool checkFrame(tLogFrame const &frame);
    void tick(rtime const &t);

private:

    void fill_video_list();
    void open_video_file(double timestamp);
    int getTargetFrame(double timestamp);
    cv::Mat getVideoFrame(int frame_num);
    Vec3d detectBall(cv::Mat image);
    Vec3d cameraToField(Vec3d camera_pos);

    RtDB2* _rtdb;

    std::shared_ptr<cv::VideoCapture> video_cap;
    double video_start_time;
    bool video_opened;
    std::string opened_video_name;
    std::vector<VideoFileInfo> video_files;

    bool flipped;

};

#endif

