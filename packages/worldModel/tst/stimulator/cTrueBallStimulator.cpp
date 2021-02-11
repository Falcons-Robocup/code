// Copyright 2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTrueBallStimulator.cpp
 *
 *  Created on: Aug 2020
 *      Author: Lucas Catabriga
 */


// system includes
#include <cstdio>
#include <dirent.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// other Falcons packages
#include "falconsCommon.hpp"
#include "tracing.hpp"
#include "FalconsRtDB2.hpp"
#include "cEnvironmentField.hpp"

// internal includes
#include "cTrueBallStimulator.hpp"

static const std::string VIDEOS_DIRECTORY = "/home/robocup/falcons/matchLogs2020/videos";

static const int VIDEO_WIDTH = 840;
static const int VIDEO_HEIGHT = 572;
static const int BORDER_SIZE = 20;

cTrueBallStimulator::cTrueBallStimulator(int agentId, std::string inputFile, std::string outputFile, int flip)
{
    _inputFile = inputFile;
    _outputFile = outputFile;
    _agentId = agentId; // TODO I think we can strip this -- script sets env, wm knows itself which agent it is
    _component = "worldModel";

    _rtdb = RtDB2Store::getInstance().getRtDB2(getRobotNumber());

    fill_video_list();
    video_opened = false;

    flipped = flip == 1;

    INIT_TRACE;
}

cTrueBallStimulator::~cTrueBallStimulator()
{

}

void cTrueBallStimulator::fill_video_list()
{
    DIR* dir = opendir(VIDEOS_DIRECTORY.c_str());

    if(dir != NULL)
    {
        while(true)
        {
            struct dirent* ent = readdir(dir);            
            if(ent == NULL)
            {
                break;
            }

            try
            {
                VideoFileInfo file_info;

                file_info.file_name = VIDEOS_DIRECTORY + "/" + ent->d_name;
                file_info.file_time = std::stod(ent->d_name);

                video_files.push_back(file_info);
            }
            catch(...)
            {
                // Do nothing, file will be ignored
            }
        }
      
        closedir (dir);
    }
}

void cTrueBallStimulator::open_video_file(double timestamp)
{
    double min_dist = std::numeric_limits<double>::max();
    std::vector<VideoFileInfo>::iterator best_video = video_files.begin();

    for(std::vector<VideoFileInfo>::iterator it = video_files.begin(); it != video_files.end(); it++)
    {
        double dist = timestamp - it->file_time;
    
        // Only look for videos that start before the current time
        if(dist > 0 && dist < min_dist)
        {
            min_dist = dist;
            best_video = it;
        }
    }

    if((best_video != video_files.end()) && (best_video->file_name != opened_video_name))
    {
        printf("Opening video: %s\n", best_video->file_name.c_str());
        video_cap = std::make_shared<cv::VideoCapture>(best_video->file_name);
        video_start_time = best_video->file_time;
        opened_video_name = best_video->file_name;
    }
}

bool cTrueBallStimulator::checkFrame(tLogFrame const &frame)
{
    return true; // TODO OK?
    /*
    // we don't introspect the data, we use the fact that the logger writes per agent a local and a shared frame on some frequency
    // so whenever we see such a frame, we have to recalculate
    if ((_agentId == frame.agent) && (frame.shared == true))
    {
        // note: here we could store things for use in tick, if we'd need to
        return true;
    }
    return false;
    */
}

int cTrueBallStimulator::getTargetFrame(double timestamp)
{
    double dt = timestamp - video_start_time;
    double fps = video_cap->get(cv::CAP_PROP_FPS);    

    int target_frame = dt * fps;

    return target_frame;
}

cv::Mat cTrueBallStimulator::getVideoFrame(int frame_num)
{
    video_cap->set(cv::CAP_PROP_POS_FRAMES, frame_num);

    cv::Mat image;
    video_cap->read(image);

    return image;   
}

Vec3d cTrueBallStimulator::cameraToField(Vec3d camera_pos)
{
    float LINE_THICKNESS = cEnvironmentField::getInstance().getLineThickness();
    float FIELD_LENGTH = cEnvironmentField::getInstance().getLength() - 2*LINE_THICKNESS;
    float FIELD_WIDTH = cEnvironmentField::getInstance().getWidth() - 2*LINE_THICKNESS;
    double border_in_meters = (BORDER_SIZE*FIELD_LENGTH)/static_cast<double>(VIDEO_WIDTH - 2*BORDER_SIZE);
    double video_scale = FIELD_LENGTH/static_cast<double>(VIDEO_WIDTH - 2*BORDER_SIZE);

    Vec3d fieldPos;

    if (flipped)
    {
        fieldPos.x = camera_pos.y * video_scale - ( FIELD_WIDTH / 2.0 + border_in_meters);
        fieldPos.y = camera_pos.x * video_scale - (FIELD_LENGTH / 2.0 + border_in_meters);
        fieldPos.z = 0.0;
    }
    else
    {
        fieldPos.x = -camera_pos.y * video_scale + ( FIELD_WIDTH / 2.0 + border_in_meters);
        fieldPos.y = -camera_pos.x * video_scale + (FIELD_LENGTH / 2.0 + border_in_meters);
        fieldPos.z = 0.0;
    }
    
    return fieldPos;
}

Vec3d cTrueBallStimulator::detectBall(cv::Mat image)
{
    cv::Mat hsv_img;
    cv::cvtColor(image, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat yellow_img;
    inRange(hsv_img, cv::Scalar(20, 120, 150), cv::Scalar(40, 255, 255), yellow_img);

    //v::imshow("Yellow", yellow_img);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(yellow_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    double max_area = 0;
    unsigned int best_contour = 0;
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if(area > max_area)
        {
            max_area = area;
            best_contour = i;
        }

        //drawContours(image, contours, i, cv::Scalar(0,0,255), 2, 8, hierarchy, 0, cv::Point());
    }

    Vec3d ballPos;

    if (contours.size() > 0)
    {
        Vec3d ballPosImg;
        cv::Moments mu = cv::moments(contours[best_contour]);
        ballPosImg.x = mu.m10 / mu.m00;
        ballPosImg.y = mu.m01 / mu.m00;
        ballPosImg.z = 0.0;

        ballPos = cameraToField(ballPosImg);
    }
    else
    {
        ballPos.x = std::numeric_limits<double>::quiet_NaN();
        ballPos.y = std::numeric_limits<double>::quiet_NaN();
        ballPos.z = std::numeric_limits<double>::quiet_NaN();    
    }

    //std::cout << "showing image" << std::endl;
    //cv::imshow("Original", image);
    
    //cv::waitKey(0);

    return ballPos;
}

void cTrueBallStimulator::tick(rtime const &t)
{
    T_DIAG_TRUE_BALL trueBall;

    double timestamp = t.toDouble();

    if(!video_opened)
    {
        open_video_file(timestamp);
        video_opened = true;
    }

    if(video_cap != nullptr && video_cap->isOpened())
    {   
        int target_frame = getTargetFrame(timestamp);
        int total_frame_count = video_cap->get(cv::CAP_PROP_FRAME_COUNT);
        
        if(target_frame < 0 || target_frame >= total_frame_count)
        {
            open_video_file(timestamp);            
            target_frame = getTargetFrame(timestamp);
            total_frame_count = video_cap->get(cv::CAP_PROP_FRAME_COUNT);
        }

        if(target_frame >= 0 && target_frame < total_frame_count)
        {
            cv::Mat image = getVideoFrame(target_frame);
            Vec3d ball = detectBall(image);

            trueBall.x = ball.x;
            trueBall.y = ball.y;
            trueBall.z = ball.z;
        }       
    }

    trueBall.timestamp = t;
    
    _rtdb->put(DIAG_TRUE_BALL, &trueBall);
}

