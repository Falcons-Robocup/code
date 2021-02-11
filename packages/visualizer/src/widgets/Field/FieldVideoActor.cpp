// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0

#include "int/widgets/Field/FieldVideoActor.h"
#include "cEnvironmentField.hpp"
#include "falconsCommonDirs.hpp"

#include <vtkImageData.h>
#include <vtkImageMapper3D.h>
#include <vtkJPEGReader.h>

#include "opencv2/opencv.hpp"
#include <dirent.h>
#include <string>
#include <limits>

static const std::string VIDEOS_DIRECTORY = pathToFalconsRoot() + "/matchLogs2020/videos";

static const int VIDEO_WIDTH = 840;
static const int VIDEO_HEIGHT = 572;
static const int BORDER_SIZE = 20;

FieldVideoActor::FieldVideoActor()
{
    fill_video_list();
    video_opened = false;
    opened_video_name = "";
    is_background_green = false;
    last_frame = -1;
    flip_fielp = false;

    image_data = vtkSmartPointer<vtkImageData>::New();
    image_data->SetDimensions(VIDEO_WIDTH, VIDEO_HEIGHT, 1);
    image_data->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

    setGreenBackground();
}

FieldVideoActor::~FieldVideoActor()
{

}

void FieldVideoActor::fill_video_list()
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

void FieldVideoActor::setFieldFlip(bool flip)
{
    flip_fielp = flip;
}

void FieldVideoActor::open_video_file(double timestamp)
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

void FieldVideoActor::convertBGRtoRGB(unsigned char* mat_data_ptr)
{
    unsigned char* vtk_data_ptr = static_cast<unsigned char*>(image_data->GetScalarPointer());

    for(int i=0; i<VIDEO_WIDTH*VIDEO_HEIGHT; i++)
    {
        // openCV uses color order BGR and vtk uses RGB, so the copy needs to swap these bytes

        int b = *mat_data_ptr; mat_data_ptr++;
        int g = *mat_data_ptr; mat_data_ptr++;
        int r = *mat_data_ptr; mat_data_ptr++;

        *vtk_data_ptr = r; vtk_data_ptr++;
        *vtk_data_ptr = g; vtk_data_ptr++;
        *vtk_data_ptr = b; vtk_data_ptr++;
    }

    this->GetMapper()->SetInputData(image_data);
    this->GetMapper()->Modified();
}

void FieldVideoActor::setGreenBackground()
{
    if(!is_background_green)
    {
        is_background_green = true;

        unsigned char* vtk_data_ptr = static_cast<unsigned char*>(image_data->GetScalarPointer());

        for(int i=0; i<VIDEO_WIDTH*VIDEO_HEIGHT; i++)
        {   
            *vtk_data_ptr = 71; // Red
            vtk_data_ptr++;
            *vtk_data_ptr = 163; // Green
            vtk_data_ptr++;
            *vtk_data_ptr = 50; // Blue
            vtk_data_ptr++;                        
        }

        this->GetMapper()->SetInputData(image_data);
        this->GetMapper()->Modified();

        setFieldTransformations();
    }    
}

void FieldVideoActor::setVideoFrame(int frame_num)
{
    if(is_background_green || (last_frame != frame_num))
    {
        is_background_green = false;
        last_frame = frame_num;

        video_cap->set(cv::CAP_PROP_POS_FRAMES, frame_num);

        cv::Mat image;
        video_cap->read(image);

        convertBGRtoRGB(image.data);
        setFieldTransformations();
    }
}

void FieldVideoActor::setFieldTransformations()
{
    float LINE_THICKNESS = cEnvironmentField::getInstance().getLineThickness();
    float FIELD_LENGTH = cEnvironmentField::getInstance().getLength() - 2*LINE_THICKNESS;
    float FIELD_WIDTH = cEnvironmentField::getInstance().getWidth() - 2*LINE_THICKNESS;
    double border_in_meters = (BORDER_SIZE*FIELD_LENGTH)/static_cast<double>(VIDEO_WIDTH - 2*BORDER_SIZE);
    double video_scale = FIELD_LENGTH/static_cast<double>(VIDEO_WIDTH - 2*BORDER_SIZE);

    if(flip_fielp)
    {
        this->SetOrientation(0, 0, -90.0);
        this->SetScale(-video_scale, video_scale, 1.0);
        this->SetPosition(-(FIELD_WIDTH/2.0 + border_in_meters), -(FIELD_LENGTH/2.0 + border_in_meters), 0.0);
    }
    else
    {
        this->SetOrientation(0, 0, 90.0);
        this->SetScale(-video_scale, video_scale, 1.0);
        this->SetPosition((FIELD_WIDTH/2.0 + border_in_meters), (FIELD_LENGTH/2.0 + border_in_meters), 0.0);
    }    
}

void FieldVideoActor::updateImage(double timestamp, const CompositeHeightmapName& heightmapName, const int robotID)
{
    if (heightmapName == CompositeHeightmapName::INVALID)
    {
        if(!video_opened)
        {
            open_video_file(timestamp);
            video_opened = true;
        }

        if(video_cap == nullptr || (!video_cap->isOpened()))
        {
            setGreenBackground();
        }
        else
        {
            double dt = timestamp - video_start_time;
            double fps = video_cap->get(cv::CAP_PROP_FPS);
            int total_frame_count = video_cap->get(cv::CAP_PROP_FRAME_COUNT);

            int target_frame = dt * fps;

            if(target_frame >= 0 && target_frame < total_frame_count)
            {
                setVideoFrame(target_frame);
            }
            else
            {
                open_video_file(timestamp);
                setGreenBackground();
            }
        }
    }
    else /* Visualize heightmap */
    {
        auto image = hmv.visualizeHeightmap(heightmapName, robotID);
        cv::Mat scaledImage(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3);
        cv::resize(image, scaledImage, scaledImage.size());

        convertBGRtoRGB(scaledImage.data);

        auto field_length = cEnvironmentField::getInstance().getLength();
        auto field_width = cEnvironmentField::getInstance().getWidth();
        auto scale = field_length/static_cast<double>(VIDEO_WIDTH);

        if(!flip_fielp)
        {
            this->SetOrientation(0, 0, -90.0);
            this->SetScale(-scale, scale, 1.0);
            this->SetPosition(-(field_width/2.0), -(field_length/2.0), 0.0);
        }
        else
        {
            this->SetOrientation(0, 0, 90.0);
            this->SetScale(-scale, scale, 1.0);
            this->SetPosition((field_width/2.0), (field_length/2.0), 0.0);
        }
    }
}
