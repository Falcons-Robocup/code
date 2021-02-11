// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef FIELDVIDEOACTOR_H
#define FIELDVIDEOACTOR_H

#include <QObject>
#include <vtkImageActor.h>
#include <vtkSmartPointer.h>

#include <memory>

#include "heightmapVisualizer.hpp"

class vtkImageData;

namespace cv
{
    class VideoCapture;
}

struct VideoFileInfo
{
    std::string file_name;
    double file_time;
};

class FieldVideoActor : public vtkImageActor
{
public:
    static FieldVideoActor* New()
    {
        return new FieldVideoActor();
    }

    FieldVideoActor();
    virtual ~FieldVideoActor();

    void updateImage(double timestamp, const CompositeHeightmapName& heightmapName, const int robotID);
    void setFieldFlip(bool flip);

private:    
    vtkSmartPointer<vtkImageData> image_data;
    std::shared_ptr<cv::VideoCapture> video_cap;
    double video_start_time;
    bool video_opened;
    std::string opened_video_name;
    bool is_background_green;
    int last_frame;
    bool flip_fielp;
    HeightmapVisualizer hmv;

    std::vector<VideoFileInfo> video_files;

    void fill_video_list();
    void open_video_file(double timestamp);
    void convertBGRtoRGB(unsigned char* mat_data_ptr);
    void setGreenBackground();
    void setVideoFrame(int frame_num);
    void setFieldTransformations();
};

#endif
