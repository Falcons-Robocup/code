// Copyright 2021-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QImage>
#include <unistd.h> // usleep

#include "detector.hpp"

detector::detector(QImage *rgbImage) {
    this->rgbImage = rgbImage;
    objects.frame_id = 0;

    // NOTE load_network_custom also performs fuse_conv_batchnorm
    // load_network_custom(cfg_file_name, weights_file_name, clear, batch_size);
    net = load_network_custom((char *) configFile.c_str(), (char *) weightsFile.c_str(), 1, 1); // clear and batch size 1

    calculate_binary_weights(*net);

    srand(2222222); // set the seed, but is the random function used during detect?
}

void detector::update(const QByteArray jpegData) {
    // use opencv to read image
    // cv::Mat origMat = imread("/home/robocup/robocup_ml/20200123/r1/cam0_20200123_210013.jpg", IMREAD_COLOR);
    QImage jpegImage;
    jpegImage.loadFromData(jpegData, "JPEG");

    image origImg = QImageToImage(jpegImage);
    image resizeImg = resize_image(origImg, net->w, net->h); // resize to 416x416

    // perform the object detection
    double timeStart = get_time_point();
    network_predict_ptr(net, (float *)resizeImg.data);
    double calcTime = get_time_point() - timeStart;

    // create bounding boxes (with relative size) for the detected objects
    int nboxes = 0;
    // get_network_boxes(net, width, height, threshold, hier_threshold, map, relative, num_boxes, letter);
    // NOTE: hier_threshold not used, custom_get_region_detections
    detection *dets = get_network_boxes(net, 1, 1, thresh, 0, 0, 1, &nboxes, 0);

    // one detected box can be classified for multiple classes, find the best class for each box
    diounms_sort(dets, nboxes, classes, nms_thresh, nms_kind, 0); // https://github.com/Zzh-tju/DIoU-darknet

    // copy the objects to a struct
    objects = detectionToStruct(dets, nboxes, classes, calcTime);
    if( objects.frame_id % 10 == 0 ) {
        printObjects(objects);
    }

    free_detections(dets, nboxes);
    free_image(origImg);
    free_image(resizeImg);

}

void detector::updateQimage() {
    QImage input = *rgbImage;
    image origImg = QImageToImage(input);
    image resizeImg = resize_image(origImg, net->w, net->h); // resize to 416x416

    // perform the object detection
    double timeStart = get_time_point();
    network_predict_ptr(net, (float *)resizeImg.data);
    double calcTime = get_time_point() - timeStart;

    // create bounding boxes (with relative size) for the detected objects
    int nboxes = 0;
    // get_network_boxes(net, width, height, threshold, hier_threshold, map, relative, num_boxes, letter);
    // NOTE: hier_threshold not used, custom_get_region_detections
    detection *dets = get_network_boxes(net, 1, 1, thresh, 0, 0, 1, &nboxes, 0);

    // one detected box can be classified for multiple classes, find the best class for each box
    diounms_sort(dets, nboxes, classes, nms_thresh, nms_kind, 0); // https://github.com/Zzh-tju/DIoU-darknet

    // copy the objects to a struct
    objects = detectionToStruct(dets, nboxes, classes, calcTime);
    if( objects.frame_id % 10 == 0 ) {
        printObjects(objects);
    }

    free_detections(dets, nboxes);
    free_image(origImg);
    free_image(resizeImg);

}

void detector::updateGrabPtr(uint8_t *grabBuff) {
    if( grabBuff == nullptr ) {
        qDebug("grab ptr %p not set, skip", grabBuff);
        usleep(10000);
        return;
    }



#ifdef NONO
    // convert landscape to portrait mode

    // best sharpness
    // downscale
    // one red and blue pixel and 2 green pixels
    // this is about 8% less CPU time then INTERPOLATION_5_5_6

    QImage myImage2 = QImage(600, 960, QImage::QImage::Format_RGB32); // portrait mode

    for( int ii = 0; ii < 1200; ii+= 2 ) {
        for( int kk = 0; kk < 1920; kk += 2 ) {
            int index = ii * 1920 + kk;
            int red = grabBuff[index+0];
            int green = grabBuff[index+1];
            int blue = grabBuff[index+1920+1];
            green += grabBuff[index+1920+0];
            myImage2.setPixel(ii/2, 959-kk/2, red << 16 | (green/2) << 8 | blue);  // red, green and blue
        }
    }

    image origImg = QImageToImage(myImage2);
#else


    // convert landscape to portrait mode

    // best sharpness
    // downscale
    // one red and blue pixel and 2 green pixels
    // this is about 8% less CPU time then INTERPOLATION_5_5_6
    uint64_t *grabBuff64 = reinterpret_cast<uint64_t*>(grabBuff);
    image origImg = make_image(600, 960, 3); // with, height, colors

    for( int ii = 0; ii < 1200; ii+= 2 ) {
        for( int kk = 0; kk < 1920; kk += 8 ) {
            int index = ii * 1920 + kk;
            uint64_t grgrgrgr = grabBuff64[index/8]; // current line contain green (upper byte), ... and read (lower byte)
            uint64_t bgbgbgbg = grabBuff64[(index+1920)/8]; // next line contain blue (upper byte), ... and green (lower byte)

            uint32_t red0 = grgrgrgr & 0x00000000000000ff;
            uint32_t green0 = ( ( (grgrgrgr & 0x000000000000ff00) >> 8 ) + (bgbgbgbg & 0x00000000000000ff) ) / 2; // average over 2 pixels
            uint32_t blue0 = (bgbgbgbg & 0x000000000000ff00) >> 8;
            uint32_t red1 = (grgrgrgr & 0x0000000000ff0000) >> 16;
            uint32_t green1 = ( ( (grgrgrgr & 0x00000000ff000000) >> 24 ) + ( (bgbgbgbg & 0x0000000000ff0000) >> 16 ) ) / 2; // average over 2 pixels
            uint32_t blue1 = (bgbgbgbg & 0x00000000ff000000) >> 24;
            uint32_t red2 = (grgrgrgr & 0x000000ff00000000) >> 32;
            uint32_t green2 = ( ( (grgrgrgr & 0x0000ff0000000000) >> 40 ) + ( (bgbgbgbg & 0x000000ff00000000) >> 32 ) ) / 2; // average over 2 pixels
            uint32_t blue2 = (bgbgbgbg & 0x0000ff0000000000) >> 40;
            uint32_t red3 = (grgrgrgr & 0x00ff000000000000) >> 48;
            uint32_t green3 = ( ( (grgrgrgr & 0xff00000000000000) >> 56 ) + ( (bgbgbgbg & 0x00ff000000000000) >> 48 ) ) / 2; // average over 2 pixels
            uint32_t blue3 = (bgbgbgbg & 0xff00000000000000) >> 56;

            //myImage2.setPixel(ii/2, 959-kk/2, red0 << 16 | green0 << 8 | blue0);  // red, green and blue
            //myImage2.setPixel(ii/2, 959-(kk+2)/2, red1 << 16 | green1 << 8 | blue1);  // red, green and blue
            //myImage2.setPixel(ii/2, 959-(kk+4)/2, red2 << 16 | green2 << 8 | blue2);  // red, green and blue
            //myImage2.setPixel(ii/2, 959-(kk+6)/2, red3 << 16 | green3 << 8 | blue3);  // red, green and blue

            origImg.data[0*600*960 + (ii/2) + (959-(kk+0)/2)*600] = red0 / 255.0f;
            origImg.data[1*600*960 + (ii/2) + (959-(kk+0)/2)*600] = green0 / 255.0f;
            origImg.data[2*600*960 + (ii/2) + (959-(kk+0)/2)*600] = blue0 / 255.0f;

            origImg.data[0*600*960 + (ii/2) + (959-(kk+2)/2)*600] = red1 / 255.0f;
            origImg.data[1*600*960 + (ii/2) + (959-(kk+2)/2)*600] = green1 / 255.0f;
            origImg.data[2*600*960 + (ii/2) + (959-(kk+2)/2)*600] = blue1 / 255.0f;

            origImg.data[0*600*960 + (ii/2) + (959-(kk+4)/2)*600] = red2 / 255.0f;
            origImg.data[1*600*960 + (ii/2) + (959-(kk+4)/2)*600] = green2 / 255.0f;
            origImg.data[2*600*960 + (ii/2) + (959-(kk+4)/2)*600] = blue2 / 255.0f;

            origImg.data[0*600*960 + (ii/2) + (959-(kk+6)/2)*600] = red3 / 255.0f;
            origImg.data[1*600*960 + (ii/2) + (959-(kk+6)/2)*600] = green3 / 255.0f;
            origImg.data[2*600*960 + (ii/2) + (959-(kk+6)/2)*600] = blue3 / 255.0f;

        }
    }

#ifdef NONO
    int ww = 600;
    int hh = 960;
    int cc = 3;
    image im = make_image(600, 960, 3);
    for (int yy = 0; yy < 960; ++yy) {
        for (int kk = 0; kk < cc; ++kk) {
            for (int xx = 0; xx < 600; ++xx) {
                QRgb rgb = input.pixel(xx,yy);
                int color;
                if( kk == 0 ) {
                    color = (rgb >> 16 ) & 0xff; // red
                } else if( kk == 1 ) {
                    color = (rgb >> 8 ) & 0xff; // green
                } else {
                    color = (rgb >> 0 ) & 0xff; // blue
                }
                im.data[kk*ww*hh + yy*ww + xx] = color / 255.0f;
            }
        }
    }

#endif


#endif

    image resizeImg = resize_image2(grabBuff, origImg, net->w, net->h); // resize to 416x416

    // perform the object detection
    double timeStart = get_time_point();
    network_predict_ptr(net, (float *)resizeImg.data);
    double calcTime = get_time_point() - timeStart;

    // create bounding boxes (with relative size) for the detected objects
    int nboxes = 0;
    // get_network_boxes(net, width, height, threshold, hier_threshold, map, relative, num_boxes, letter);
    // NOTE: hier_threshold not used, custom_get_region_detections
    detection *dets = get_network_boxes(net, 1, 1, thresh, 0, 0, 1, &nboxes, 0);

    // one detected box can be classified for multiple classes, find the best class for each box
    diounms_sort(dets, nboxes, classes, nms_thresh, nms_kind, 0); // https://github.com/Zzh-tju/DIoU-darknet

    // copy the objects to a struct
    objects = detectionToStruct(dets, nboxes, classes, calcTime);
    if( objects.frame_id % 10 == 0 ) {
        printObjects(objects);
    }

    free_detections(dets, nboxes);
    free_image(origImg);
    free_image(resizeImg);
}


void detector::updateImage(image origImg ) {
    image resizeImg = resize_image(origImg, net->w, net->h); // resize to 416x416

    // perform the object detection
    double timeStart = get_time_point();
    network_predict_ptr(net, (float *)resizeImg.data);
    double calcTime = get_time_point() - timeStart;

    // create bounding boxes (with relative size) for the detected objects
    int nboxes = 0;
    // get_network_boxes(net, width, height, threshold, hier_threshold, map, relative, num_boxes, letter);
    // NOTE: hier_threshold not used, custom_get_region_detections
    detection *dets = get_network_boxes(net, 1, 1, thresh, 0, 0, 1, &nboxes, 0);

    // one detected box can be classified for multiple classes, find the best class for each box
    diounms_sort(dets, nboxes, classes, nms_thresh, nms_kind, 0); // https://github.com/Zzh-tju/DIoU-darknet

    // copy the objects to a struct
    objects = detectionToStruct(dets, nboxes, classes, calcTime);
    if( objects.frame_id % 10 == 0 ) {
        printObjects(objects);
    }

    free_detections(dets, nboxes);
    free_image(resizeImg);
}




// Convert Qt image type to darknet image type
// darknet image type:
// red line
// green line
// blue line
// ..
// red line
// green line
// blue line
image detector::QImageToImage(const QImage input) {
    int ww = input.width();
    int hh = input.height();
    int cc = 3;
    image im = make_image(ww, hh, cc);
    for (int yy = 0; yy < hh; ++yy) {
        for (int kk = 0; kk < cc; ++kk) {
            for (int xx = 0; xx < ww; ++xx) {
                QRgb rgb = input.pixel(xx,yy);
                int color;
                if( kk == 0 ) {
                    color = (rgb >> 16 ) & 0xff; // red
                } else if( kk == 1 ) {
                    color = (rgb >> 8 ) & 0xff; // green
                } else {
                    color = (rgb >> 0 ) & 0xff; // blue
                }
                im.data[kk*ww*hh + yy*ww + xx] = color / 255.0f;
            }
        }
    }
    return im;
}

frame_t detector::detectionToStruct(detection *dets, int nboxes, int classes, double calcTime) {
    frame_t objects;

    static uint32_t frame_id = 0;

    objects.frame_id = frame_id++;
    objects.calcTime = float(calcTime / 1000000.0);

    for (int ii = 0; ii < nboxes; ++ii) {
        for (int jj = 0; jj < classes; ++jj) {
            if (dets[ii].prob[jj] > 0.005) { // function get_network_boxes() has already filtered dets by actual threshold
                object_t obj;
                obj.class_id = (uint32_t)jj;
                obj.x = dets[ii].bbox.x;
                obj.y = dets[ii].bbox.y;
                obj.w = dets[ii].bbox.w;
                obj.h = dets[ii].bbox.h;
                obj.confidence = dets[ii].prob[jj];
                objects.obj.push_back(obj);
            }
        }
    }
    return objects;
}

void detector::printObjects(const frame_t objects) {
    for( size_t ii = 0; ii < objects.obj.size(); ii++ ) {
        object_t obj = objects.obj[ii];
        if( ii == 0 ) {
            printf("%8u %3.0f c %1u x %4.2f y %4.2f w %4.2f h %4.2f conf %3.0f%%\n",
                   objects.frame_id, objects.calcTime * 1000.0, obj.class_id, obj.x, obj.y, obj.w, obj.h, 100.0*obj.confidence);
        } else {
            printf("             c %1u x %4.2f y %4.2f w %4.2f h %4.2f conf %3.0f%%\n",
                   obj.class_id, obj.x, obj.y, obj.w, obj.h, 100.0*obj.confidence);

        }
    }
}


image detector::resize_image2(uint8_t *grabBuff, image im, int w, int h) {
    if( w == 0 || *grabBuff == 3 || h == 0 ) {
        qDebug("blaat");
    }
// w = 600
// h = 960
    image resized = make_image(416, 416, 3); // x and y scaled
    image part = make_image(416, 960, 3); // only x slaled
    int r, c, k;
    float w_scale = (float) 599 / 415;
    float h_scale = (float) 959 / 415;
    // scale in x direction
    for(k = 0; k < 3; ++k){ // color
        for(r = 0; r < 960; ++r){ // height: y
            for(c = 0; c < 416; ++c){ // width: x
                float val = 0;
                if(c == 415){ // last on colum
                    val = get_pixel2(im, 599, r, k); // use last pixel on line, TODO: also use new function for last colum
                } else {
                    float sx = c*w_scale; // scaled pixel on line
                    int ix = (int) sx; // round down ?
                    float dx = sx - ix; // error
                    // use an amount of the nearbly left pixel and amount of nearby right pixel
                    val = (1 - dx) * get_pixel2(im, ix, r, k) + dx * get_pixel2(im, ix+1, r, k);
                    // TODO: this oneval = (1 - dx) * get_pixel3(grabBuff, ix, r, k) + dx * get_pixel3(grabBuff, ix+1, r, k);
                }
                set_pixel2(part, c, r, k, val);
            }
        }
    }
    // scale in y direction
    for(k = 0; k < 3; ++k){ // color
        for(r = 0; r < 416; ++r){ // height: y
            float sy = r*h_scale;
            int iy = (int) sy;
            float dy = sy - iy;
            for(c = 0; c < 416; ++c){ // width: x
                float val = (1-dy) * get_pixel2(part, c, iy, k);
                set_pixel2(resized, c, r, k, val);
            }
            if(r == 415 ) continue;
            for(c = 0; c < 416; ++c){
                float val = dy * get_pixel2(part, c, iy+1, k);
                add_pixel2(resized, c, r, k, val);
            }
        }
    }

    free_image(part);
    return resized;
}


// x range [0:599], y range [0:959]
float detector::get_pixel3(uint8_t *grabBuff, int x, int y, int c) {
    // memory
    // green red line
    // blue green line
    // there are 1200 lines
    // with each 1920 pixel per line
    // multiply y by 2
    // multiply x by 2
    // swap x and y because camera rotated
    int xx = 2 * y; // range [0:1918]
    int yy = 1198 - (2 * x); // range [0:1198]

    uint8_t value;
    if( c == 0 ) {
        value = grabBuff[yy*1920+xx+0]; // red
    } else if( c == 1 ) {
        value = grabBuff[yy*1920+xx+1]; // green // TODO add pixel next line
   } else {
        value = grabBuff[yy*1920+1920+xx+1]; // blue
    }

    return value / 255.0f;
}


//    assert(x < m.w && y < m.h && c < m.c);
//    return m.data[c*m.h*m.w + y*m.w + x];

//        for( int ii = 0; ii < 1200; ii+= 2 ) {
//            for( int kk = 0; kk < 1920; kk += 2 ) {
//                int index = ii * 1920 + kk;
//                int red = grabBuff[index+0];
//                int green = grabBuff[index+1];
//                int blue = grabBuff[index+1920+1];
//                green += grabBuff[index+1920+0];
//                myImage2.setPixel(ii/2, 959-kk/2, red << 16 | (green/2) << 8 | blue);  // red, green and blue
//            }
//        }
//    }



float detector::get_pixel2(image m, int x, int y, int c) {
    assert(x < m.w && y < m.h && c < m.c);
    return m.data[c*m.h*m.w + y*m.w + x];
}

void detector::set_pixel2(image m, int x, int y, int c, float val) {
    if (x < 0 || y < 0 || c < 0 || x >= m.w || y >= m.h || c >= m.c) return;
    assert(x < m.w && y < m.h && c < m.c);
    m.data[c*m.h*m.w + y*m.w + x] = val;
}


void detector::add_pixel2(image m, int x, int y, int c, float val) {
    assert(x < m.w && y < m.h && c < m.c);
    m.data[c*m.h*m.w + y*m.w + x] += val;
}
