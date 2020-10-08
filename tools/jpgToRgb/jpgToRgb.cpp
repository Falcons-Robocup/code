// Copyright 2016-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <cerrno>
#include <fcntl.h>    /* For O_RDWR */
#include <iostream>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/time.h> // gettimeofday
#include <unistd.h> // getopt

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "jpgToRgb.hpp"

#define SEND_IMAGE_WIDTH (32*25) // 800
#define SEND_IMAGE_HEIGHT (32*19) // 608

using namespace std;
using namespace cv;

jpgToRgb::jpgToRgb(std::string inputFileName, std::string outputFileName, bool verbose) {

    if (verbose) {
        printf("INFO      : convert %dx%d image %s to %s\n", SEND_IMAGE_WIDTH, SEND_IMAGE_HEIGHT, inputFileName.c_str(),
                outputFileName.c_str());
    }

    // read the jpg image
    inputFrame = imread(inputFileName.c_str(), cv::IMREAD_COLOR);

    if (!inputFrame.data) {
        printf("ERROR     : cannot load %s\n", inputFileName.c_str());
        exit(EXIT_FAILURE);
    }

    int width = inputFrame.cols;
    int height = inputFrame.rows;
    if ((width != SEND_IMAGE_WIDTH) or (height != SEND_IMAGE_HEIGHT)) {
        printf("ERROR     : expected image of %dx%d but got %dx%d\n", SEND_IMAGE_WIDTH, SEND_IMAGE_HEIGHT, width,
                height);
        exit(EXIT_FAILURE);
    }

    uint8_t cameraFrame[3 * SEND_IMAGE_WIDTH * SEND_IMAGE_HEIGHT];

    size_t bufferIndex = 0;
    for (size_t yy = 0; yy < SEND_IMAGE_HEIGHT; yy++) { // lines
        for (size_t xx = 0; xx < SEND_IMAGE_WIDTH; xx++) { // pixels
            cameraFrame[bufferIndex++] = inputFrame.at<Vec3b>(yy, xx)[2]; // red
            cameraFrame[bufferIndex++] = inputFrame.at<Vec3b>(yy, xx)[1]; // green
            cameraFrame[bufferIndex++] = inputFrame.at<Vec3b>(yy, xx)[0]; // blue
        }
    }

    FILE *writeFilePtr;
    writeFilePtr = fopen(outputFileName.c_str(), "wb"); // w for write, b for binary

    if (writeFilePtr == NULL) {
        printf("ERROR     : %s when opening file %s for writing\n", strerror(errno), outputFileName.c_str());
        exit(EXIT_FAILURE);
    }

    fwrite(cameraFrame, sizeof(cameraFrame), 1, writeFilePtr); // write the buffer to file
    fclose(writeFilePtr);
}

void jpgToRgb::display() {
    bool keepGoing = true;
    while (keepGoing) {

        imshow("input jpg", inputFrame);

        int key = waitKey(20);
        switch (key) {
        case 1048603: // escape
        case 1179729: // caps lock q
        case 1048689: // q
        case 27: // escape
        case 'q':
            keepGoing = false;
            break;
        default:
            break;
        }
    }

}

int main(int argc, char** argv) {
    string inputFileName = "image.jpg";
    string outputFileName = "image.rgb";
    bool display = false;
    bool help = false;
    bool verbose = false;
    int opt = 0;
    while ((opt = getopt(argc, argv, "dhi:o:v")) != -1) {
        switch (opt) {
        case 'd':
            display = true;
            break;
        case 'h':
            printf("INFO      : <-i inputFileName.jpg> <-o outputFileName.rgb> <-v verbose>\n");
            help = true;
            break;
        case 'i':
            inputFileName.assign(optarg);
            break;
        case 'o':
            outputFileName.assign(optarg);
            break;
        case 'v':
            verbose = true;
            break;
        } // switch
    } // while

    if (!help) {
        jpgToRgb *convert = new jpgToRgb(inputFileName, outputFileName, verbose);
        if (display) {
            convert->display();
        }
    }
    if (verbose) {
        printf("INFO      : all done\n");
    }

    return 0;
}

