// Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// convert raw rgb 8 bit image to jpg

#include <cerrno>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h> // getopt

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rgbToJpg.hpp"

#define IMAGE_WIDTH (32*25) // 800
#define IMAGE_HEIGHT (32*19) // 608

using namespace std;
using namespace cv;

rgbToJpg::rgbToJpg(std::string inputFileName, std::string outputFileName, bool verbose) {
	this->inputFileName = inputFileName;
	this->outputFileName = outputFileName;
	this->verbose = verbose;
}

void rgbToJpg::update() {
	if (verbose) {
		printf("INFO      : convert %dx%d image %s to %s\n", IMAGE_WIDTH, IMAGE_HEIGHT, inputFileName.c_str(),
				outputFileName.c_str());
	}

	FILE *readBin = NULL;
	if ((readBin = fopen(inputFileName.c_str(), "rb")) == NULL) {
		printf("ERROR     : cannot read file %s, message %s\n", inputFileName.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	// create empty input frame
	inputFrame = Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3); // y, x, depth

	uint8_t data, red, green, blue;
	size_t colorIndex = 0;
	size_t numPixels = 0;
	size_t numBytes = 0;
	int xx = 0, yy = 0;
	while (!feof(readBin)) {
		fread(&data, sizeof(data), 1, readBin);
		if (colorIndex == 0) {
			red = data;
			colorIndex = 1;
		} else if (colorIndex == 1) {
			green = data;
			colorIndex = 2;
		} else if (colorIndex == 2) {
			blue = data;
			numPixels++;
			line(inputFrame, Point(xx, yy), Point(xx, yy), Scalar(blue, green, red), 1);
			colorIndex = 0; // start over for the next byte
			xx++;
			if (xx == IMAGE_WIDTH) {
				xx = 0;
				yy++;
			}
		}
		numBytes++;
	}
	if (verbose) {
		printf("INFO      : bytes %zu, pixels %zu, next x pixel %d, next y pixel %d\n", numBytes, numPixels, xx, yy);
	}

	fclose(readBin);

	// save as jpg
	vector<int> jpgCompressionParams;
	jpgCompressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
	jpgCompressionParams.push_back(90); // compression factor

	imwrite(outputFileName.c_str(), inputFrame, jpgCompressionParams);
}

void rgbToJpg::display() {
	bool keepGoing = true;
	while (keepGoing) {

		imshow("original", inputFrame);

		// read the just stored jpg image
		Mat readBackImage = imread(outputFileName.c_str(), CV_LOAD_IMAGE_COLOR);

		imshow("compressed", readBackImage);

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
	string inputFileName = "grab.rgb";
	string outputFileName = "grab.jpg";
	bool help = false;
	bool display = true;
	bool verbose = false;

	int opt = 0;
	while ((opt = getopt(argc, argv, "dhi:o:v")) != -1) {
		switch (opt) {
		case 'd':
			display = true;
			break;
		case 'h':
			printf("INFO      : <-i inputFileName.rgb> <-o outputFileName.jpg> <-d display><-v verbose>\n");
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

	rgbToJpg *convert = new rgbToJpg(inputFileName, outputFileName, verbose);

	if (!help) {
		convert->update();

		if (display) {
			convert->display();
		}

	}

	if (verbose) {
		printf("INFO      : all done\n");
	}

	return EXIT_SUCCESS;
}

