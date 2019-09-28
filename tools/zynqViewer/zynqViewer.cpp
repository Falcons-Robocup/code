// Copyright 2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h> // getopt

using namespace cv;
using namespace std;

// input dimension
#define WIDTH 1280
#define HEIGHT 720

class zynqViewer {
private:
	FILE *readBin, *writeBin; // handles to binary files
	string inputName, outputName, cropName;
	Mat inputFrame, outputFrame, cropFrame, roiFrame;
	bool verbose;

public:
	zynqViewer(string inputName, string outputName, string cropName, bool verbose);
	void binaryToJpg();
	void binaryToBinary();
	void jpgToBinary();
	void view();
};

// read from file and write to inputFrame
zynqViewer::zynqViewer(string inputName, string outputName, string cropName, bool verbose) {
	this->inputName = inputName;
	this->outputName = outputName;
	this->cropName = cropName;
	this->verbose = verbose;
}

void zynqViewer::binaryToJpg() {
	if (verbose) {
		printf("INFO    : using file name %s\n", inputName.c_str());
	}
	if ((readBin = fopen(inputName.c_str(), "rb")) == NULL) {
		printf("ERROR   : cannot read file %s, message %s\n", inputName.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	inputFrame = Mat::zeros(HEIGHT, WIDTH, CV_8UC3); // y, x, depth

	uint8_t data, value, saturation, hue;
	size_t numBytes = 0;
	size_t colorIndex = 0;
	size_t x = 0, y = 0;
	size_t pixels = 0;
	while (!feof(readBin)) {
		fread(&data, sizeof(data), 1, readBin);
		if (colorIndex == 0) {
			value = data;
			colorIndex = 1;
		} else if (colorIndex == 1) {
			saturation = data;
			colorIndex = 2;
		} else if (colorIndex == 2) {
			hue = data;
			colorIndex = 3;
		} else if (colorIndex == 3) {
			pixels++;
			// the 4th byte is used to decode color clasification (e.g. white (line) or yellow (ball)
			// write the first 3 bytes (hue saturation and value to frame)
			line(inputFrame, Point(x, y), Point(x, y), Scalar(hue, saturation, value), 1);
			colorIndex = 0; // start over for the next byte
			x++;
			if (x == WIDTH) {
				x = 0;
				y++;
			}
		}
		numBytes++;
	}

	if (verbose) {
		printf("INFO    : bytes %zu, pixels %zu, next x pixel %zu, next y pixel %zu\n", numBytes, pixels, x, y);
	}

	fclose(readBin);

	// color conversion, crop and write to file
	// cvtColor(inputFrame, outputFrame, COLOR_HSV2BGR); // convert to BGR
	outputFrame = inputFrame;

	vector<int> jpgCompressionParams;

	jpgCompressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
	jpgCompressionParams.push_back(90); // compression factor

	// imwrite("zynqGrabFull.jpg", outputFrame, jpgCompressionParams);

	Rect roi;
	roi.x = 0;
	roi.y = 0;
	roi.width = 820;
	roi.height = 616;
	roiFrame = outputFrame(roi);
	imwrite(outputName.c_str(), roiFrame, jpgCompressionParams);
	if (verbose) {
		printf("INFO    : region of interest width %u, height %u, pixels %u\n", roi.width, roi.height,
				roi.width * roi.height);
	}

	roi.width = 370;
	roi.height = 616;
	cropFrame = outputFrame(roi);
	imwrite(cropName.c_str(), cropFrame, jpgCompressionParams);
}


void zynqViewer::binaryToBinary() {
	if (verbose) {
		printf("INFO    : using file name %s\n", inputName.c_str());
	}
	if ((readBin = fopen(inputName.c_str(), "rb")) == NULL) {
		printf("ERROR   : cannot read file %s, message %s\n", inputName.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	inputFrame = Mat::zeros(HEIGHT, WIDTH, CV_8UC3); // y, x, depth

	uint8_t data, value, saturation, hue;
	size_t numBytes = 0;
	size_t colorIndex = 0;
	size_t x = 0, y = 0;
	size_t pixels = 0;
	while (!feof(readBin)) {
		fread(&data, sizeof(data), 1, readBin);
		if (colorIndex == 0) {
			value = data;
			colorIndex = 1;
		} else if (colorIndex == 1) {
			saturation = data;
			colorIndex = 2;
		} else if (colorIndex == 2) {
			hue = data;
			colorIndex = 3;
		} else if (colorIndex == 3) {
			pixels++;
			// the 4th byte is used to decode color classification (e.g. white (line) or yellow (ball)
			// write the first 3 bytes (hue saturation and value to frame)
			line(inputFrame, Point(x, y), Point(x, y), Scalar(hue, saturation, value), 1);
			colorIndex = 0; // start over for the next byte
			x++;
			if (x == WIDTH) {
				x = 0;
				y++;
			}
		}
		numBytes++;
	}

	if (verbose) {
		printf("INFO    : bytes %zu, pixels %zu, next x pixel %zu, next y pixel %zu\n", numBytes, pixels, x, y);
	}

	fclose(readBin);

	// color conversion, the Binary input file is in HSV, but the camera expects BGR
	cvtColor(inputFrame, outputFrame, COLOR_HSV2BGR); // convert to BGR

	// store back to file
	if ((writeBin = fopen(outputName.c_str(), "wb")) == NULL) {
		printf("ERROR   : cannot write file %s, message %s\n", outputName.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	// from zynqGrabConfig.cpp
	float greenMult = 88;
	float blueMult = 190;

	for (size_t yy = 0; yy < HEIGHT; yy++) {
		for (size_t xx = 0; xx < WIDTH; xx++) {
			Vec3b pixel = outputFrame.at<Vec3b>(yy, xx);
			uint8_t data = pixel.val[2]; // red
			fwrite(&data, sizeof(data), 1, writeBin);

			float green = 256.0 * pixel.val[1] / (256.0 + greenMult - 127.0 );
			if( green > 255 ) { green = 255; }
			data = (uint8_t) green;
			fwrite(&data, sizeof(data), 1, writeBin);

			float blue = 256.0 * pixel.val[0] / (256.0 + blueMult - 127.0 );
			if( blue > 255 ) { blue = 255; }
			data = (uint8_t) blue;
			fwrite(&data, sizeof(data), 1, writeBin);

			data = 0; // filler
			fwrite(&data, sizeof(data), 1, writeBin);
		}
	}

	fclose(writeBin);
}


void zynqViewer::jpgToBinary() {
	if (verbose) {
		printf("INFO    : input file name %s, ouput file name %s\n", inputName.c_str(), outputName.c_str());
	}
	inputFrame = imread(inputName.c_str(), CV_LOAD_IMAGE_COLOR);
	if (!inputFrame.data) {
		printf("ERROR   : cannot read file %s, message %s\n", inputName.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	// create output frame
	outputFrame = Mat::zeros(HEIGHT, WIDTH, CV_8UC3); // y, x, depth

	// copy the input frame into the (larger) output frame)
	// the zynqberry expects an image of 1280x720 instead of 820x616
	inputFrame.copyTo(outputFrame.colRange(0, inputFrame.cols).rowRange(0, inputFrame.rows));

	// binary file is in HSV format
	// update: supply the input file as /dev/fb1, which is RGB format instead of HSV
	cvtColor(outputFrame, outputFrame, COLOR_BGR2HSV);

	// store back to file
	if ((writeBin = fopen(outputName.c_str(), "wb")) == NULL) {
		printf("ERROR   : cannot write file %s, message %s\n", outputName.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	// from zynqGrabConfig.cpp
	float greenMult = 105;
	float blueMult = 185;

	for (size_t yy = 0; yy < HEIGHT; yy++) {
		for (size_t xx = 0; xx < WIDTH; xx++) {
			Vec3b pixel = outputFrame.at<Vec3b>(yy, xx);
			uint8_t data = pixel.val[2]; // red
			fwrite(&data, sizeof(data), 1, writeBin);

			float green = 256.0 * pixel.val[1] / (256.0 + greenMult - 127.0 );
			if( green > 255 ) { green = 255; }
			data = (uint8_t) green;
			fwrite(&data, sizeof(data), 1, writeBin);

			float blue = 256.0 * pixel.val[0] / (256.0 + blueMult - 127.0 );
			if( blue > 255 ) { blue = 255; }
			data = (uint8_t) blue;
			fwrite(&data, sizeof(data), 1, writeBin);

			data = 0; // filler
			fwrite(&data, sizeof(data), 1, writeBin);
		}
	}

	fclose(writeBin);

}

void zynqViewer::view() {
	imshow("q to quit", roiFrame);

	usleep(10000);
}

int main(int argc, char** argv) {
	string inputName = "zynqGrab.bin";
	string outputName = "zynqGrab.jpg";
	string cropName = "zynqGrabCrop.jpg";
	bool reverse = false;
	bool verbose = false;
	bool binFile = false;
	int opt = 0;
	while ((opt = getopt(argc, argv, "bc:f:ho:rv")) != -1) {
		switch (opt) {
		case 'b':
			binFile = true;
			break;
		case 'c':
			cropName.assign(optarg);
			break;
		case 'f':
			inputName.assign(optarg);
			break;
		case 'h':
			printf("INFO      : <-f inputFileName.bin> <-o outputFileName.jpg>\n");
			printf("          : <-r -f inputFileName.jpg> <-o outputFileName.bin>\n");
			printf("          : <-b -r -f inputFileName.bin> <-o outputFileName.bin>\n");
			break;
		case 'o':
			outputName.assign(optarg);
			break;
		case 'r':
			reverse = true;
			break;
		case 'v':
			verbose = true;
			break;
		}
	}

	zynqViewer *rmt = new zynqViewer(inputName, outputName, cropName, verbose);

	if (reverse) {
		if( binFile ) {
			rmt->binaryToBinary();
		} else {
			rmt->jpgToBinary();
		}
	} else {
		rmt->binaryToJpg();
	}

#ifdef NONO
	bool busy = true;
	while (busy) {
		rmt->view();
		int key = waitKey(20);
		// if( key >= 0 ) { printf( "key: %d\n", key );  }
		switch (key) {
			case 1048603: // escape
			case 1179729:// caps lock q
			case 1048689:// q
			case 27:// escape
			case 'q':
			busy = false;
			break;
			default:
			break;
		}
	}
#endif

	return 0;
}

