// Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// convert raw rgb 8 bit image to bmp

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "raspiDefaults.hpp" // for height and width

#define ROI_PIXELS (ROI_HEIGHT * ROI_WIDTH)

#pragma pack(push,1)
typedef struct {
	uint8_t signature[2];
	uint32_t filesize;
	uint32_t reserved;
	uint32_t fileoffset_to_pixelarray;
} fileHeaderSt;

typedef struct {
	uint32_t dibheadersize;
	uint32_t width;
	uint32_t height;
	uint16_t planes;
	uint16_t bitsperpixel;
	uint32_t compression;
	uint32_t imagesize;
	uint32_t ypixelpermeter;
	uint32_t xpixelpermeter;
	uint32_t numcolorspallette;
	uint32_t mostimpcolor;
} bitmapInfoHeaderSt;

typedef struct {
	fileHeaderSt fileheader;
	bitmapInfoHeaderSt bitmapinfoheader;
} bitmapSt;
#pragma pack(pop)

void rgbToBmp(size_t camIndex, char *inputFileName, char *outputFileName, bool reduce, bool verbose) {
	if (verbose) {
		printf("INFO      : cam %zu system image width %d height %d\n", camIndex, ROI_WIDTH, ROI_HEIGHT);
		printf("INFO      : cam %zu system image input file name %s\n", camIndex, inputFileName);
		printf("INFO      : cam %zu system image output file name %s\n", camIndex, outputFileName);
	}

	// determine input file size
	struct stat st;
	stat(inputFileName, &st);
	size_t fileSize = st.st_size;
	if (verbose) {
		printf("INFO      : cam %zu system input image file size %.1fkB\n", camIndex, fileSize / 1000.0);
	}

	// verify if file matches provided dimensions
	size_t size = ROI_WIDTH * ROI_HEIGHT * 3;
	if (fileSize != size) {
		printf("ERROR     : cam %zu system image file size is %zu bytes, but expected %zu bytes\n", camIndex, fileSize, size);
	}

	// open the file
	FILE *readBin = NULL;
	if ((readBin = fopen(inputFileName, "rb")) == NULL) {
		printf("ERROR     : cam %zu system cannot read file %s, message %s\n", camIndex, inputFileName, strerror(errno));
		exit(EXIT_FAILURE);
	}

#ifdef NONO
	// alternative way to determine file size
	fseek(readBin, 0, SEEK_END);
	fileSize = (size_t)ftell(readBin);
	printf("INFO      : cam %zu system input image file size %.1fkB\n", camIndex, fileSize/1000.0 );
	rewind(readBin);
#endif

	// allocate the memory for the input image (input file)
	uint8_t *bufferRgb;
	bufferRgb = (uint8_t *) malloc(fileSize);
	if (bufferRgb == NULL) {
		printf("ERROR     : cam %zu system cannot allocate %zu bytes for input image, message %s\n", camIndex, fileSize, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (verbose) {
		printf("INFO      : cam %zu system first buffer pointer %p last buffer pointer %p size %zu bytes\n", camIndex, bufferRgb,
				&bufferRgb[fileSize - 1], &bufferRgb[fileSize - 1] - bufferRgb + 1);
	}

	// copy the file into the buffer
	size_t nRead = fread(bufferRgb, 1, fileSize, readBin);
	if (nRead != fileSize) {
		printf("ERROR     : cam %zu system only %zu bytes of %zu have been read from the file, message %s\n", camIndex, nRead, fileSize,
				strerror(errno));
		exit(EXIT_FAILURE);
	}
	fclose(readBin);

	// convert rgb to bgr and if needed scale down to 1/4 of the original
	uint8_t *bufferBgr;
	if (reduce) {
		bufferBgr = (uint8_t *) malloc(fileSize / 4);
	} else {
		bufferBgr = (uint8_t *) malloc(fileSize);
	}

	// bmp format begins with bottom scan line instead of top scan line (ROI_PIXELS are from left to right)
	if (reduce) {
		size_t outPixel = 0;
		for (int row = ROI_HEIGHT - 2; row >= 0; row -= 2) { // rgb rows 606 to 0 (bottom to top)
			int rgbOffset = row * ROI_WIDTH;

			for (int collum = 0; collum < ROI_WIDTH - 1; collum += 2) { // left to right
				int inPixel = rgbOffset + collum;
				if ((inPixel < 0) || (inPixel >= ROI_PIXELS)) {
					printf("ERROR     : cam %zu system input pixel %d out of range [0 to %u]\n", camIndex, inPixel, ROI_PIXELS);
					exit(EXIT_FAILURE);
				}

				if (outPixel >= ROI_PIXELS / 4) {
					printf("ERROR     : cam %zu system  output pixel %zu out of range [0 to %u]\n", camIndex, outPixel, ROI_PIXELS / 4);
					exit(EXIT_FAILURE);
				}

				// bmp rows 0 to 303 (bottom to top)
				// swap red and blue
				bufferBgr[3 * outPixel + 2] = bufferRgb[3 * inPixel + 0];
				bufferBgr[3 * outPixel + 1] = bufferRgb[3 * inPixel + 1];
				bufferBgr[3 * outPixel + 0] = bufferRgb[3 * inPixel + 2];
				outPixel++;
			}
		}
	} else {
		size_t outPixel = 0;
		for (int row = ROI_HEIGHT - 1; row >= 0; row--) { // rgb rows 607 to 0 (bottom to top)
			int rgbOffset = row * ROI_WIDTH;

			for (int collum = 0; collum < ROI_WIDTH; collum++) { // left to right

				int inPixel = rgbOffset + collum;
				if ((inPixel < 0) || (inPixel >= ROI_PIXELS)) {
					printf("ERROR     : cam %zu system input pixel %d out of range [0 to %u]\n", camIndex, inPixel, ROI_PIXELS);
					exit(EXIT_FAILURE);
				}

				if (outPixel >= ROI_PIXELS) {
					printf("ERROR     : cam %zu system output pixel %zu out of range [0 to %u]\n", camIndex, outPixel, ROI_PIXELS);
					exit(EXIT_FAILURE);
				}

				// bmp rows 0 to 607 (bottom to top)
				// swap red and blue
				bufferBgr[3 * outPixel + 2] = bufferRgb[3 * inPixel + 0];
				bufferBgr[3 * outPixel + 1] = bufferRgb[3 * inPixel + 1];
				bufferBgr[3 * outPixel + 0] = bufferRgb[3 * inPixel + 2];
				outPixel++;
			}
		}
	}

// free the buffer with rgb input image
	free(bufferRgb);

	size_t outHeight = ROI_HEIGHT;
	size_t outWidth = ROI_WIDTH;
	if (reduce) {
		outHeight /= 2;
		outWidth /= 2;
	}
	size_t outSize = outHeight * outWidth * 3;

// create bmp header
	bitmapSt *pbitmap = (bitmapSt*) calloc(1, sizeof(bitmapSt));
	pbitmap->fileheader.signature[0] = 0x42; // BM
	pbitmap->fileheader.signature[1] = 0x4d;
	pbitmap->fileheader.filesize = outSize + sizeof(bitmapSt);
	pbitmap->fileheader.fileoffset_to_pixelarray = sizeof(bitmapSt);
	pbitmap->bitmapinfoheader.dibheadersize = sizeof(bitmapInfoHeaderSt);
	pbitmap->bitmapinfoheader.width = outWidth;
	pbitmap->bitmapinfoheader.height = outHeight;
	pbitmap->bitmapinfoheader.planes = 1;
	pbitmap->bitmapinfoheader.bitsperpixel = 24; // 3 colors of 8 bits
	pbitmap->bitmapinfoheader.compression = 0;
	pbitmap->bitmapinfoheader.imagesize = outSize;
	pbitmap->bitmapinfoheader.ypixelpermeter = 0x130b; // 72 dpi
	pbitmap->bitmapinfoheader.xpixelpermeter = 0x130b; // 72 dpi
	pbitmap->bitmapinfoheader.numcolorspallette = 0;

// open the output file
	FILE *outfile = NULL;
	if ((outfile = fopen(outputFileName, "wb")) == NULL) {
		printf("ERROR     : cam %zu system cannot open output file %s, message %s\n", camIndex, outputFileName, strerror(errno));
		exit(EXIT_FAILURE);
	}

// write bmp header to file
	fwrite(pbitmap, 1, sizeof(bitmapSt), outfile);
	free(pbitmap);

// write the bgr image to file
	fwrite(bufferBgr, 1, size, outfile);

// free the buffer with bgr image
	free(bufferBgr);

// the writing has finished, close the output file
	fclose(outfile);
}
