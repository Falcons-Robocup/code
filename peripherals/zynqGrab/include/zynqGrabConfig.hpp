// Copyright 2017 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef ZYNQ_GRAB_CONFIG_HPP
#define ZYNQ_GRAB_CONFIG_HPP

#include <cstdint>

// resolution is 820x616, but 820 does not divide by 8
#define ROI_WIDTH 824
#define ROI_HEIGHT 616

#define SEND_IMAGE_WIDTH_FACTOR 4
#define SEND_IMAGE_HEIGHT_FACTOR 4

#define SEND_IMAGE_WIDTH (ROI_WIDTH/SEND_IMAGE_WIDTH_FACTOR)
#define SEND_IMAGE_HEIGHT (ROI_HEIGHT/SEND_IMAGE_HEIGHT_FACTOR)


// maximal packet size 3*95*77 = 21945 bytes
typedef struct {
	uint8_t id;
	uint8_t cnt;
	uint16_t size; // packet size including header in bytes
	union {
		// 2^16 = 64 KiB
		uint8_t u8[1 << 16]; // unsigned payload
		uint16_t u16[1 << 15];
		uint32_t u32[1 << 14];
		uint64_t u64[1 << 13];
		int8_t s8[1 << 16]; // signed payload
		int16_t s16[1 << 15];
		int32_t s32[1 << 14];
		int64_t s64[1 << 13];
		float f32[1 << 14];
		double d64[1 << 13];
	} pl;
}__attribute__((packed)) packetT;

#define HEADER_SIZE 4

// #define MAX_PAYLOAD_SIZE ( 8192 - HEADER_SIZE )
#define MAX_PAYLOAD_SIZE ( 16384 - HEADER_SIZE )

class zynqGrabConfig {
private:
	bool cameraReset;
	uint8_t lineValMin;
	uint8_t lineSatMax;
	uint16_t lineLengthMax;
	uint16_t lineLengthMin;
	uint16_t lineSizeMin;
	uint16_t lineTransferPixelsMax;
	uint16_t lineFloorWindowSize;
	uint16_t lineFloorPixelsMin;
	uint16_t lineWindowSize;
	uint16_t linePixelsMin;
	uint8_t ballValMin;
	uint8_t ballSatMin;
	uint8_t ballHueMin;
	uint8_t ballHueMax;
	uint16_t ballWindowSize;
	uint16_t ballPixelsMin;
	uint16_t ballFalsePixelsMax;
	uint8_t floorValMin;
	uint8_t floorSatMin;
	uint8_t floorHueMin;
	uint8_t floorHueMax;
	uint8_t obstacleValMax;
	uint8_t obstacleSatMax;
	uint32_t obstacleFloorWindowSize;
	uint32_t obstacleLineWindowSize;
	uint32_t obstacleBallWindowSize;
	uint32_t obstacleWindowSize;
	uint32_t obstacleTransferPixelsMax;
	uint32_t obstacleFloorPixelsMin;
	uint32_t obstacleLinePixelsMin;
	uint32_t obstacleBallPixelsMin;
	uint32_t obstaclePixelsMin;

	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t testPattern;
	uint8_t analogGain;
	uint16_t shutter;

	uint16_t lines;
	uint16_t pixels;
	uint16_t xStart;
	uint16_t xEnd;
	uint16_t xSize;
	uint16_t yStart;
	uint16_t yEnd;
	uint16_t ySize;

	uint32_t EXCK_FREQ;
	uint32_t VTPXCK_DIV;
	uint32_t VTSYCK_DIV;
	uint32_t PREPLLCK_VT_DIV;
	uint32_t PREPLLCK_OP_DIV;
	uint32_t PLL_VT_MPY;
	uint32_t OPPXCK_DIV;
	uint32_t OPSYCK_DIV;
	uint32_t PLL_OP_MPY;

public:
	// ## setup ##
	zynqGrabConfig(); // constructor
	void setCameraReset( ) { cameraReset = true; }
	uint32_t getCameraReset ( ) {
		if( cameraReset ) {
			cameraReset = false;
			return true;
		} else {
			return false;
		}
	}

	void setLineValMin( uint32_t value ) { lineValMin = value; }
	uint32_t getLineValMin ( ) { return lineValMin; }
	void setLineSatMax( uint32_t value ) { lineSatMax = value; }
	uint32_t getLineSatMax ( ) { return lineSatMax; }
	void setLineLengthMax( uint32_t value ) { lineLengthMax = value; }
	uint32_t getLineLengthMax ( ) { return lineLengthMax; }
	void setLineTransferPixelsMax( uint32_t value ) { lineTransferPixelsMax = value; }
	uint32_t getLineTransferPixelsMax ( ) { return lineTransferPixelsMax; }
	void setLineFloorWindowSize( uint32_t value ) { lineFloorWindowSize = value; }
	uint32_t getLineFloorWindowSize ( ) { return lineFloorWindowSize; }
	void setLineFloorPixelsMin( uint32_t value ) { lineFloorPixelsMin = value; }
	uint32_t getLineFloorPixelsMin ( ) { return lineFloorPixelsMin; }
	void setLineWindowSize( uint32_t value ) { lineWindowSize = value; }
	uint32_t getLineWindowSize ( ) { return lineWindowSize; }
	void setLinePixelsMin( uint32_t value ) { linePixelsMin = value; }
	uint32_t getLinePixelsMin ( ) { return linePixelsMin; }

	void setBallValMin( uint32_t value ) { ballValMin = value; }
	uint32_t getBallValMin ( ) { return ballValMin; }
	void setBallSatMin( uint32_t value ) { ballSatMin = value; }
	uint32_t getBallSatMin ( ) { return ballSatMin; }
	void setBallHueMin( uint32_t value ) { ballHueMin = value; }
	uint32_t getBallHueMin ( ) { return ballHueMin; }
	void setBallHueMax( uint32_t value ) { ballHueMax = value; }
	uint32_t getBallHueMax ( ) { return ballHueMax; }
	void setBallWindowSize( uint32_t value ) { ballWindowSize = value; }
	uint32_t getBallWindowSize ( ) { return ballWindowSize; }
	void setBallPixelsMin( uint32_t value ) { ballPixelsMin = value; }
	uint32_t getBallPixelsMin ( ) { return ballPixelsMin; }
	void setBallFalsePixelsMax( uint32_t value ) { ballFalsePixelsMax = value; }
	uint32_t getBallFalsePixelsMax ( ) { return ballFalsePixelsMax; }

	void setFloorValMin( uint32_t value ) { floorValMin = value; }
	uint32_t getFloorValMin ( ) { return floorValMin; }
	void setFloorSatMin( uint32_t value ) { floorSatMin = value; }
	uint32_t getFloorSatMin ( ) { return floorSatMin; }
	void setFloorHueMin( uint32_t value ) { floorHueMin = value; }
	uint32_t getFloorHueMin ( ) { return floorHueMin; }
	void setFloorHueMax( uint32_t value ) { floorHueMax = value; }
	uint32_t getFloorHueMax ( ) { return floorHueMax; }


	void setObstacleValMax( uint32_t value ) { obstacleValMax = value; }
	uint32_t getObstacleValMax ( ) { return obstacleValMax; }
	void setObstacleSatMax( uint32_t value ) { obstacleSatMax = value; }
	uint32_t getObstacleSatMax ( ) { return obstacleSatMax; }
	void setObstacleFloorWindowSize( uint32_t value ) { obstacleFloorWindowSize = value; }
	uint32_t getObstacleFloorWindowSize ( ) { return obstacleFloorWindowSize; }
	void setObstacleLineWindowSize( uint32_t value ) { obstacleLineWindowSize = value; }
	uint32_t getObstacleLineWindowSize ( ) { return obstacleLineWindowSize; }
	void setObstacleBallWindowSize( uint32_t value ) { obstacleBallWindowSize = value; }
	uint32_t getObstacleBallWindowSize ( ) { return obstacleBallWindowSize; }
	void setObstacleWindowSize( uint32_t value ) { obstacleWindowSize = value; }
	uint32_t getObstacleWindowSize ( ) { return obstacleWindowSize; }
	void setObstacleTransferPixelsMax( uint32_t value ) { obstacleTransferPixelsMax = value; }
	uint32_t getObstacleTransferPixelsMax( ) { return obstacleTransferPixelsMax; }
	void setObstacleFloorPixelsMin( uint32_t value ) { obstacleFloorPixelsMin = value; }
	uint32_t getObstacleFloorPixelsMin ( ) { return obstacleFloorPixelsMin; }
	void setObstacleLinePixelsMin( uint32_t value ) { obstacleLinePixelsMin = value; }
	uint32_t getObstacleLinePixelsMin( ) { return obstacleLinePixelsMin; }
	void setObstacleBallPixelsMin( uint32_t value ) { obstacleBallPixelsMin = value; }
	uint32_t getObstacleBallPixelsMin( ) { return obstacleBallPixelsMin; }
	void setObstaclePixelsMin( uint32_t value ) { obstaclePixelsMin = value; }
	uint32_t getObstaclePixelsMin( ) { return obstaclePixelsMin; }

	void setRed( uint32_t value ) { red = value; }
	uint32_t getRed ( ) { return red; }
	void setGreen( uint32_t value ) { green = value; }
	uint32_t getGreen ( ) { return green; }
	void setBlue( uint32_t value ) { blue = value; }
	uint32_t getBlue ( ) { return blue; }

	void setTestPattern( uint8_t value ) { testPattern = value; }
	uint8_t getTestPattern ( ) { return testPattern; }

	void setAnalogGain( uint8_t value ) { analogGain = value; }
	uint8_t getAnalogGain ( ) { return analogGain; }

	void setShutter( uint16_t value ) { shutter = value; }
	uint16_t getShutter ( ) { return shutter; }

	void setLines( uint16_t value ) { lines = value; }
	uint16_t getLines ( ) { return lines; }

	void setPixels( uint16_t value ) { pixels = value; }
	uint16_t getPixels ( ) { return pixels; }

	void setXStart( uint16_t value ) { xStart = value; }
	uint16_t getXStart ( ) { return xStart; }

	void setXEnd( uint16_t value ) { xEnd = value; }
	uint16_t getXEnd ( ) { return xEnd; }

	void setXSize( uint16_t value ) { xSize = value; }
	uint16_t getXSize ( ) { return xSize; }

	void setYStart( uint16_t value ) { yStart = value; }
	uint16_t getYStart ( ) { return yStart; }

	void setYEnd( uint16_t value ) { yEnd = value; }
	uint16_t getYEnd ( ) { return yEnd; }

	void setYSize( uint16_t value ) { ySize = value; }
	uint16_t getYSize ( ) { return ySize; }

	void setEXCK_FREQ( uint32_t value ) { EXCK_FREQ = value; }
	uint32_t getEXCK_FREQ ( ) { return EXCK_FREQ; }

	void setVTPXCK_DIV( uint32_t value ) { VTPXCK_DIV = value; }
	uint32_t getVTPXCK_DIV ( ) { return VTPXCK_DIV; }

	void setVTSYCK_DIV( uint32_t value ) { VTSYCK_DIV = value; }
	uint32_t getVTSYCK_DIV ( ) { return VTSYCK_DIV; }

	void setPREPLLCK_VT_DIV( uint32_t value ) { PREPLLCK_VT_DIV = value; }
	uint32_t getPREPLLCK_VT_DIV ( ) { return PREPLLCK_VT_DIV; }

	void setPREPLLCK_OP_DIV( uint32_t value ) { PREPLLCK_OP_DIV = value; }
	uint32_t getPREPLLCK_OP_DIV ( ) { return PREPLLCK_OP_DIV; }

	void setPLL_VT_MPY( uint32_t value ) { PLL_VT_MPY = value; }
	uint32_t getPLL_VT_MPY ( ) { return PLL_VT_MPY; }

	void setOPPXCK_DIV( uint32_t value ) { OPPXCK_DIV = value; }
	uint32_t getOPPXCK_DIV ( ) { return OPPXCK_DIV; }

	void setOPSYCK_DIV( uint32_t value ) { OPSYCK_DIV = value; }
	uint32_t getOPSYCK_DIV ( ) { return OPSYCK_DIV; }

	void setPLL_OP_MPY( uint32_t value ) { PLL_OP_MPY = value; }
	uint32_t getPLL_OP_MPY ( ) { return PLL_OP_MPY; }

};

#endif


