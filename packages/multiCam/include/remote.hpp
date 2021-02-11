// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef REMOTE_HPP
#define REMOTE_HPP

#include "configurator.hpp"
#include "multicastReceive.hpp"
#include "robotFloor.hpp"
#include <thread>


class remote
{
private:
	configurator *conf;
	multicastReceive *multRecv;
	robotFloor *rFloor;


	int width, height;
	int xCenter, yCenter;
	double remoteRatio; // used to convert robot pixels to remote viewer pixels

	cv::Mat grassFrame, allFrame;

	// create the frame for all 6 robots
	std::vector<cv::Mat> robots; // latest and greatest viewer image
	std::vector<cv::Mat> robotsHistory; // keep history for balls, cyan and magenta in viewer window
	bool flip[6];

	std::thread receiveThread;

	void drawFloorLinePoints( size_t index );
	void drawFloorRobot( size_t index );
	void drawFloorBalls( size_t index, cv::Scalar color, cv::Scalar colorDark, size_t type );
	void drawFloorObstacles( size_t index );
	void floorPrintText( size_t index, cv::Scalar color );
	void printBallPosition( size_t index, size_t type, size_t leftIndent, int line, cv::Scalar color );


public:
	remote();
	void update();
	void packetIndexPauseToggle( ) { multRecv->packetIndexPauseToggle( ); }
	void packetIndexAdd( int value );
	void startReceiveThread();
	void doFlip( size_t index ) { flip[index] = ! flip[index]; }
};

#endif
