// Copyright 2017-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "remote.hpp"

#include <unistd.h> // getopt

using namespace cv;
using namespace std;

remote::remote() {
	conf = new configurator(1);
	conf->setRemote(true); // used to generate a scaled rFloor version
	rFloor = new robotFloor(conf);
	multRecv = new multicastReceive();

	// when starting none of the robot images is mirrored
	for (size_t ii = 0; ii < 6; ii++) {
		flip[ii] = false;
	}

	width = rFloor->getWidth();
	height = rFloor->getHeight();
	remoteRatio = rFloor->getRemoteRatio();

	xCenter = (width - 1) / 2.0; // Width is odd number, -1 because index starts at 0
	yCenter = (height - 1) / 2.0; // Height is odd number, -1 because index starts at 0

	Mat lineFrame = Mat::zeros(height, width, CV_8UC3); // y, x, depth
	lineFrame = COLOR_LINE;
	grassFrame = Mat::zeros(height, width, CV_8UC3); // y, x, depth
	grassFrame = COLOR_GRASS;

	// create the grassFrame with the lines
	lineFrame.copyTo(grassFrame, (rFloor->getRefField() != 0)); // virtual field used for human interpretation

	// create the frame for all 6 robots
	for (size_t ii = 0; ii < 6; ii++) {
		robots.push_back(Mat::zeros(height, width, CV_8UC3)); // y, x, depth
		robotsHistory.push_back(Mat::zeros(height, width, CV_8UC3)); // y, x, depth
	}

	// create the frame for all 2*3 = 6 robots
	allFrame = Mat::zeros(2 * height, 3 * width, CV_8UC3); // y, x, depth

	for (size_t ii = 0; ii < robots.size(); ii++) {
		grassFrame.copyTo(robots[ii]);
		grassFrame.copyTo(robotsHistory[ii]);
	}
}

void remote::drawFloorLinePoints(size_t index) {
	ballObstListSt linePoints = multRecv->getLinePointList(index);
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);

	if ((goodEnoughLoc.state != invalid) && (linePoints.state != invalid)) {
		float xRatio = remoteRatio * goodEnoughLoc.pos.x;
		float yRatio = remoteRatio * goodEnoughLoc.pos.y;
		float rz = M_PI * goodEnoughLoc.pos.rz / 180.0;

		for (size_t ii = 0; ii < linePoints.list.size(); ii++) {
			// angle provided in radians
			float angle = linePoints.list[ii].azimuth + rz;

			// radius provided in meters
			float radiusMeter = linePoints.list[ii].radius;
			// the remoteRatio expects pixels of 2 cm instead of meters
			float radius = remoteRatio * radiusMeter / 0.02;

			// x-axis is 0 degrees on rectangular floor
			float x = xRatio + radius * cos(angle);
			float y = yRatio - radius * sin(angle);

			// because the field is symmetrical omniCam can come up with two solutions, swap if needed
			if (flip[index]) {
				x = width - x;
				y = height - y;
			}

			// draw positions outside the floor on the border
			if (x < 0.0) {
				x = 0.0;
			}
			if (x > width) {
				x = width - 1;
			}
			if (y < 0) {
				y = 0;
			}
			if (y >= height) {
				y = height - 1;
			}

			Scalar color = COLOR_LINEPOINTS;

			// change the color when the robot position is old or floor line points information is old
			if ((goodEnoughLoc.state == old) || (linePoints.state == old)) {
				color = COLOR_LINEPOINTS_OLD;
			}

			// show the line pixels in red if the location of the robot is not good enough
			if (!goodEnoughLoc.goodEnough) {
				color = COLOR_LINEPOINTS_INVALID
				;
			}

			line(robots[index], Point(round(x), round(y)), Point(round(x), round(y)), color, 2);
		}
	}
}

// show rz rotated robot at position x, y in the rectangular field viewer
void remote::drawFloorRobot(size_t index) {
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);

	if (goodEnoughLoc.state != invalid) {
		// posRos.x and y are 0 in center of screen and in meters
		// convert to remote viewer pixels
		// and add offset from center position to top left (used as 0 for remote viewer)
		float xRatio = xCenter + goodEnoughLoc.posRos.x * rFloor->getMetersToPixels(); // Width is odd number, -1 because index starts at 0
		float yRatio = yCenter + goodEnoughLoc.posRos.y * rFloor->getMetersToPixels(); // Height is odd number, -1 because index starts at 0

		float rz = goodEnoughLoc.posRos.rz;

		Scalar color = COLOR_ROBOT;

		// change the color when the robot position is old
		if (goodEnoughLoc.state == old) {
			color = COLOR_ROBOT_OLD
			;
		}

		// make robot yellow when ball possession
		int thickness = 1;
		if (multRecv->getStats(index).possessionPixels > multRecv->getPossessionThreshold()) {
			thickness = -1; // negative thickness means a filled circle is to be drawn
			color = COLOR_ROBOT_BALLPOSSESSION;
		}

		// because the field is symmetrical omniCam can come up with two solutions, swap if needed
		if (flip[index]) {
			xRatio = width - xRatio;
			yRatio = height - yRatio;
			rz = CV_PI + rz;
		}

		int circleRadius = 12; // obstacle / robot ~ 25cm radius, field 2cm = 1 pixel, obstacle ~ 12 pixels
		circle(robots[index], Point(round(xRatio), round(yRatio)), circleRadius, color, thickness);

		// rz = 0 on y-axis, anti-clockwise
		float xShooter = xRatio + 2.0f * circleRadius * sin(-rz);
		float yShooter = yRatio - 2.0f * circleRadius * cos(rz); // subtract because negative y is on top and positive y is on bottom
		line(robots[index], Point(round(xRatio), round(yRatio)), Point(round(xShooter), round(yShooter)), color, 1);
	}
}

// show the balls on the remote viewer, also used for cyan and magenta
void remote::drawFloorBalls(size_t index, Scalar color, Scalar colorDark, size_t type) {
	ballObstListSt balls = multRecv->getBallList(index, type);
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);

	if ((goodEnoughLoc.state != invalid) && (balls.state != invalid)) {
		float xRatio = remoteRatio * goodEnoughLoc.pos.x;
		float yRatio = remoteRatio * goodEnoughLoc.pos.y;
		float rz = M_PI * goodEnoughLoc.pos.rz / 180.0;

		if (balls.list.size() > 0) {
			for (size_t ii = 0; ii < balls.list.size(); ii++) {

				// azimuth provided in radians
				float azimuth = balls.list[ii].azimuth + rz;

				// elevation provided in radians
				float elevation = balls.list[ii].elevation;

				// radius provided in meters
				float radiusMeter = balls.list[ii].radius;
				// the remoteRatio expects pixels of 2 cm instead of meters
				float radius = remoteRatio * radiusMeter / 0.02;

				// x-axis is 0 degrees on rectangular floor
				float x = xRatio + radius * cos(azimuth);
				float y = yRatio - radius * sin(azimuth);

				// because the field is symmetrical omniCam can come up with two solutions, swap if needed
				if (flip[index]) {
					x = width - x;
					y = height - y;
				}

				// change the color when the robot position is old or ball information is old
				if ((goodEnoughLoc.state == old) || (balls.state == old)) {
					color = colorDark;
				}

				int ballRadius = log(balls.list[ii].size);
				if ((elevation < -2.0 * M_PI / 180.0) || (elevation == 0.0)) { // elevation above 3.0 degrees is flying ball
					// ball on floor
					circle(robots[index], Point(round(x), round(y)), ballRadius, color, -1);
				} else {
					// ball in air
					circle(robots[index], Point(round(x), round(y)), ballRadius, color, 1);
				}
			}
		}
	}
}

// if available print the position of largest ball in remote viewer, also used for cyan and magenta
void remote::printBallPosition(size_t index, size_t type, size_t leftIndent, int line, Scalar color) {
	ballObstListSt balls = multRecv->getBallList(index, type);
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);
	char buf[256];
	char *bufp = buf;
	char textLabel[16];

	if (type == TYPE_BALLDETECTION) {
		strcpy(textLabel, "ball ");
	} else if (type == TYPE_CYANDETECTION) {
		strcpy(textLabel, "cyan");
	} else if (type == TYPE_MAGENTADETECTION) {
		strcpy(textLabel, "mage");
	} else {
		printf("ERROR     : type %zu out of range\n", type);
		exit(EXIT_FAILURE);
	}

	if ((goodEnoughLoc.state != invalid) && (balls.state != invalid)) {
		float x = goodEnoughLoc.posRos.x;
		float y = goodEnoughLoc.posRos.y;
		float rz = M_PI * goodEnoughLoc.pos.rz / 180.0;

		if (balls.list.size() > 0) {
			// angle provided in radians
			float azimuth = balls.list[0].azimuth + rz;
			// TODO: checkout if we want to use elevation

			// radius provided in meters
			float radiusMeter = balls.list[0].radius;

			// x-axis is 0 degrees on rectangular floor
			x += radiusMeter * cos(azimuth);
			y -= radiusMeter * sin(azimuth);

			// because the field is symmetrical omniCam can come up with two solutions, swap if needed
			if (flip[index]) {
				x = width - x;
				y = height - y;
			}
			sprintf(bufp, "%s x %3.1f cm y %3.1f cm", textLabel, 100.0 * x, -100.0 * y); // invert y because pixel direction in y is inverted
			putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
		}
	}
}

// show the obstacles on the rectangular floor
void remote::drawFloorObstacles(size_t index) {
	ballObstListSt obstacles = multRecv->getObstacleList(index);
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);

	if ((goodEnoughLoc.state != invalid) && (obstacles.state != invalid)) {
		float xRatio = remoteRatio * goodEnoughLoc.pos.x;
		float yRatio = remoteRatio * goodEnoughLoc.pos.y;
		float rz = M_PI * goodEnoughLoc.pos.rz / 180.0;

		if (obstacles.list.size() > 0) {
			for (size_t ii = 0; ii < obstacles.list.size(); ii++) {

				// angle provided in radians
				float azimuth = obstacles.list[ii].azimuth + rz;

				// radius provided in meters
				float radiusMeter = obstacles.list[ii].radius;
				// the remoteRatio expects pixels of 2 cm instead of meters
				float radius = remoteRatio * radiusMeter / 0.02;

				// x-axis is 0 degrees on rectangular floor
				float x = xRatio + radius * cos(azimuth);
				float y = yRatio - radius * sin(azimuth);

				// because the field is symmetrical omniCam can come up with two solutions, swap if needed
				if (flip[index]) {
					x = width - x;
					y = height - y;
				}

				Scalar color = COLOR_OBSTACLE;

				// change the color when the robot position is old or obstacle information is old
				if ((goodEnoughLoc.state == old) || (obstacles.state == old)) {
					color = COLOR_OBSTACLE_OLD
					;
				}

				int obstacleRadius = log(obstacles.list[ii].size);
				circle(robots[index], Point(round(x), round(y)), obstacleRadius, color, -1);
			}
		}
	}
}

void remote::floorPrintText(size_t index, Scalar color) {
	locListSt locs = multRecv->getLocList(index);
	statsSt stats = multRecv->getStats(index);
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);

	char buf[256];
	char *bufp = buf;
	int line = 22;
	int leftIndent = 54;
	int rightIndent = width - 300;
	int bottomIndent = height + 2;
	int lineDelta = 17;

	// sprintf(bufp, "Falcon %d", (int)(index + 1));
	// putText(robots[index], buf, Point(leftIndent,line), 1, 1, color, 1); line+=lineDelta;
	sprintf(bufp, "Frame %6d loc %6d uptime %7.1f  fps", stats.prepFrames, stats.locFrames, stats.uptime);
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);

	Scalar colorTmp = color;
	if (stats.prepFps < 28.0) {
		colorTmp = Scalar(0, 0, 255);
	}
	sprintf(bufp, "%4.1f", stats.prepFps);
	putText(robots[index], buf, Point(leftIndent + 395, line), 1, 1, colorTmp, 1);

	sprintf(bufp, "%4.1f", stats.locFps);
	putText(robots[index], buf, Point(leftIndent + 440, line), 1, 1, color, 1);
	line += lineDelta;

	sprintf(bufp, "Balls %2d Cyan %2d Magenta %2d Pos pixels %3d Obst %3d Line Pnts %3d", stats.ballAmount,
			stats.cyanAmount, stats.magentaAmount, stats.possessionPixels, stats.obstacleAmount, stats.linePoints);
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;

#ifdef NONO
	// goodEnoughLoc.pos range x[0:976], y[0:700], rz[0:359]
	sprintf(bufp, "local x %3.1f y %3.1fd %3.1f",
			goodEnoughLoc.pos.x, goodEnoughLoc.pos.y, goodEnoughLoc.pos.rz );
	putText(robots[index], buf, Point(leftIndent,line), 1, 1, color, 1); line+=lineDelta;
#endif

	// calculate the real world x, y and rz (meters and radians)
	// TODO: determine the following numbers from configuration
	int xCenter = (977 - 1) / 2; // Width is odd number
	int yCenter = (701 - 1) / 2; // Height is odd number
	// TODO: add usage of metersToPixels
	double x = 0.02f * (goodEnoughLoc.pos.x - xCenter); // range -(9+1) to (9+1) meter, center point is 0, negative number is on left half floor, positive numbers right half floor
	double y = -0.02f * (goodEnoughLoc.pos.y - yCenter); // range -(6+1) to (6+1) meter, center point is 0, negative number is on the bottom, positive number is on the top
	double angleDeg = fmod(goodEnoughLoc.pos.rz - 90.0 + 360.0, 360.0); // align with the ROS angle, which is on the y-axis instead of x-axis
	if (angleDeg > 180.0) {
		angleDeg -= 360.0;
	}

#ifdef NONO
	// rz of 0 is on the positive x axis
	sprintf(bufp, "local x %3.1f cm y %3.1f cm %3.1f deg", 100.0*x, 100.0*y, angleDeg );
	putText(robots[index], buf, Point(leftIndent,line), 1, 1, color, 1); line+=lineDelta;
#endif

	x = goodEnoughLoc.posRos.x;
	y = goodEnoughLoc.posRos.y;
	angleDeg = goodEnoughLoc.posRos.rz * 180.0 / CV_PI;
	// because the field is symmetrical omniCam can come up with two solutions, swap if needed
	if (flip[index]) {
		x = -x;
		y = -y;
		angleDeg = fmod(angleDeg + 180, 360);
	}
	if (angleDeg > 180.0) {
		angleDeg -= 360.0;
	}

	sprintf(bufp, "ros  x %3.1f cm y %3.1f cm %3.1f deg", 100.0 * x, -100.0 * y, angleDeg);
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;

	positionStDbl rosAvg = multRecv->getGoodEnoughLocRosAverage(index);
	x = rosAvg.x;
	y = rosAvg.y;
	angleDeg = rosAvg.rz * 180 / CV_PI;
	// because the field is symmetrical omniCam can come up with two solutions, swap if needed
	if (flip[index]) {
		x = -x;
		y = -y;
		angleDeg = fmod(angleDeg + 180, 360);
	}
	if (angleDeg > 180.0) {
		angleDeg -= 360.0;
	}
	sprintf(bufp, "ravg x %3.1f cm y %3.1f cm %3.1f deg", 100.0 * x, -100.0 * y, angleDeg);
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;

	printBallPosition(index, TYPE_BALLDETECTION, leftIndent, line, color);
	line += lineDelta;
	printBallPosition(index, TYPE_CYANDETECTION, leftIndent, line, color);
	line += lineDelta;
	printBallPosition(index, TYPE_MAGENTADETECTION, leftIndent, line, color);
	line += lineDelta;

	line = bottomIndent - 5 * lineDelta;
	bufp += sprintf(bufp, "Score ");
	for (size_t ii = 0; ii < locs.list.size(); ii++) {
		bufp += sprintf(bufp, " %5.3f", locs.list[ii].score);
	}
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;
	bufp = buf; // reset buffer pointer

	bufp += sprintf(bufp, "Age   ");
	for (size_t ii = 0; ii < locs.list.size(); ii++) {
		bufp += sprintf(bufp, " %5d", locs.list[ii].age);
	}
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;
	bufp = buf; // reset buffer pointer

	bufp += sprintf(bufp, "Active ");

	Scalar activeColor = color;
	for (size_t ii = 0; ii < locs.list.size(); ii++) {
		if ((ii == 0) && (locs.list[0].lastActive > 5)) {
			activeColor = {0, 0, 255};}
		bufp += sprintf(bufp, " %5d", locs.list[ii].lastActive);
	}
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, activeColor, 1);
	line += lineDelta;
	bufp = buf; // reset buffer pointer

	bufp += sprintf(bufp, "Tries  ");
	for (size_t ii = 0; ii < locs.list.size(); ii++) {
		bufp += sprintf(bufp, " %5d", locs.list[ii].numberOfTries);
	}
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;
	bufp = buf; // reset buffer pointer

	bufp += sprintf(bufp, "Pixels ");
	for (size_t ii = 0; ii < locs.list.size(); ii++) {
		bufp += sprintf(bufp, " %5d", locs.list[ii].amountOnFloor);
	}
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);
	line += lineDelta;
	// putText(robots[index], buf, Point(leftIndent,height-lineDelta), 1, 1, color, 1);
	bufp = buf; // reset buffer pointer

	line = bottomIndent - 2 * lineDelta;
	sprintf(bufp, "Packet errors %5zu", multRecv->getPacketErrorCnt(index));
	putText(robots[index], buf, Point(rightIndent, line), 1, 1, color, 1);
	line += lineDelta;

	// latency in milliseconds
	int64_t latency = (uint64_t) stats.localTime - (uint64_t) stats.remoteTime;
	// correct for possible time zone differences
	// double oneHour = 60*60*1000*1000;
	// latency = latency & oneHour;
	// showing latencies of less then 2 minutes (e.g. because of recordings or unsynchronized time on  remote viewer laptop
	if (abs(latency) < (2 * 60 * 1000 * 1000)) {
		sprintf(bufp, "Time %9s Latency %4.1f ms", stats.remoteTimeString, (double) latency / 1000.0);
		if (abs(latency) > 1000000) {
			// latency > 100ms in red
			putText(robots[index], buf, Point(rightIndent, line), 1, 1, Scalar(0, 0, 255), 1);
			line += lineDelta;
		} else {
			putText(robots[index], buf, Point(rightIndent, line), 1, 1, color, 1);
			line += lineDelta;
		}
	} else {
		sprintf(bufp, "Time %9s Latency -", stats.remoteTimeString);
		putText(robots[index], buf, Point(rightIndent, line), 1, 1, color, 1);
		line += lineDelta;
	}
}

void remote::update() {
	for (size_t ii = 0; ii < robots.size(); ii++) {
#ifndef NONO
		// start with an empty floor
		grassFrame.copyTo(robots[ii]);
#else
		// start with the history of the previous obstacles, cyan, magenta and balls from the previous run
		robotsHistory[ii].copyTo(robots[ii]);
#endif
	}

	for (size_t ii = 0; ii < robots.size(); ii++) {
		drawFloorObstacles(ii);
		drawFloorBalls(ii, COLOR_CYAN, COLOR_CYAN_SMALL, TYPE_CYANDETECTION);
		drawFloorBalls(ii, COLOR_MAGENTA, COLOR_MAGENTA_SMALL, TYPE_MAGENTADETECTION);
		drawFloorBalls(ii, COLOR_BALL, COLOR_BALL_SMALL, TYPE_BALLDETECTION); // balls on top of obstacles, cyan or magenta
	}
	for (size_t ii = 0; ii < robots.size(); ii++) {
		// re-use the robots[ii] with only obstacles, cyan, magenta and balls for the next cycle
		// robots[ii].copyTo(robotsHistory[ii]);
	}
	for (size_t ii = 0; ii < robots.size(); ii++) {
		// do not keep an history of the robot position, line points and text
		drawFloorRobot(ii);
		drawFloorLinePoints(ii);
		floorPrintText(ii, COLOR_FLOOR_TEXT);
	}

	for (size_t ii = 0; ii < robots.size(); ii++) {
		int row = ii / 3;
		int col = ii % 3;
		robots[ii].copyTo(allFrame.rowRange(row * height, (row + 1) * height).colRange(col * width, (col + 1) * width)); // y, x, depth
	}
	imshow("q to quit", allFrame);
	usleep(10000);
}

void remote::startReceiveThread() {
	receiveThread = thread(&multicastReceive::receive, multRecv);
}

void remote::packetIndexAdd(int value) {
	multRecv->packetIndexAdd(value);
}

int main(int argc, char** argv) {
	int opt = 0;
	while ((opt = getopt(argc, argv, "h")) != -1) {
		switch (opt) {
		case 'h':
			printf("INFO      : no arguments required\n");
			break;
		}
	}

	remote *rmt = new remote();

	rmt->startReceiveThread();

	bool busy = true;
	while (busy) {
		rmt->update();
		int key = waitKey(20);
		// if( key >= 0 ) { printf( "key: %d\n", key );  }
		switch (key) {
		case 1048603: // escape
		case 1179729: // caps lock q
		case 1048689: // q
		case 27: // escape
		case 'q':
			printf("INFO      : exit now\n");
			busy = false;
			break;
		case 1048625: // 1
		case '1':
			rmt->doFlip(0);
			break;
		case 1048626: // 2
		case '2':
			rmt->doFlip(1);
			break;
		case 1048627: // 3
		case '3':
			rmt->doFlip(2);
			break;
		case 1048628: // 4
		case '4':
			rmt->doFlip(3);
			break;
		case 1048629: // 5
		case '5':
			rmt->doFlip(4);
			break;
		case 1048630: // 6
		case '6':
			rmt->doFlip(5);
			break;
		case 1048673: // a
		case 'a':
			rmt->packetIndexAdd(-1000);
			break;
		case 1048675: // c
		case 'c':
			rmt->packetIndexAdd(+10);
			break;
		case 1048676: // d
		case 'd':
			rmt->packetIndexAdd(+100);
			break;
		case 'e':
			rmt->packetIndexAdd(+1);
			break;
		case 1048678: // f
		case 'f':
			rmt->packetIndexAdd(+1000);
			break;
		case 1048688: // p
		case 'p':
			rmt->packetIndexPauseToggle();
			break;
		case 1048691: // s
		case 's':
			rmt->packetIndexAdd(-100);
			break;
		case 'w':
			rmt->packetIndexAdd(-1);
			break;
		case 1048696: // x
		case 'x':
			rmt->packetIndexAdd(-10);
			break;
		case 1048608: // space
		case ' ':
			rmt->packetIndexPauseToggle();
			break;
		default:
			break;
		}
	}
	return 0;
}
