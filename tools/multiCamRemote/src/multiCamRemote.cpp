// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "multiCamRemote.hpp"

#include <unistd.h> // getopt
#include <sys/time.h> // gettimeofday

using namespace cv;
using namespace std;

multiCamRemote::multiCamRemote() {
	conf = new configurator(1); // the robot index of 1 is not used by multiCamRemote (it does not require specific robot settings)
	conf->setRemote(true); // used to generate a scaled rFloor version
	rFloor = new robotFloor(conf);
	multRecv = new multicastReceive();

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
	for (size_t ii = 0; ii < MAX_ROBOTS; ii++) {
		robots.push_back(Mat::zeros(height, width, CV_8UC3)); // y, x, depth
		flip.push_back(false); // when starting none of the robot images is mirrored
		robotAlive[ii] = false;
	}

	// create the frame for all 2*3 = 6 robots
	allFrame = Mat::zeros(2 * height, 3 * width, CV_8UC3); // y, x, depth

	for (size_t ii = 0; ii < robots.size(); ii++) {
		grassFrame.copyTo(robots[ii]);
	}
}

void multiCamRemote::drawFloorLinePoints(size_t index) {
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

			// because the field is symmetrical vision can come up with two solutions, swap if needed
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
				color = COLOR_LINEPOINTS_INVALID;
			}

			line(robots[index], Point(round(x), round(y)), Point(round(x), round(y)), color, 2);
		}
	}
}

// show rz rotated robot at position x, y in the rectangular field viewer
void multiCamRemote::drawFloorRobot(size_t index) {
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
			color = COLOR_ROBOT_OLD;
		}

		// make robot yellow when ball possession
		int thickness = 1;
		if (multRecv->getStats(index).possessionPixels > multRecv->getPossessionThreshold()) {
			thickness = -1; // negative thickness means a filled circle is to be drawn
			color = COLOR_ROBOT_BALLPOSSESSION;
		}

		// because the field is symmetrical vision can come up with two solutions, swap if needed
		if (flip[index]) {
			xRatio = width - xRatio;
			yRatio = height - yRatio;
			rz = CV_PI + rz;
		}

		int circleRadius = 12; // obstacle / robot ~ 25cm radius, field 2cm = 1 pixel, obstacle ~ 12 pixels
		circle(robots[index], Point(round(xRatio), round(yRatio)), circleRadius, color, thickness);

		// rz = 0 on y-axis, anti-clockwise
		float xShooter = xRatio + 4.0f * circleRadius * sin(-rz);
		float yShooter = yRatio - 4.0f * circleRadius * cos(rz); // subtract because negative y is on top and positive y is on bottom
		line(robots[index], Point(round(xRatio), round(yRatio)), Point(round(xShooter), round(yShooter)), color, 1);
	}
}

// show the balls on the remote viewer, also used for cyan and magenta
void multiCamRemote::drawFloorBalls(size_t index, Scalar color, Scalar colorDark, size_t type) {
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

				// because the field is symmetrical vision can come up with two solutions, swap if needed
				if (flip[index]) {
					x = width - x;
					y = height - y;
				}

				// change the color when the robot position is old or ball information is old
				if ((goodEnoughLoc.state == old) || (balls.state == old)) {
					color = colorDark;
				}

				int ballRadius = log(balls.list[ii].size);
				// elation range provided is 0 to 360, convert to -180 to 180
				if (elevation > M_PI) {
					elevation -= 2 * M_PI;
				}
				// printf("elevation %5.1f degrees\n", 180.0 * elevation / M_PI);
				if ((elevation < -7.0 * M_PI / 180.0) || (elevation == 0.0)) { // elevation above -7.0 degrees is flying ball
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
void multiCamRemote::printBallPosition(size_t index, size_t type, size_t leftIndent, int line, Scalar color) {
	ballObstListSt balls = multRecv->getBallList(index, type);
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);
	char buf[256];
	char *bufp = buf;
	char textLabel[16];

	if (type == TYPE_BALLDETECTION) {
		strcpy(textLabel, "ball ");
	} else if (type == TYPE_BALLFARDETECTION) {
		strcpy(textLabel, "farb");
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

			// because the field is symmetrical vision can come up with two solutions, swap if needed
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
void multiCamRemote::drawFloorObstacles(size_t index) {
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

				// because the field is symmetrical vision can come up with two solutions, swap if needed
				if (flip[index]) {
					x = width - x;
					y = height - y;
				}

				Scalar color = COLOR_OBSTACLE;

				// change the color when the robot position is old or obstacle information is old
				if ((goodEnoughLoc.state == old) || (obstacles.state == old)) {
					color = COLOR_OBSTACLE_OLD;
				}

				int obstacleRadius = log(obstacles.list[ii].size);
				circle(robots[index], Point(round(x), round(y)), obstacleRadius, color, -1);
			}
		}
	}
}

// show the obstacles on the rectangular floor
void multiCamRemote::checkIfRobotStillAlive(size_t index) {

	if (!multRecv->getPacketIndexPause()) {
		statsSt stats = multRecv->getStats(index);
		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t localTime = tv.tv_sec * 1000000 + tv.tv_usec;
		uint64_t deltaTime = localTime - stats.localTime;
		if (deltaTime > 3 * 1000000) { // 3 seconds
			robotAlive[index] = false;
		} else {
			robotAlive[index] = true;
		}
	}
	if (!robotAlive[index]) {
		line(robots[index], Point(0, height), Point(width, 0), Scalar(0, 0, 255), 2);
		line(robots[index], Point(0, 0), Point(width, height), Scalar(0, 0, 255), 2);
	}

}

void multiCamRemote::floorPrintText(size_t index, Scalar color) {
	locListSt locs = multRecv->getLocList(index);
	statsSt stats = multRecv->getStats(index);

	char tmpbuf[256];
	char buf[256];
	char *bufp = buf;
	int line = 25;
	int leftIndent = 44;
	int rightIndent = width - 300;
	int bottomIndent = height + 2;
	int lineDelta = 17;

// sprintf(bufp, "Falcon %d", (int)(index + 1));
// putText(robots[index], buf, Point(leftIndent,line), 1, 1, color, 1); line+=lineDelta;
	sprintf(bufp, "Frame %6d loc %6d uptime %7.1f  fps", stats.prepFrames, stats.locFrames, stats.uptime);
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, color, 1);

	Scalar colorTmp = color;
	Scalar colorError = Scalar(0, 0, 255); // red
	if (stats.prepFps < 28.0) {
		colorTmp = colorError;
	}
	sprintf(bufp, "%4.1f", stats.prepFps);
	putText(robots[index], buf, Point(leftIndent + 405, line), 1, 1, colorTmp, 1);

	sprintf(bufp, "%4.1f", stats.locFps);
	putText(robots[index], buf, Point(leftIndent + 450, line), 1, 1, color, 1);
	line += lineDelta;

	for (size_t camIndex = 0; camIndex < 4; camIndex++) {
		raspiStatsSt raspi = stats.raspi[camIndex];
		hhMmSsSt anaApplUp = multRecv->secondsToHhMmSs(raspi.anaApplUptime);
		hhMmSsSt cpuUp = multRecv->secondsToHhMmSs(raspi.cpuUptime);
		sprintf(bufp, "cpu %2u load %4.2f cpu %02u:%02u:%02u ana %02u:%02u:%02u", raspi.cpuTemp, raspi.cpuLoad / 32.0,
				cpuUp.hours, cpuUp.minutes, cpuUp.seconds, anaApplUp.hours, anaApplUp.minutes, anaApplUp.seconds);

		colorTmp = color;

		// determine the average for the other 3 camera's
		float camValAvgOther = 0;
		for (size_t ii = 0; ii < 4; ii++) {
			if (ii != camIndex) {
				camValAvgOther += (float) stats.raspi[ii].camValAverage;
			}
		}
		camValAvgOther = camValAvgOther / 3.0;

		// verify if the current average camera value is within range compared with the values of the other camera's
		float camValAvg = raspi.camValAverage;
		if (camValAvg < (camValAvgOther * 0.7)) { // 30 % lower
			// the camera value for this camera significant lower then of the other camera's
			sprintf(tmpbuf, " cam val %2u TOO DARK", raspi.camValAverage);
            strcat(bufp, tmpbuf);
			colorTmp = colorError;
		} else if (camValAvg > (camValAvgOther * 1.5)) { // 50 % higher (if one camera is lower then the average over the other camera's is significantly lower)
			// the camera value for this camera significant higher then of the other camera's
			sprintf(tmpbuf, " cam val %2u TOO BRIGHT", raspi.camValAverage);
            strcat(bufp, tmpbuf);
			colorTmp = colorError;
		}

		if ((raspi.rebootReceivedAge > 0) && (raspi.rebootReceivedAge < 60)) {
			sprintf(tmpbuf, " REBOOT %2u", raspi.rebootReceivedAge);
            strcat(bufp, tmpbuf);
			colorTmp = colorError;
		}

		if (raspi.cpuTemp >= 70) {
			strcat(bufp, " cpu temp");
			colorTmp = colorError;
		}

		if (raspi.cmosTemp > 57) {
			sprintf(tmpbuf, " cmos %2u", raspi.cmosTemp);
            strcat(bufp, tmpbuf);
			colorTmp = colorError;
		}

		// normal load is between 1.7 and 3.7 (the cpuLoad is in fixed point)
		if ((raspi.cpuLoad < (1.7 * 32)) || (raspi.cpuLoad > (uint8_t) (3.7 * 32))) {
			strcat(bufp, " cpu load");
			colorTmp = colorError;
		}

		if ((raspi.cpuUptime < 60) || (raspi.anaApplUptime < 10)) {
			strcat(bufp, " cpu uptime");
			colorTmp = colorError;
		}

		// 0: under-voltage
		// 1: arm frequency capped
		// 2: currently throttled
		// 3: soft temp limit (added in 2018)

		// WARNING: on sender side bits 16-19 have been shifted to bits 4-7
		// 4: under-voltage has occurred
		// 5: arm frequency capped has occurred
		// 6: throttling has occurred
		// 7: soft temp limit (added in 2018)
		bool underVoltage = (((raspi.cpuStatus >> 0) & 0x1) == 1);
		bool frequencyCapped = (((raspi.cpuStatus >> 1) & 0x1) == 1);
		bool throttling = (((raspi.cpuStatus >> 2) & 0x1) == 1);
		bool softTempLimit = (((raspi.cpuStatus >> 3) & 0x1) == 1);
		bool underVoltageOccured = (((raspi.cpuStatus >> 4) & 0x1) == 1);
		bool frequencyCappedOccured = (((raspi.cpuStatus >> 5) & 0x1) == 1);
		bool throttlingOccured = (((raspi.cpuStatus >> 6) & 0x1) == 1);
		bool softTempLimitOccured = (((raspi.cpuStatus >> 7) & 0x1) == 1);

		if (underVoltage) {
			strcat(bufp, " VOLT");
		} else if (underVoltageOccured) {
			strcat(bufp, " volt");
		}

		if (frequencyCapped) {
			strcat(bufp, " FREQ");
		} else if (frequencyCappedOccured) {
			strcat(bufp, " freq");
		}

		if (throttling) {
			strcat(bufp, " THROT");
		} else if (throttlingOccured) {
			strcat(bufp, " throt");
		}

		if (softTempLimit) {
			strcat(bufp, " SOFT TEMP");
		} else if (softTempLimitOccured) {
			strcat(bufp, " soft temp");
		}

		if (raspi.cpuStatus != 0) {
			// sprintf(bufp, "%s cpu status 0xx%x", bufp, raspi.cpuStatus);
			colorTmp = colorError;
		}

		putText(robots[index], buf, Point(leftIndent, line), 1, 1, colorTmp, 1);
		line += lineDelta;
	}

	colorTmp = color;
#ifdef NONO
// ball and far away ball overlap and count double, use default color
	if (stats.ballAmount > 1) {
		// in a game only 1 ball on floor possible
		colorTmp = colorError;
	}
#endif
	if (stats.linePoints < 60) {
		colorTmp = colorError;
	}

	sprintf(bufp, "low balls %3d pos pixels %4d obst %3d line pnts %3d", stats.ballAmount, stats.possessionPixels,
			stats.obstacleAmount, stats.linePoints);
	putText(robots[index], buf, Point(leftIndent, line), 1, 1, colorTmp, 1);
	line += lineDelta;

#ifdef NONO
	goodEnoughLocSt goodEnoughLoc = multRecv->getGoodEnoughLoc(index);
// goodEnoughLoc.pos range x[0:976], y[0:700], rz[0:359]
	sprintf(bufp, "local x %3.1f y %3.1fd %3.1f",
			goodEnoughLoc.pos.x, goodEnoughLoc.pos.y, goodEnoughLoc.pos.rz );
	putText(robots[index], buf, Point(leftIndent,line), 1, 1, color, 1); line+=lineDelta;
#endif

#ifdef NONO
// calculate the real world x, y and rz (meters and radians)
// TODO: determine the following numbers from configuration
	int xCenter = (977 - 1) / 2;// Width is odd number
	int yCenter = (701 - 1) / 2;// Height is odd number
// TODO: add usage of metersToPixels
	double x = 0.02f * (goodEnoughLoc.pos.x - xCenter);// range -(9+1) to (9+1) meter, center point is 0, negative number is on left half floor, positive numbers right half floor
	double y = -0.02f * (goodEnoughLoc.pos.y - yCenter);// range -(6+1) to (6+1) meter, center point is 0, negative number is on the bottom, positive number is on the top
	double angleDeg = fmod(goodEnoughLoc.pos.rz - 90.0 + 360.0, 360.0);// align with the ros angle, which is on the y-axis instead of x-axis
	if (angleDeg > 180.0) {
		angleDeg -= 360.0;
	}
#endif

#ifdef NONO
// rz of 0 is on the positive x axis
	sprintf(bufp, "local x %3.1f cm y %3.1f cm %3.1f deg", 100.0*x, 100.0*y, angleDeg );
	putText(robots[index], buf, Point(leftIndent,line), 1, 1, color, 1); line+=lineDelta;
#endif

#ifdef NONO
	x = goodEnoughLoc.posRos.x;
	y = goodEnoughLoc.posRos.y;
	angleDeg = goodEnoughLoc.posRos.rz * 180.0 / CV_PI;
// because the field is symmetrical vision can come up with two solutions, swap if needed
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
// because the field is symmetrical vision can come up with two solutions, swap if needed
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
	printBallPosition(index, TYPE_BALLFARDETECTION, leftIndent, line, color);
	line += lineDelta;
	printBallPosition(index, TYPE_CYANDETECTION, leftIndent, line, color);
	line += lineDelta;
	printBallPosition(index, TYPE_MAGENTADETECTION, leftIndent, line, color);
	line += lineDelta;
#endif
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
			activeColor = {0, 0, 255};
			putText(robots[index], "!", Point(2 * width / 9 - 0, 4 * height / 6), 2, 5, Scalar(0, 0, 200), 10);
		}
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
		colorTmp = color;
		if (abs(latency) > 1000000) {
			// latency > 100ms in red
			colorTmp = colorError;
		}
		putText(robots[index], buf, Point(rightIndent, line), 1, 1, colorTmp, 1);
		line += lineDelta;
	} else {
		sprintf(bufp, "Time %9s Latency -", stats.remoteTimeString);
		putText(robots[index], buf, Point(rightIndent, line), 1, 1, color, 1);
		line += lineDelta;
	}
}

void multiCamRemote::update() {
// start with an empty canvas
	allFrame = Mat::zeros(2 * height, 3 * width, CV_8UC3); // y, x, depth

// for each robot (can be more then 6) create the field
	for (size_t ii = 0; ii < robots.size(); ii++) {
		// start with a green floor
		grassFrame.copyTo(robots[ii]);

		// add the robot number in the background (because the location on the canvas is dynamic)
		char buf[256];
		char *bufp = buf;
		sprintf(bufp, "%zu", ii + 1); // index starts at 0 while robot number starts at 1
		putText(robots[ii], buf, Point(4 * width / 7 - 0, 3 * height / 4), 2, 9, Scalar(0, 70, 30), 10);

		// now add the obstacles and balls to the field of one robot
		drawFloorObstacles(ii);
		drawFloorBalls(ii, COLOR_CYAN, COLOR_CYAN_SMALL, TYPE_CYANDETECTION);
		drawFloorBalls(ii, COLOR_MAGENTA, COLOR_MAGENTA_SMALL, TYPE_MAGENTADETECTION);
		drawFloorBalls(ii, COLOR_BALL, COLOR_BALL_SMALL, TYPE_BALLDETECTION); // balls on top of obstacles, cyan or magenta
		drawFloorBalls(ii, COLOR_BALL_FAR, COLOR_BALL_FAR_SMALL, TYPE_BALLFARDETECTION); // far balls on top of obstacles, cyan or magenta

		// draw the floor
		drawFloorRobot(ii);
		drawFloorLinePoints(ii);

		// debug text overlay on the floor
		floorPrintText(ii, COLOR_FLOOR_TEXT);
		checkIfRobotStillAlive(ii);
	}

// the viewer is organized in a 3x2 layout
// it is possible that more then 6 robots are available (of which maximal 5 can play)
// only show robot's that have been active recently
// if more then 6 robot's have been active recently, then show the 6 which most recently provided data

// by default disable all robot's
	bool activeRobot[MAX_ROBOTS];
	for (size_t ii = 0; ii < robots.size(); ii++) {
		activeRobot[ii] = false;
	}

	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t localTime = tv.tv_sec * 1000000 + tv.tv_usec;

// the first element in the statsSort list is the newest and the last element the oldest
// create a list of the most recent updated 6 robots
	std::vector<statsSt> getStatsSort = multRecv->getStatsSort();
	for (size_t ii = 0; ii < 6; ii++) {
		size_t jj = getStatsSort[ii].robotIndex;
		uint64_t deltaTime = localTime - getStatsSort[ii].localTime;
		// only show robots that have send data the last 15 minutes
		if (deltaTime < (15 * 60 * 1000000)) {
			activeRobot[jj] = true;
		} else {
			activeRobot[jj] = false;
		}
	}
// the activeRobot list now contains maximal 6 active entries

// copy the active robots on the canvas
// use the same order as the robot number, but skip robot's that have not send
// a recent update
	for (size_t ii = 0; ii < 6; ii++) {
		int row = ii / 3;
		int col = ii % 3;
		bool notFound = true;
		size_t jj = 0;
		while (notFound) {
			// loop though the active robot list
			if (activeRobot[jj]) {
				robots[jj].copyTo(
						allFrame.rowRange(row * height, (row + 1) * height).colRange(col * width, (col + 1) * width)); // y, x, depth
				// this robot has been used, remove from list for the next location on the canvas
				activeRobot[jj] = false;
				// exit the while loop, to fill in the next location on the canvas
				notFound = false;
			}
			jj++;
			if (jj >= getStatsSort.size()) {
				// there are no more active robot's
				notFound = false;
			}
		}
	}
	imshow("q to quit", allFrame);
}

void multiCamRemote::startReceiveThread() {
	receiveThread = thread(&multicastReceive::receive, multRecv);
}

void multiCamRemote::packetIndexAdd(int value) {
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

	multiCamRemote *rmt = new multiCamRemote();

	rmt->startReceiveThread();

	bool busy = true;
	while (busy) {
		rmt->update();
		int key = waitKey(200);
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
		case 1048631: // 7
		case '7':
			rmt->doFlip(6);
			break;
		case 1048632: // 8
		case '8':
			rmt->doFlip(7);
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

