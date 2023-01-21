// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "fieldLut.hpp"

using namespace cv;

fieldLut::fieldLut(configurator *conf, linePointDetection *linePoint, robotFloor *rFloor) {
	this->conf = conf;
	this->linePoint = linePoint;
	this->rFloor = rFloor;

	pixelThreshold = 90;
}

// calculates the score depending on x, y and rz, return value between 0 and 1, lower value is better
double fieldLut::calc(const double *x) const {
	int costValue = 0;
	int fieldPointsAmount = 0; // dirty way to export information
	int fieldPointsOutsideField = 0;
	double scoreLocal = 0;
	// calculates new points using x, y and rz = (x[0],x[1],x[2])
	size_t linePointsUsed = linePoint->getLinePointsPolar().size();
	if( (int)linePointsUsed > conf->getLine().linePointsNumberMaximal ) {
		linePointsUsed = (size_t)conf->getLine().linePointsNumberMaximal;
	}
	; // more line points will reduce frames per seconds
	for( size_t ii = 0; ii < linePointsUsed; ii++ ) {
		// there is probably an easier function to do this angle/rad to x/y calculation
		float angleRad = linePoint->getLinePointsPolar().at(ii).angle + (2.0f * CV_PI / 360.0f) * x[2];
		float radiusMeters = linePoint->getLinePointsPolar().at(ii).radius / 20.0; // radius provided in mm, solvers uses granularity of 20mm
		int px = round(x[0] + radiusMeters * cos(angleRad));
		int py = round(x[1] - radiusMeters * sin(angleRad)); // upper part view has lower y
		// only calculate the cost of pixels that are on the floor (field + border)
		if( px >= 0 && px <= rFloor->getXFloorRight() && py >= 0 && py <= rFloor->getYFloorBottom() ) {
			costValue += 255 - rFloor->getCost(px, py); // add value of each found pixel to the score
			fieldPointsAmount++;
		} else {
			fieldPointsOutsideField++;
		}
	}

	if( fieldPointsAmount >= conf->getLine().linePointsNumberMinimal ) {
		// at least 60 points need to be inside the field, otherwise the location does not make sense
		scoreLocal = 1.0 * costValue / (fieldPointsAmount * 256.0); // calculate score and normalize between 0 and 1

		// make the score slightly worse for each pixels that is outside the field (to prevent a good score while most pixels are outside the field)
		// a good lock has a score of less then 0.050
		// a bad lock has a score of higher then 0.175
		// 30 points outside the field should result result in a worse score
		// so set 30 points outside the field to a score increase of 0.15, that means each point adds 0.005 to the score
		// TODO: double check this with the goaly
		scoreLocal += fieldPointsOutsideField * 0.005;

		if( scoreLocal > 1.0 ) {
			scoreLocal = 1.0;
		} // truncate for the points outside the field
	} else {
		scoreLocal = 1.0;
	}
	return scoreLocal;
}

// calculates the score depending on x, y and rz, return value between 0 and 1, lower value is better
// APOX todo: this is a temporary function until i figured out how to share this one with the calc function
scoreStruct fieldLut::calcSimple(positionStDbl pos, double threshold) {
	int costValue = 0;
	int fieldPointsAmount = 0; // dirty way to export information
	int fieldPointsOutsideField = 0;
	double scoreLocal = 0;
	// calculates new points using x, y and rz = (x[0],x[1],x[2])
	size_t linePointsUsed = linePoint->getLinePointsPolar().size();
	if( (int)linePointsUsed > conf->getLine().linePointsNumberMaximal ) {
		linePointsUsed = (size_t)conf->getLine().linePointsNumberMaximal;
	}; // more line points will reduce frames per seconds
	for( size_t ii = 0; ii < linePointsUsed; ii++ ) {
		// there is probably an easier function to do this angle/rad to x/y calculation
		float angleRad = linePoint->getLinePointsPolar().at(ii).angle + (2.0f * CV_PI / 360.0f) * pos.rz;
		float radiusMeters = linePoint->getLinePointsPolar().at(ii).radius / 20.0; // radius provided in mm, solvers uses granularity of 20mm
		int px = round(pos.x + radiusMeters * cos(angleRad));
		int py = round(pos.y - radiusMeters * sin(angleRad)); // upper part view has lower y
		// only calculate the cost of pixels that are on the floor (field + border)
		if( px >= 0 && px <= rFloor->getXFloorRight() && py >= 0 && py <= rFloor->getYFloorBottom() ) {
			costValue += 255 - rFloor->getCost(px, py); // add value of each found pixel to the score
			fieldPointsAmount++;
		} else {
			fieldPointsOutsideField++;
		}
	}

	if( fieldPointsAmount >= conf->getLine().linePointsNumberMinimal ) {
		// at least 60 points need to be inside the field, otherwise the location does not make sense
		scoreLocal = 1.0 * costValue / (fieldPointsAmount * 256.0); // calculate score and normalize between 0 and 1

		// make the score slightly worse for each pixels that is outside the field (to prevent a good score while most pixels are outside the field)
		// a good lock has a score of less then 0.050
		// a bad lock has a score of higher then 0.175
		// so set 30 points outside the field to a score increase of 0.15, that means each point adds 0.005 to the score
		// TODO: double check this with the goaly
		scoreLocal += fieldPointsOutsideField * 0.005;

		if( scoreLocal > 1.0 ) {
			scoreLocal = 1.0;
		} // truncate for the points outside the field
	} else {
		scoreLocal = 1.0;
	}

	scoreStruct retVal;
	retVal.score = scoreLocal;
	retVal.amountOnFloor = fieldPointsAmount;
	retVal.amountOffFloor = fieldPointsOutsideField;
	if( scoreLocal < threshold ) {
		retVal.positionFound = true;
	} else {
		retVal.positionFound = false;
	}
	return retVal;
}
