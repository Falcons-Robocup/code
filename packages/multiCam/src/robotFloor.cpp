// Copyright 2018-2020 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Class to create the robot soccer floor.
// The floor is used for robot localization, but also to visualize the results which helps
// testing and debugging.
// For testing the robot floor is also called rectangular floor to distinguish with the
// round viewer, which is the cropped unwarped image from the camara.
// The floor is generated in the class constructor, so each time the multiCam is started.
// The floor is created from the dimensions defined in the multiCam.yaml file.
// These dimensions need to be set as defined in the MSL specifications.
// The MSL specifications define a size including the lines itself.
// In multiCam all sizes are defined from the center of the line.
// The MSL specification is in meters.
// In multiCam all sizes are in pixels.
// The conversion from meters to pixels and using center of the lines is performed when
// importing the numbers from the yaml file.
// The yaml file also contains the ratio from meters to pixels, which is default set to 50.
// To prevent asymmetry issue it is required that the floor, in pixel dimensions, has an exact
// middle in as well the x-axis as y-axis. So the center of the field is exactly at the center
// pixel in the 2 dimensional pixel matrix. This requires that as well the x-axis as y-axis
// require an odd amount of pixels, and because the index starts with 0 and the last pixels
// is part of the matrix, the last pixel index number should be even.
// After importing and converting the dimensions the related sizes will be corrected if the
// index is odd.
//
// The robot floor has the following structure
// floor = field + borders
// field contains:
//   - left penalty area
//   - left goal area
//   - right penalty area
//   - right goal area
//   - left penalty mark
//   - right penalty mark
//   - center circle
//   - center mark
//   - one corner arc on each field corner
//
// For localization this above structure is blurred to make it easier for the algorithm to
// find a difference even if it is not close on the exact location.
// The non blurred floor is used for visualization.

#include "robotFloor.hpp"
#include "fieldLut.hpp"

#include "falconsCommonDirs.hpp" // pathToCodeRepo()

#include <unistd.h>

using namespace std;
using namespace cv;

robotFloor::robotFloor(	configurator *conf ) {
	this->conf = conf;
	calibrateMode = false;
	blurPixels = 0;

	std::string configFile = pathToConfig() + "/multiCam.yaml";
	printf("INFO      : robot floor uses config file: %s\n", configFile.c_str());
	FileStorage fs(configFile, FileStorage::READ);

	// the field dimensions are defined in the global section of yaml file
	FileNode global = fs["global"];
	string fieldDimensions = (string)global["dimensions"];
	printf("INFO      : using field dimensions: %s\n", fieldDimensions.c_str());

	// Inside multiCam all dimensions are in pixels and at the center of the lines.
	// This differs from the MSL definitions !
	// APOX todo: checkout index / size difference, the size below is in pixels, but the matrix is in
	// index numbers where index 0 has already a size of 1.
	FileNode dimensions = fs[fieldDimensions];
	if( conf->getRemote() ) {
		metersToPixels = dimensions["remoteMetersToPixels"];
	} else {
		metersToPixels = dimensions["metersToPixels"];
	}
	remoteRatio = (double)metersToPixels / (double)dimensions["metersToPixels"];

	float lineThickness = (float)dimensions["lineThickness"] * metersToPixels;
	int A = round((float)dimensions["A"] * metersToPixels - lineThickness);
	int B = round((float)dimensions["B"] * metersToPixels - lineThickness);
	int C = round((float)dimensions["C"] * metersToPixels - lineThickness);
	int D = round((float)dimensions["D"] * metersToPixels - lineThickness);
	int E = round((float)dimensions["E"] * metersToPixels - lineThickness);
	int F = round((float)dimensions["F"] * metersToPixels - lineThickness);
	int G = round((float)dimensions["G"] * metersToPixels - lineThickness);
	int H = round((float)dimensions["H"] * metersToPixels - lineThickness);
	int J = round((float)dimensions["J"] * metersToPixels);
	int I = round((float)dimensions["I"] * metersToPixels - lineThickness/2 - J/2);
	int K = round((float)dimensions["K"] * metersToPixels);
	int goalWidth = round((float)dimensions["goalWidth"] * metersToPixels - lineThickness); // goal width measured including both goal post
	int goalDepth = round((float)dimensions["goalDepth"] * metersToPixels - lineThickness/2); // measured including line
	if( goalDepth < 0 ) { goalDepth = 0; } // prevent a negative goalDepth in case the goal post "line" is not needed (and set to 0)
	int goalLineBorder = round((float)dimensions["goalLineBorder"] * metersToPixels + lineThickness/2);
	int touchLineBorder = round((float)dimensions["touchLineBorder"] * metersToPixels + lineThickness/2);

	if( A == 0 ) {
		printf("ERROR     : dimension A not set, probably fieldDimensions argument '%s' incorrect\n", fieldDimensions.c_str());
		exit( EXIT_FAILURE );
	}

	// We need an exact center on all lines, this requires an even number because that
	// last even number is part of the matrix, resulting in a total odd amount of pixels
	cout << "robotFloor: dimension correction for: ";
	if( A % 2 == 1 ) { A--; cout << "A "; } // APOX todo: investigate if error is less with -- or ++
	if( B % 2 == 1 ) { B--; cout << "B "; }
	if( C % 2 == 1 ) { C--; cout << "C "; }
	if( D % 2 == 1 ) { D--; cout << "D "; }
	if( H % 2 == 1 ) { H--; cout << "H "; }
	if( J % 2 == 1 ) { J--; cout << "J "; }
	if( K % 2 == 1 ) { K--; cout << "K "; }
	if( goalWidth % 2 == 1 ) { goalWidth--; cout << "goalWidth "; }
	cout << endl;
	cout << "robotFloor: A " << A << " B " << B << " C " << C << " D " << D << " E " << E << " F " << F << " G " << G << " H " << H << " I " << I << " J " << J << " K " << K << endl;
	cout << "robotFloor: goal line border " << goalLineBorder << " touch line border " << touchLineBorder << " line thickness " << lineThickness << " meters to pixels " << metersToPixels << endl;

	floorSize.xLeft = 0;
	floorSize.xRight = A + 2 * goalLineBorder;
	floorSize.yTop = 0;
	floorSize.yBottom = B + 2 * touchLineBorder;

	fieldSize.xLeft = goalLineBorder;
	fieldSize.xMiddle = goalLineBorder + A/2;
	fieldSize.xRight = goalLineBorder + A;
	fieldSize.yTop = touchLineBorder;
	fieldSize.yMiddle = touchLineBorder + B/2;
	fieldSize.yBottom = touchLineBorder + B;

	penaltySize.xLeftArea.left = goalLineBorder;
	penaltySize.xLeftArea.right = goalLineBorder + E;
	penaltySize.xLeftArea.mark = goalLineBorder + I;
	penaltySize.xRightArea.left = goalLineBorder + A - E;
	penaltySize.xRightArea.right = goalLineBorder + A;
	penaltySize.xRightArea.mark = goalLineBorder + A - I;
	penaltySize.yTop = touchLineBorder + B/2 - C/2;
	penaltySize.yBottom = touchLineBorder + B/2 + C/2;

	goalSize.xLeftArea.left = goalLineBorder;
	goalSize.xLeftArea.right = goalLineBorder + F;
	goalSize.xRightArea.left = goalLineBorder + A - F;
	goalSize.xRightArea.right = goalLineBorder + A;
	goalSize.yTop = touchLineBorder + B/2 - D/2;
	goalSize.yBottom = touchLineBorder + B/2 + D/2;
	goalSize.xDepth = goalDepth; ; // to prevent wrong locking, APOX todo: remove
	goalSize.yPoleToCenter = goalWidth/2; // this one not from the spreadsheet but visually determined, APOX todo: remove (also part of the goal line)

	cornerSize.arcRadius = G;
	cornerSize.arcLineThickness = round(lineThickness); // APOX todo: checkout why this variable is required

	centerArcRadius = H/2;

    /*
	floorSizeFile.open("floorSize.yaml");
	floorSizeFile << "%YAML:1.0" << endl;
	floorSizeFile << "# This file is created/overwritten each time multiCam is started." << endl;
	floorSizeFile << "# It is mainly for debugging." << endl;
	floorSizeFile << "# The information is calculated from: multiCam.yaml:dimensions" << endl;

	floorSizeFile << endl << "floor:" << endl;
	floorSizeFile << "   xFloorLeft : " << floorSize.xLeft << endl;
	floorSizeFile << "   xFloorRight : " << floorSize.xRight << endl;
	floorSizeFile << "   yFloorTop : " << floorSize.yTop << endl;
	floorSizeFile << "   yFloorBottom : " << floorSize.yBottom << endl;

	floorSizeFile << endl << "field:" << endl;
	floorSizeFile << "   xFieldLeft : " << fieldSize.xLeft << endl;
	floorSizeFile << "   xFieldMiddle : " << fieldSize.xMiddle << endl;
	floorSizeFile << "   xFieldRight : " << fieldSize.xRight << endl;
	floorSizeFile << "   yFieldTop : " << fieldSize.yTop << endl;
	floorSizeFile << "   yFieldMiddle : " << fieldSize.yMiddle << endl;
	floorSizeFile << "   yFieldBottom : " << fieldSize.yBottom << endl;

	floorSizeFile << endl << "penalty:" << endl;
	floorSizeFile << "   xLeftPenaltyAreaLeft : " << penaltySize.xLeftArea.left << endl;
	floorSizeFile << "   xLeftPenaltyAreaRight : " << penaltySize.xLeftArea.right << endl;
	floorSizeFile << "   xRightPenaltyAreaLeft : " << penaltySize.xRightArea.left << endl;
	floorSizeFile << "   xRightPenaltyAreaRight : " << penaltySize.xRightArea.right << endl;
	floorSizeFile << "   xLeftPenaltyMark : " << penaltySize.xLeftArea.mark << endl;
	floorSizeFile << "   xRightPenaltyMark : " << penaltySize.xRightArea.mark << endl;
	floorSizeFile << "   yPenaltyAreaTop : " << penaltySize.yTop << endl;
	floorSizeFile << "   yPenaltyAreaBottom : " << penaltySize.yBottom << endl;

	floorSizeFile << endl << "goal:" << endl;
	floorSizeFile << "   xLeftGoalAreaLeft : " << goalSize.xLeftArea.left << endl;
	floorSizeFile << "   xLeftGoalAreaRight : " << goalSize.xLeftArea.right << endl;
	floorSizeFile << "   xRightGoalAreaLeft : " << goalSize.xRightArea.left << endl;
	floorSizeFile << "   xRightGoalAreaRight : " << goalSize.xRightArea.right << endl;
	floorSizeFile << "   yGoalAreaTop : " << goalSize.yTop << endl;
	floorSizeFile << "   yGoalAreaBottom : " << goalSize.yBottom << endl;
	floorSizeFile << "   xGoalDepth : " << goalSize.xDepth << endl;
	floorSizeFile << "   yGoalPoleToCenter : " << goalSize.yPoleToCenter << endl;

	floorSizeFile << endl << "corner:" << endl;
	floorSizeFile << "   cornerArcRadius : " << cornerSize.arcRadius << endl;
	floorSizeFile << "   cornerArcLineThickness : " << cornerSize.arcLineThickness << endl;
	floorSizeFile << "   centerArcRadius : " << centerArcRadius << endl; // APOX todo: this one is not part of the corner data
	floorSizeFile.close();
	*/

	createNormal(); // use the just collected dimensions to create floor

	conf->setFloorSize(floorSize); // provide the floor size to configuration so it can be used by other functions (which do not have a relation with robotFloor)

	fs.release();
}

int robotFloor::getWidth()
{
	return floorSize.xRight + 1;
}

int robotFloor::getHeight()
{
	return floorSize.yBottom + 1;
}

int robotFloor::getXFloorRight()
{
	return floorSize.xRight;
}

int robotFloor::getYFloorBottom()
{
	return floorSize.yBottom;
}

void robotFloor::update()
{
	if( conf->getCalibrateMode() != calibrateMode || conf->getBlurPixels() != blurPixels )
	{
		// so the field mode changed or the amount of blurPixels change, update blur field
		calibrateMode = conf->getCalibrateMode(); // store current mode to determine if we can skip the update in the next run
		blurPixels = conf->getBlurPixels(); // store current blur value to determine if we can skip the update in the next run
		if( calibrateMode )
		{
			createCalibrated();
		}
		else
		{
			createNormal();
		}
	}
}

void robotFloor::createNormal()
{
	int drawLineThickness = 1; // same as calcLineThickness, improves image interpretation
	Scalar lineColor = 255;

	exportMutex.lock();
	frameCost = Mat::zeros(floorSize.yBottom + 1, floorSize.xRight + 1, CV_8UC1);
	Point fieldLeftTop(fieldSize.xLeft, fieldSize.yTop);
	Point fieldMiddleTop(fieldSize.xMiddle, fieldSize.yTop);
	Point fieldRightTop(fieldSize.xRight, fieldSize.yTop);
	Point fieldLeftBottom(fieldSize.xLeft, fieldSize.yBottom);
	Point fieldMidBottom(fieldSize.xMiddle, fieldSize.yBottom);
	Point fieldRightBottom(fieldSize.xRight, fieldSize.yBottom);
	Point fieldCenter(fieldSize.xMiddle, fieldSize.yMiddle);

	Point leftPenaltyAreaLeftTop(penaltySize.xLeftArea.left, penaltySize.yTop);
	Point leftPenaltyAreaRightBottom(penaltySize.xLeftArea.right, penaltySize.yBottom);
	Point rightPenaltyAreaLeftTop(penaltySize.xRightArea.left, penaltySize.yTop);
	Point rightPenaltyAreaRightBottom(penaltySize.xRightArea.right, penaltySize.yBottom);

	Point leftGoalAreaLeftTop(goalSize.xLeftArea.left, goalSize.yTop);
	Point leftGoalAreaRightBottom(goalSize.xLeftArea.right, goalSize.yBottom);
	Point rightGoalAreaLeftTop(goalSize.xRightArea.left, goalSize.yTop);
	Point rightGoalAreaRightBottom(goalSize.xRightArea.right, goalSize.yBottom);

	Point leftTopGoalBlockLeft(fieldSize.xLeft - goalSize.xDepth, fieldSize.yMiddle - goalSize.yPoleToCenter);
	Point leftTopGoalBlockRight(fieldSize.xLeft, fieldSize.yMiddle - goalSize.yPoleToCenter);
	Point leftBottomGoalBlockLeft(fieldSize.xLeft - goalSize.xDepth, fieldSize.yMiddle + goalSize.yPoleToCenter);
	Point leftBottomGoalBlockRight(fieldSize.xLeft, fieldSize.yMiddle + goalSize.yPoleToCenter);
	Point rightTopGoalBlockLeft(fieldSize.xRight, fieldSize.yMiddle - goalSize.yPoleToCenter);
	Point rightTopGoalBlockRight(fieldSize.xRight + goalSize.xDepth, fieldSize.yMiddle - goalSize.yPoleToCenter);
	Point rightBottomGoalBlockLeft(fieldSize.xRight, fieldSize.yMiddle + goalSize.yPoleToCenter);
	Point rightBottomGoalBlockRight(fieldSize.xRight + goalSize.xDepth, fieldSize.yMiddle + goalSize.yPoleToCenter);

	Point leftPenaltyMark(penaltySize.xLeftArea.mark, fieldSize.yMiddle);
	Point rightPenaltyMark(penaltySize.xRightArea.mark, fieldSize.yMiddle);

	rectangle(frameCost, fieldLeftTop, fieldRightBottom, lineColor, drawLineThickness); // field
	rectangle(frameCost, leftPenaltyAreaLeftTop, leftPenaltyAreaRightBottom, lineColor, drawLineThickness); // left penalty  area
	rectangle(frameCost, rightPenaltyAreaLeftTop, rightPenaltyAreaRightBottom, lineColor, drawLineThickness); // right penalty area
	rectangle(frameCost, leftGoalAreaLeftTop, leftGoalAreaRightBottom, lineColor, drawLineThickness); // left goal
	rectangle(frameCost, rightGoalAreaLeftTop, rightGoalAreaRightBottom, lineColor, drawLineThickness); // right goal

	// APOX todo: investigate if pole lines are required because the goal should be removed by the sun spot detector
	line(frameCost, leftTopGoalBlockLeft, leftTopGoalBlockRight, lineColor, drawLineThickness); // block left top goal post
	line(frameCost, leftBottomGoalBlockLeft, leftBottomGoalBlockRight, lineColor, drawLineThickness); // block left bottom goal post
	line(frameCost, rightTopGoalBlockLeft, rightTopGoalBlockRight, lineColor, drawLineThickness); // block right top goal post
	line(frameCost, rightBottomGoalBlockLeft, rightBottomGoalBlockRight, lineColor, drawLineThickness); // block right bottom goal post

	int goalPostOut = conf->getGoalPostOut( );
	if( goalPostOut > 0 ) {
		line(frameCost, leftTopGoalBlockRight + Point(-3,-goalPostOut), leftTopGoalBlockRight + Point(-30,-10*goalPostOut), lineColor, drawLineThickness); // block left top goal post
		line(frameCost, leftBottomGoalBlockRight + Point(-3,goalPostOut), leftBottomGoalBlockRight + Point(-30,10*goalPostOut), lineColor, drawLineThickness); // block left top goal post
		line(frameCost, rightTopGoalBlockLeft + Point(3,-goalPostOut), rightTopGoalBlockLeft + Point(30,-10*goalPostOut), lineColor, drawLineThickness); // block left top goal post
		line(frameCost, rightBottomGoalBlockLeft + Point(3,goalPostOut), rightBottomGoalBlockLeft + Point(30,10*goalPostOut), lineColor, drawLineThickness); // block left top goal post
	}

	int goalPostIn = conf->getGoalPostIn( );
	if( goalPostIn > 0 ) {
		line(frameCost, leftTopGoalBlockRight + Point(4,-goalPostIn), leftTopGoalBlockRight + Point(40,-10*goalPostIn), lineColor, drawLineThickness); // block left top goal post
		line(frameCost, leftBottomGoalBlockRight + Point(4,goalPostIn), leftBottomGoalBlockRight + Point(40,10*goalPostIn), lineColor, drawLineThickness); // block left top goal post
		line(frameCost, rightTopGoalBlockLeft + Point(-4,-goalPostIn), rightTopGoalBlockLeft + Point(-40,-10*goalPostIn), lineColor, drawLineThickness); // block left top goal post
		line(frameCost, rightBottomGoalBlockLeft + Point(-4,goalPostIn), rightBottomGoalBlockLeft + Point(-40,10*goalPostIn), lineColor, drawLineThickness); // block left top goal post
	}

	line(frameCost, fieldMiddleTop, fieldMidBottom, lineColor, drawLineThickness); // halfway line
	circle(frameCost, leftPenaltyMark, drawLineThickness + 1, lineColor, -1); // left penalty point
	circle(frameCost, rightPenaltyMark, drawLineThickness + 1, lineColor, -1); // right penalty point
	circle(frameCost, fieldCenter, centerArcRadius, lineColor, drawLineThickness); // center circle
	circle(frameCost, fieldCenter, drawLineThickness + 1, lineColor, -1); // center point

	Mat cornerArc = Mat::zeros(cornerSize.arcRadius + cornerSize.arcLineThickness / 2, cornerSize.arcRadius + cornerSize.arcLineThickness / 2, CV_8UC1);
	circle(cornerArc, Point(0, 0), cornerSize.arcRadius, lineColor, drawLineThickness); // corner arc
	line(cornerArc, Point(0, 0), Point(cornerArc.cols, 0), lineColor, drawLineThickness); // x line, rewrite corner lines to make merging simpler
	line(cornerArc, Point(0, 0), Point(0, cornerArc.rows), lineColor, drawLineThickness); // y line

	cornerArc.copyTo(frameCost.colRange(fieldSize.xLeft, fieldSize.xLeft + cornerArc.cols).rowRange(fieldSize.yTop, fieldSize.yTop + cornerArc.rows));
	cv::flip(cornerArc, cornerArc, 1);
	cornerArc.copyTo(frameCost.colRange(fieldSize.xRight - cornerArc.cols + 1, fieldSize.xRight + 1).rowRange(fieldSize.yTop, fieldSize.yTop + cornerArc.rows));
	cv::flip(cornerArc, cornerArc, 0);
	cornerArc.copyTo(frameCost.colRange(fieldSize.xRight - cornerArc.cols + 1, fieldSize.xRight + 1).rowRange(fieldSize.yBottom - cornerArc.rows + 1, fieldSize.yBottom + 1));
	cv::flip(cornerArc, cornerArc, 1);
	cornerArc.copyTo(frameCost.colRange(fieldSize.xLeft, fieldSize.xLeft + cornerArc.cols).rowRange(fieldSize.yBottom - cornerArc.rows + 1, fieldSize.yBottom + 1));

	frameCost.copyTo(refField); // the frameCost is re-used by the GaussianBlur function

	if( ! conf->getRemote() ) {
		blurFloor( ); // uses frameCost as input and output

		// for( int ii = 0; ii < 20; ii++ ) {
		//	frameCost.colRange(0,500).rowRange(0,500).copyTo(blaat.colRange(ii,500+ii).rowRange(ii,500+ii));
		//	frameCost += ((20 - ii)/200.0f)  * blaat;

		// GaussianBlur(frameCost, frameCost, Size(kernelSize, kernelSize), sigma, sigma, BORDER_DEFAULT);
		// Mat gausKern(getGaussianKernel(kernelSize, sigma, CV_64F)); // APOX: the cost table lines are still pretty small, increase kernel size to e.g. 51
		// frameCost /= 40.0f;
		// frameCost /= gausKern.row(10);
	}
	exportMutex.unlock();
}

// convert the floor into a blurred floor
// this is done by calculating for each pixel the values of the surrounding pixels
// because the surrounding also have surrounding, the calculation is run iterative
// this calculation is not part of the main calculation loop, so optimization is not
// that important for this function
void robotFloor::blurFloor()
{
	Mat tmpFloor; // 3rd pointer as helper to swap the inputFloor and outputFloor pointer
	inputFloor = frameCost;

	for( int ii = 0; ii < conf->getBlurPixels(); ii++ ) {
		float blurValue = cos( 2.0f*CV_PI*ii/(4*conf->getBlurPixels()) ); // use only first 45 degrees of cos curve
		// std::cout << " " << blurValue;
		blurPixel( blurValue );
		tmpFloor = inputFloor; // swap the inputFloor and outputFloor pointers, so we can safe the copyTo method for each loop
		inputFloor = outputFloor;
		outputFloor = tmpFloor;
	}
	frameCost = inputFloor; // this is the pointer to the last outputFloor
}

// copy each pixel to a new floor and copy also the 8 Neighbours to the new floor
// the 8 Neighbours should have a lower value then the center pixel
void robotFloor::blurPixel( float blurValue )
{
	outputFloor = Mat::zeros(floorSize.yBottom + 1, floorSize.xRight + 1, CV_8UC1);
	int pixelVal;
	for( int x = 1; x < floorSize.xRight; x++ ) { // range is from 0 to <= xFloorRight, but use difference for xDelta
		for( int y = 1; y < floorSize.yBottom; y++ ) { // range is from 0 to <= yFloorBottom, but use difference for yDelta
			pixelVal = inputFloor.at<uchar>(y,x);
		    if( outputFloor.at<uchar>(y  ,x  ) < pixelVal ) { outputFloor.at<uchar>(y  ,x  ) = pixelVal; } // center pixel
		    pixelVal = (int)(blurValue*pixelVal + 0.5f );
		    if( outputFloor.at<uchar>(y-1,x-1) < pixelVal ) { outputFloor.at<uchar>(y-1,x-1) = pixelVal; } // bottom left
		    if( outputFloor.at<uchar>(y-1,x  ) < pixelVal ) { outputFloor.at<uchar>(y-1,x  ) = pixelVal; } // bottom
		    if( outputFloor.at<uchar>(y-1,x+1) < pixelVal ) { outputFloor.at<uchar>(y-1,x+1) = pixelVal; } // bottom right
		    if( outputFloor.at<uchar>(y  ,x+1) < pixelVal ) { outputFloor.at<uchar>(y  ,x+1) = pixelVal; } // right
		    if( outputFloor.at<uchar>(y+1,x+1) < pixelVal ) { outputFloor.at<uchar>(y+1,x+1) = pixelVal; } // top right
		    if( outputFloor.at<uchar>(y+1,x  ) < pixelVal ) { outputFloor.at<uchar>(y+1,x  ) = pixelVal; } // top
		    if( outputFloor.at<uchar>(y+1,x-1) < pixelVal ) { outputFloor.at<uchar>(y+1,x-1) = pixelVal; } // top left
		    if( outputFloor.at<uchar>(y  ,x-1) < pixelVal ) { outputFloor.at<uchar>(y  ,x-1) = pixelVal; } // left
		}
	}
}

void robotFloor::createCalibrated()
{
	int drawLineThickness = 1; // same as calcLineThickness, improves image interpretation
	Scalar lineColor = 255;

	exportMutex.lock();
	frameCost = Mat::zeros(floorSize.yBottom + 1, floorSize.xRight + 1, CV_8UC1);

	Point fieldCenter(fieldSize.xMiddle, fieldSize.yMiddle);
	float x, y;
	Point left, right, top, bottom;
	// tile is 25x25cm cm to pixel ratio is 2cm is one pixel
	float tile = 12.5f;
	float range = 250.0f; // -250 pixels to 250 pixels = 1000 + 2 cm
	// note MSL specs measures outside line (because line is part of the 1200x1800cm field)
	// multiCam uses center of lines, so for multiCam the field is a little smaller
	// then 1200x1800cm, however this does not apply for the calibration floor
	// So in the viewer the calibration floor has a higher y then on the normal floor
	for( x = -range; x <= range; x = x + tile )
	{
		left = fieldCenter + Point(round(x), round(-range));
		right = fieldCenter + Point(round(x) , round(range));
		line(frameCost, left, right, lineColor, drawLineThickness);
	}
	for( y = -range; y <= range; y = y + tile )
	{
		top = fieldCenter + Point(round(-range), round(y));
		bottom = fieldCenter + Point(round(range) , round(y));
		line(frameCost, top, bottom, lineColor, drawLineThickness);
	}
	circle(frameCost, fieldCenter, drawLineThickness + 1, lineColor, -1); // center point

	frameCost.copyTo(refField); // the frameCost is re-used by the GaussianBlur function

	blurFloor( ); // uses frameCost as input and output
	exportMutex.unlock();
}


bool robotFloor::inField( Point position )
{
	return ( position.x >= fieldSize.xLeft && position.x <= fieldSize.xRight && position.y >= fieldSize.yTop && position.y <= fieldSize.yBottom );
}

cv::Mat robotFloor::getRefField() {
	cv::Mat retVal;
	exportMutex.lock();
	refField.copyTo(retVal); // prevent users to overwriting reference field
	exportMutex.unlock();
	return retVal;
}
cv::Mat robotFloor::getCostField() {
	cv::Mat retVal;
	exportMutex.lock();
	frameCost.copyTo(retVal); // prevent users from overwriting the cost table
	exportMutex.unlock();
	return retVal;
}
