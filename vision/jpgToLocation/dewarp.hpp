/*
 * dewarp.hpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Jan Feitsma
 */


#ifndef DEWARP_HPP_
#define DEWARP_HPP_


#include <string>
#include <vector>
#include <tuple>
#include "opencv4/opencv2/core/core.hpp"


class Dewarper
{

public:
   // robotId and camera have to be provided to figure out the calibration file(s)
   // if flag is set, automatically newest calibration files will be downloaded / used
   // based on provided timestamp, which is default taken to be 'newest'
   Dewarper(int robotId, int cam, bool autoUpdate = true, double timestamp = 1e99);
   ~Dewarper();

   // init time
   void readBIN(std::string const &filename); // read from binary file

   // run time

   // 'floor' transformation is designed for localization (white pixels)
   // it might also be used for obstacles (since we can safely assume these are not flying)
   // it should NOT be used for balls, instead see 'front' transformation
   // input: pixel location
   // output: success boolean and RCS coordinates in mm if successful
   bool transformFloor(uint16_t xPixel, uint16_t yPixel, int16_t &xField, int16_t &yField) const;

   // 'front' transformation is designed for balls
   // it basically takes out the intrinsic lens warp, so correct angles can be calculated
   // even when balls are airborne (which is often)
   // but it also corrects for small mounting tilts
   // this might also be used for obstacles, although floor transformation might be better?
   // input: pixel location
   // output: success boolean and spherical angles in radians
   bool transformFront(uint16_t xPixel, uint16_t yPixel, float &azimuth, float &elevation) const;

   // (test) utilities
   bool rangeCheck(uint16_t xPixel, uint16_t yPixel) const;

   // provide lookup table pixel width and height to verify if these match with the camera width and height
   // WARNING cols are height and rows are width
   int getPixelWith() { return _pixelRows; }
   int getPixelHeight(){ return _pixelCols; }

private:
   int _pixelRows, _pixelCols; // size of lookup tables, allowed range for input pixels
   cv::Mat _floorLookupX, _floorLookupY; // lookup table for Floor transformation: pixel to int RCS offset (x,y) in mm
   cv::Mat _frontLookupAz, _frontLookupEl; // lookup table for Front transformation: pixel to float angles (az,el)

   // with these calibrated fisheye matrices K and D, we could inline
   // correct the distortion in the diagnostics viewers using cv::fisheye::undistortImage
   // but for that, we need to upgrade to OpenCV3 - TODO
   cv::Matx33d _K;
   cv::Vec4d _D;

   // floor perspective homography, stored in calibration file but not used
   cv::Mat _Hf;

   // helpers
   void updateCalibrationFiles();
   std::string selectCalibrationFile(int robotId, int cam, double timestamp);

};

#endif /* DEWARP_HPP_ */

