/*
 * dewarp.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Jan Feitsma
 */

// #include "falconsCommonDirs.hpp"

#include "dewarp.hpp"
#include "cvmatio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>
#include <fstream>
#include <locale>
#include <dirent.h>

#include <QString>
#include <QDebug>


// calibration was done with a border, which we need to account for ... (TODO: get rid of this)
int const BORDER_SIZE = 120;
// std::string CALIBRATION_FOLDER = pathToDataRepo() + "/internal/vision/multiCam/calibration";
std::string CALIBRATION_FOLDER = "/home/robocup/falcons/data/internal/vision/multiCam/calibration";

Dewarper::Dewarper(int robotId, int cam, bool autoUpdate, double timestamp)
{
   // update calibration
   if (autoUpdate)
   {
      updateCalibrationFiles();
   }
   // load the appropriate file
   if (robotId > 0)
   {
      readBIN(selectCalibrationFile(robotId, cam, timestamp));
   }
}

Dewarper::~Dewarper()
{
}

bool Dewarper::rangeCheck(uint16_t xPixel, uint16_t yPixel) const
{
   bool ok = true;
   if (xPixel >= _pixelCols)
   {
      qDebug().noquote() << QString().asprintf("ERROR   provided xPixel (%d) out of bounds (%d,%d)", xPixel, 0, _pixelCols);
      ok = false;
   }
   if (yPixel >= _pixelRows)
   {
      qDebug().noquote() << QString().asprintf("ERROR   provided yPixel (%d) out of bounds (%d,%d)", yPixel, 0, _pixelRows);
      ok = false;
   }
   return ok;
}

bool Dewarper::transformFloor(uint16_t xPixel, uint16_t yPixel, int16_t &xField, int16_t &yField) const
{
   xField = 0;
   yField = 0;
   // range checking
   if (!rangeCheck(xPixel, yPixel)) return false;
   // transform the pixel
   xField = _floorLookupX.at<int16_t>(yPixel+BORDER_SIZE, xPixel);
   yField = _floorLookupY.at<int16_t>(yPixel+BORDER_SIZE, xPixel);
   // check calibration range
   if (xField == 0 && yField == 0) return false;
   return true;
}

bool Dewarper::transformFront(uint16_t xPixel, uint16_t yPixel, float &azimuth, float &elevation) const
{
   azimuth = 0.0;
   elevation = 0.0;
   // range checking
   if (!rangeCheck(xPixel, yPixel)) return false;
   // transform the pixel
   azimuth   = _frontLookupAz.at<float>(yPixel+BORDER_SIZE, xPixel);
   elevation = _frontLookupEl.at<float>(yPixel+BORDER_SIZE, xPixel);
   return true; // no limited calibration range for front view
}

void Dewarper::readBIN(std::string const &filename)
{
   std::ifstream ifs(filename, std::ios::binary);
   // header: image dimensions
   ifs.read((char*)&_pixelCols, sizeof(_pixelCols));
   ifs.read((char*)&_pixelRows, sizeof(_pixelRows));
   // qDebug().noquote() << QString().asprintf("_pixelCols=%d _pixelRows=%d", _pixelCols, _pixelRows);
   // see opticalCalibrator::save for the ordering of the maps
   bool success = readMatBinary(ifs, _floorLookupX);
   if (!success)
   {
      throw std::runtime_error("failed to read _floorLookupX as binary from file " + filename);
   }
   success = readMatBinary(ifs, _floorLookupY);
   if (!success)
   {
      throw std::runtime_error("failed to read _floorLookupY as binary from file " + filename);
   }
   success = readMatBinary(ifs, _K);
   if (!success)
   {
      throw std::runtime_error("failed to read _K as binary from file " + filename);
   }
   success = readMatBinary(ifs, _D);
   if (!success)
   {
      throw std::runtime_error("failed to read _D as binary from file " + filename);
   }
   success = readMatBinary(ifs, _Hf);
   if (!success)
   {
      throw std::runtime_error("failed to read _Hf as binary from file " + filename);
   }
   success = readMatBinary(ifs, _frontLookupAz);
   if (!success)
   {
      throw std::runtime_error("failed to read _frontLookupAz as binary from file " + filename);
   }
   success = readMatBinary(ifs, _frontLookupEl);
   if (!success)
   {
      throw std::runtime_error("failed to read _frontLookupEl as binary from file " + filename);
   }
   qDebug().noquote() << QString().asprintf("INFO    successfully read file %s", filename.c_str());
}

void Dewarper::updateCalibrationFiles()
{
   // just call a script to do the work
   std::string command = "updateDewarpCalibration"; // use -q for quiet
   if (system(command.c_str()) != 0)
   {
      qDebug().noquote() << QString().asprintf("ERROR   command failed: %s", command.c_str());
      exit(EXIT_FAILURE);
   }
}

double dateStrToTimestamp(std::string d)
{
   std::tm t = {};
   if (strptime(d.c_str(), "%Y%m%d", &t) != NULL)
   {
      return std::mktime(&t);
   }
   qDebug().noquote() << QString().asprintf("ERROR   dateStrToTimestamp(%s) failed", d.c_str());
   exit(EXIT_FAILURE);
   return 0;
}

std::string Dewarper::selectCalibrationFile(int robotId, int cam, double timestamp)
{
   std::string result = "";
   double timestampMax = timestamp;
   double timestampOfResult = 0.0;
   // find newest file with timestamp smaller than provided argument
   DIR *d;
   struct dirent *dir;
   d = opendir(CALIBRATION_FOLDER.c_str());
   if (!d)
   {
      qDebug().noquote() << QString().asprintf("ERROR   calibration folder not found: %s", CALIBRATION_FOLDER.c_str());
      exit(EXIT_FAILURE);
   }
   else
   {
      while ((dir = readdir(d)) != NULL)
      {
         // exclude "." and ".."
         if (strcmp(dir->d_name, ".") == 0 || strcmp(dir->d_name, "..") == 0)
         {
            continue;
         }
         // set file names
         std::string filenameBase = dir->d_name;
         std::string filenameFull = CALIBRATION_FOLDER + "/" + dir->d_name;
         // calculate timestamp of this file
         // make use of naming convention: first eight characters are yyyymmdd
         double t = dateStrToTimestamp(filenameBase.substr(0, 8));
         // check if this file matches with robotId and cam
         std::size_t found = filenameBase.find("_r" + std::to_string(robotId));
         if (found == std::string::npos) continue;
         found = filenameBase.find("_cam" + std::to_string(cam));
         if (found == std::string::npos) continue;
         // check timestamp and update result if applicable
         if ((t > timestampOfResult) && (t < timestampMax))
         {
            result = filenameFull;
            timestampOfResult = t;
         }
      }
      closedir(d);
   }
   // done, check result
   if (result.size() == 0)
   {
      qDebug().noquote() << QString().asprintf("ERROR   could not find a suitable calibration file for robot %d, cam %d", robotId, cam);
      exit(EXIT_FAILURE);
   }
   return result;
}

