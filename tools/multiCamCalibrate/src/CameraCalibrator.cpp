#include "CameraCalibrator.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <stdexcept>


CameraCalibrator::CameraCalibrator()
{
}

CameraCalibrator::~CameraCalibrator()
{
}

void CameraCalibrator::getObjectPoints(const cv::Size patternSize, std::vector<std::vector<cv::Point3f>>& objectPoints) const
{
	std::vector<cv::Point3f> knownBoardPositions;
	for (int i = 0; i < patternSize.height; ++i)
	{
		for (int j = 0; j < patternSize.width; ++j)
		{
			knownBoardPositions.push_back(cv::Point3f(j*_squareSizeChessboard, i*_squareSizeChessboard, 0.0f));
		}
	}
	if (knownBoardPositions.size() > 0)
		objectPoints.push_back(knownBoardPositions);
}

bool CameraCalibrator::getImagePoints(cv::Mat& image, cv::Size& patternSize, std::vector<std::vector<cv::Point2f>>& imagePoints) const
{
	std::vector<cv::Point2f> corners;
	const bool patternFound = cv::findChessboardCorners(image, cv::Size(_widthChessboard, _heightChessboard), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_NORMALIZE_IMAGE);
	if (patternFound)
	{
		patternSize.width = _widthChessboard;
		patternSize.height = _heightChessboard;
		imagePoints.push_back(corners);
	}
	return patternFound;
}

void CameraCalibrator::run(std::vector<cv::Mat>& image, cv::Matx33d& K, cv::Vec4d& D) const
{
	std::vector<std::vector<cv::Point2f>> imagePoints;
	std::vector<std::vector<cv::Point3f>> objectPoints;
	cv::Size patternSize;

	for (auto& it : image)
	{
		if (getImagePoints(it, patternSize, imagePoints))
		{
			getObjectPoints(patternSize, objectPoints);
		}
	}

	if (imagePoints.empty())
	{
		throw std::runtime_error("Image points are empty");
	}

	std::vector<cv::Vec3d> rvecs;
	std::vector<cv::Vec3d> tvecs;
	cv::fisheye::calibrate(
		objectPoints,
		imagePoints,
		image[0].size(),
		K,
		D,
		rvecs,
		tvecs,
		cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND,
		cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 20, 1e-6)
	);
}
