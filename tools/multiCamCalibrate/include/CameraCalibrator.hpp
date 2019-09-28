#include "opencv2/core/core.hpp"

class CameraCalibrator
{
public:
	CameraCalibrator();
	~CameraCalibrator();

	void run(std::vector<cv::Mat>& image, cv::Matx33d& K, cv::Vec4d& D) const;

private:
	const int _widthChessboard = 9;
	const int _heightChessboard = 6;
	const float _squareSizeChessboard = 0.00375f;

	void getObjectPoints(const cv::Size patternSize, std::vector<std::vector<cv::Point3f>>& objectPoints) const;
	bool getImagePoints(cv::Mat& image, cv::Size& patternSize, std::vector<std::vector<cv::Point2f>>& imagePoints) const;
};
