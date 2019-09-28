#include "opencv2/core/core.hpp"

class Homography
{
public:
	Homography();
	~Homography();

	cv::Mat Run(cv::Mat& getImg);

private:
	static void onMouse(int event, int x, int y, int, void*);
	static cv::Point2f roi4point[4];
	static int roiIndex;
	static bool oksign;

	void PointOrderbyConner(cv::Point2f* inPoints);
};
