#include <opencv2/opencv.hpp>
#include "CameraCalibrator.h"
#include "Homography.h"

void mouseEvent(int event, int x, int y, int flags, void* param)
{
	IplImage* img = (IplImage*)param;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		printf("%d,%d\n",
			x, y);
	}
}

int main()
{
	// NOTE: This commented code can be used to find pixel positions in the image
	//{
		//// Read image from file 
		//IplImage *img = cvLoadImage(pathToJpg + "chessBoard0_12.jpg", 1);

		////Create a window
		//cvNamedWindow("My Window", 1);

		////set the callback function for any mouse event
		//cvSetMouseCallback("My Window", mouseEvent, &img);

		////show the image
		//cvShowImage("My Window", img);

		//// Wait until user press some key
		//cvWaitKey(0);

		//return 0;

	//}
    std::string pathToJpg = "/home/robocup/falcons/data/internal/vision/multiCam/chessboard/";
    
	const auto img1 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:09.jpg");
	const auto img2 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:20.jpg");
	const auto img3 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:24.jpg");
	const auto img4 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:28.jpg");
	const auto img5 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:32.jpg");
	const auto img6 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:39.jpg");
	const auto img7 = cv::imread(pathToJpg + "grab0_2018-04-07_18:17:51.jpg");
	const auto img8 = cv::imread(pathToJpg + "grab0_2018-04-07_18:18:09.jpg");
	const auto img9 = cv::imread(pathToJpg + "grab0_2018-04-07_18:18:20.jpg");
	const auto img10 = cv::imread(pathToJpg + "grab0_2018-04-07_18:18:39.jpg");
	const auto img11 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:01.jpg");
	const auto img12 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:28.jpg");
	const auto img13 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:32.jpg");
	const auto img14 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:36.jpg");
	const auto img15 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:47.jpg");
	const auto img16 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:50.jpg");
	const auto img17 = cv::imread(pathToJpg + "grab0_2018-04-07_18:19:58.jpg");
	const auto img18 = cv::imread(pathToJpg + "grab0_2018-04-07_18:20:02.jpg");
	const auto img19 = cv::imread(pathToJpg + "grab0_2018-04-07_18:20:06.jpg");
	const auto img20 = cv::imread(pathToJpg + "grab0_2018-04-07_18:20:32.jpg");
	const auto img21 = cv::imread(pathToJpg + "grab0_2018-04-07_18:20:36.jpg");

	std::vector<cv::Mat> images
	{ 
		img1, img2, img3, img4, img5, img6, img7, img8, img9, img10, img11, img12, img13, img14, img15, img16, img17, img18, img19, img20, img21
	};

	// Find camera parameters
	CameraCalibrator cameraCalibrator;
	cv::Matx33d K;  cv::Vec4d D;
	cameraCalibrator.run(images, K, D);

	// Remove warping - dewarp
	const auto img = cv::imread(pathToJpg + "../chessBoard0_12.jpg");
	cv::Mat undistorted;
	cv::Mat map1, map2;
	const cv::Matx33d newCameraMatrix = K;
	cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), newCameraMatrix, img.size(), CV_32FC1, map1, map2);
	//cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), newCameraMatrix, img.size(), CV_16SC2, map1, map2); // NOTE: int16_t variant is also possible
	cv::remap(img, undistorted, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	namedWindow("undistoredNew", cv::WINDOW_FULLSCREEN);
	cv::imshow("undistoredNew", undistorted);

	// find homography matrix
	Homography h;
	const auto homographyMatrix = h.Run(undistorted);

	// Apply homography matrix for perspective transformation
	cv::Mat undistortedNew;
	warpPerspective(undistorted, undistortedNew, homographyMatrix, cv::Size(undistorted.cols, undistorted.rows));
	namedWindow("undistortedNew", cv::WINDOW_FULLSCREEN);
	cv::imshow("undistortedNew", undistortedNew);

	// Merge output maps from dewarping with homography
	cv::Mat warped_image_X, warped_image_Y;
	warpPerspective(map1, warped_image_X, homographyMatrix, cv::Size(undistorted.cols, undistorted.rows));
	warpPerspective(map2, warped_image_Y, homographyMatrix, cv::Size(undistorted.cols, undistorted.rows));

	// Verification --> whether one set of outputmaps can perform dewarp and perspective transform
	cv::remap(img, undistorted, warped_image_X, warped_image_Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	namedWindow("undistoredTestImage", cv::WINDOW_FULLSCREEN);
	cv::imshow("undistoredTestImage", undistorted);

	// NOTE: Write output maps to file for further investigation and debugging
	//std::ofstream myfileX;
	//myfileX.open("image.txt");

	//for (int i = 0; i < img.rows; ++i)
	//{
	//	for (int j = 0; j < img.cols; ++j)
	//	{
	//		const auto val = img.at<float>(i, j);
	//		myfileX << val << " ";
	//	}
	//	myfileX << "\n";
	//}
	//myfileX.close();

	//std::ofstream myfileX;
	//myfileX.open("warped_image_X_16bit3.txt");

	//for (int i = 0; i < warped_image_X.rows; ++i)
	//{
	//	for (int j = 0; j < warped_image_X.cols; ++j)
	//	{
	//		const auto val = warped_image_X.at<float>(i, j);
	//		myfileX << val << " ";
	//	}
	//	myfileX << "\n";
	//}
	//myfileX.close();

	//std::ofstream myfileY;
	//myfileY.open("warped_image_Y_16bit3.txt");

	//for (int i = 0; i < warped_image_Y.rows; ++i)
	//{
	//	for (int j = 0; j < warped_image_Y.cols; ++j)
	//	{
	//		const auto val = warped_image_Y.at<float>(i, j);
	//		myfileY << val << " ";
	//	}
	//	myfileY << "\n";
	//}
	//myfileY.close();

	cv::waitKey(0);
	return 0;
}
