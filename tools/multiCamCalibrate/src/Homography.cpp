

#include <string>  
#include <stdio.h>  
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Homography.h"

using namespace std;
using namespace cv;

cv::Point2f Homography::roi4point[4]; // TODO initialize?
int Homography::roiIndex = 0;
bool Homography::oksign = false;

Homography::Homography()
{
}

Homography::~Homography()
{
}

cv::Mat Homography::Run(cv::Mat& getImg)
{
	Mat RoiImg;

	//window
	namedWindow("set roi by 4 points", 0);

	//mouse callback
	setMouseCallback("set roi by 4 points", onMouse, 0);

	//point selection until 4 points setting
	while (1)
	{
		if (oksign == true) //right button click
			break;

		//draw point
		RoiImg = getImg.clone();
		for (int i = 0; i< roiIndex; ++i)
			circle(RoiImg, roi4point[i], 5, CV_RGB(255, 0, 255), 5);
		imshow("set roi by 4 points", RoiImg);

		waitKey(10);
	}

	printf("points ordered by LT, RT, RB, LB \n");
	PointOrderbyConner(roi4point);
	for (auto i = 0; i < 4; ++i)
	{
		printf("[%d] (%.2lf, %.2lf) \n", i, roi4point[i].x, roi4point[i].y);
	}

	//drwaring
	RoiImg = getImg.clone();
	string TestStr[4] = { "LT","RT","RB","LB" };
	putText(RoiImg, TestStr[0].c_str(), roi4point[0], CV_FONT_NORMAL, 1, Scalar(0, 0, 255), 3);
	circle(RoiImg, roi4point[0], 3, CV_RGB(0, 0, 255));
	int i;
	for (i = 1; i< roiIndex; ++i)
	{
		line(RoiImg, roi4point[i - 1], roi4point[i], CV_RGB(255, 0, 0), 1);
		circle(RoiImg, roi4point[i], 1, CV_RGB(0, 0, 255), 3);
		putText(RoiImg, TestStr[i].c_str(), roi4point[i], CV_FONT_NORMAL, 1, Scalar(0, 0, 255), 3);
	}

	line(RoiImg, roi4point[0], roi4point[i - 1], CV_RGB(255, 0, 0), 1);
	imshow("set roi by 4 points2", RoiImg);

	//prepare to get homography matrix
	vector< Point2f> P1; //clicked positions
	vector< Point2f> P2(4); //user setting positions
	for (int point = 0; point< 4; ++point)
		P1.push_back(roi4point[point]);

	//user setting position
	P2[0].x = 0; P2[0].y = 0;
	P2[1].x = 300; P2[1].y = 0;
	P2[2].x = 300; P2[2].y = 300;
	P2[3].x = 0; P2[3].y = 300;

	//get homography
	Mat H = findHomography(P1, P2);

	///////////////////////////
	//calculation confirm
	cout << "h" << endl << H << endl;
	cout << "size rows and cols " << H.rows << " " << H.cols << endl;

	Mat A(3, 4, CV_64F); //3xN, P1
	Mat B(3, 4, CV_64F); //3xN, P2
						 //B = H*A  (P2 = h(P1))

	for (int number = 0; number< 4; ++number)
	{
		A.at< double>(0, number) = P1[number].x;
		A.at< double>(1, number) = P1[number].y;
		A.at< double>(2, number) = 1;


		B.at< double>(0, number) = P2[number].x;
		B.at< double>(1, number) = P2[number].y;
		B.at< double>(2, number) = 1;
	}

	cout << "a" << endl << A << endl;
	cout << "b" << endl << B << endl;
	Mat ha = H*A;

	for (int number = 0; number< 4; ++number)
	{
		ha.at< double>(0, number) /= ha.at< double>(2, number);
		ha.at< double>(1, number) /= ha.at< double>(2, number);
		ha.at< double>(2, number) /= ha.at< double>(2, number);
	}

	cout << "HA" << endl << ha << endl;

	return H;
}

void Homography::PointOrderbyConner(Point2f* inPoints)
{
	vector< pair< float, float> > s_point;
	for (int i = 0; i< 4; ++i)
		s_point.push_back(make_pair(inPoints[i].x, inPoints[i].y));

	//sort
	sort(s_point.begin(), s_point.end(), [](const pair< float, float>& A, const pair< float, float>& B) { return A.second < B.second; });

	if (s_point[0].first < s_point[1].first)
	{
		inPoints[0].x = s_point[0].first;
		inPoints[0].y = s_point[0].second;

		inPoints[1].x = s_point[1].first;
		inPoints[1].y = s_point[1].second;

	}
	else {
		inPoints[0].x = s_point[1].first;
		inPoints[0].y = s_point[1].second;

		inPoints[1].x = s_point[0].first;
		inPoints[1].y = s_point[0].second;
	}

	if (s_point[2].first > s_point[3].first)
	{
		inPoints[2].x = s_point[2].first;
		inPoints[2].y = s_point[2].second;

		inPoints[3].x = s_point[3].first;
		inPoints[3].y = s_point[3].second;

	}
	else {
		inPoints[2].x = s_point[3].first;
		inPoints[2].y = s_point[3].second;

		inPoints[3].x = s_point[2].first;
		inPoints[3].y = s_point[2].second;
	}
}

void Homography::onMouse(const int event, const int x, const int y, int, void*)
{
	if (event == CV_EVENT_LBUTTONDOWN && oksign == false)
	{
		//4 point select
		if (roiIndex >= 4)
		{
			roiIndex = 0;
			for (auto i = 0; i< 4; ++i)
				roi4point[i].x = roi4point[i].y = 0;
		}

		roi4point[roiIndex].x = x;
		roi4point[roiIndex].y = y;

		roiIndex++;
	}

	if (event == CV_EVENT_RBUTTONDOWN)
	{
		//set point.
		if (roiIndex == 4)
		{
			oksign = true;
			printf("Warping Start!!!\n");
		}
	}
}

