// 2016.12 by astraywu
// 从字幕文件中读入目标的运动轨迹，使用卡尔曼滤波器器进行跟踪并显示

#define _CRT_SECURE_NO_WARNINGS
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "KalmanFilter.h"

// 常用颜色
const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(230, 130, 255);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar LIGHT_BLUE(255, 255, 160);
const cv::Scalar LIGHT_GREEN(144, 238, 144);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar PURPLE(205, 0, 205);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar GRAY(190, 190, 190);

// 画出X字线
#define DRAW_CROSS(image, center, color, d)\
	do{\
		cv::line(image, cv::Point(center.x - d, center.y - d), \
			cv::Point(center.x + d, center.y + d), color, 4, CV_AA, 0); \
		cv::line(image, cv::Point(center.x - d, center.y + d), \
			cv::Point(center.x + d, center.y - d), color, 4, CV_AA, 0); \
	} while (0)

// 显示视频帧，30FPS，Q和Esc退出，空格暂停
#define SHOW_FRAME(winName, img) \
	do {\
		cv::imshow(winName, img);\
		char ch = cv::waitKey(33);\
		if (toupper(ch) == 'Q' || ch == 27)\
			exit(0);\
		else if (toupper(ch) == ' ')\
			cv::waitKey(0);\
	} while (0)


// 从文本中解析字幕，解析帧索引到vecFrameIndex和跟踪坐标到vecTrackPos
inline void getTrackPos(const std::string &posFilename, std::vector<cv::Rect> &vecTrackPos, std::vector<int> &vecFrameIndex)
{
  vecTrackPos.empty();
  vecFrameIndex.empty();
  std::ifstream fin(posFilename);
  std::cout << posFilename << std::endl;
  fin.is_open();
  std::string line;

  while (std::getline(fin, line)) {
    int frameIndex, width, height, x, y;
    sscanf_s(line.c_str(), "%d [%d x %d from (%d, %d)]", &frameIndex, &width, &height, &x, &y);

    cv::Rect track(x, y, width, height);
    vecTrackPos.push_back(track);
    vecFrameIndex.push_back(frameIndex);
  }
}

// 将运动轨迹缩放到一定尺寸，避免原图太大无法显示
inline void scaleTrackPos(std::vector<cv::Rect> &vecTrackPos, const int scale)
{
  for (auto &trackPos : vecTrackPos) {
    trackPos.width /= scale;
    trackPos.height /= scale;
    trackPos.x /= scale;
    trackPos.y /= scale;
  }
}

int main()
{
	const std::string posFilename = "C:\\3rdParty\\source\\Exercise\\res\\24.txt";
	const int scale = 4;
	std::vector<cv::Rect> vecTrackPos;	// 跟踪的轨迹
	std::vector<int> vecFrameIndex;
	getTrackPos(posFilename, vecTrackPos, vecFrameIndex);
	scaleTrackPos(vecTrackPos, 4);

	cv::Mat imgBG(540, 1024, CV_8UC3);
	cv::namedWindow("imgBG");
	cv::resizeWindow("imgBG", 1024, 540);
	wrj_cv::CKalmanFilter KF;

	cv::Rect initTrack = vecTrackPos[0];
	KF.Init(initTrack.x, initTrack.y, -5.0 / scale, 12.0 / scale);

	// 根据轨迹文件跟踪观测值
	// 绿色是测量值，蓝色是KF跟踪值
	for (auto it = vecTrackPos.cbegin() + 1; it != vecTrackPos.cend(); ++it) {
		cv::Point measurePt(it->x, it->y);
		cv::Mat predictMat;
		cv::Point2f filterPt(KF.OtherPredictAndCorrect(it->x, it->y));

		imgBG = cv::Scalar::all(0);
		DRAW_CROSS(imgBG, measurePt, GREEN, 3);
		DRAW_CROSS(imgBG, filterPt, BLUE, 3);
		SHOW_FRAME("bg", imgBG);
	}

	// 没有轨迹文件后，自动预测观测值
	// 蓝色是KF跟踪值
	for (;;) {
		cv::Point2f filterPt(KF.SelfPredictAndCorrect());
		imgBG = cv::Scalar::all(0);
		DRAW_CROSS(imgBG, filterPt, BLUE, 3);
		SHOW_FRAME("bg", imgBG);
	}

	return 0;
}
