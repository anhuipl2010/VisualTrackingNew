// 2016.12 by astraywu
// This is a utilities OpenCV, which contains some tools I may use frequently.
#ifndef UTILS_OPENCV_H__
#define UTILS_OPENCV_H__

#include <cstdlib>
#include <cmath>
#include <opencv2/opencv.hpp>
//#include <direct.h>


// ������ɫ
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


// ���ñ���
typedef std::vector<cv::Mat> vecM;
typedef const cv::Mat CMat;


// ��ʾͼƬ������waitKey
#define SHOW(img) \
	do { \
		cv::imshow(#img, img);\
	} while (0)


#ifndef NDEBUG_SHOW
#define D_SHOW(img) \
	do { \
		cv::imshow(#img, img); \
		cv::imwrite(#img + std::string(".jpg"), img); \
	} while (0)
#else
#define D_SHOW(img) \
	do { \
		;\
	} while (0)
#endif


// ��ʾͼƬ����waitKey
#define SHOW_WAIT(img) \
	do {\
		cv::imshow(#img, img);\
		cv::waitKey(0);\
	} while(0)


// ��ʾ��Ƶ֡��30FPS��Q�˳����ո���ͣ
#define SHOW_FRAME(winName, img) \
	do {\
		cv::imshow(winName, img);\
		char ch = cv::waitKey(33);\
		if (toupper(ch) == 'Q')\
			exit(0);\
		else if (toupper(ch) == ' ')\
			cv::waitKey(0);\
	} while (0)


// ����X����
#define DRAW_CROSS(image, center, color, d)\
	do{\
		cv::line(image, cv::Point(center.x - d, center.y - d), \
			cv::Point(center.x + d, center.y + d), color, 4, CV_AA, 0); \
		cv::line(image, cv::Point(center.x - d, center.y + d), \
			cv::Point(center.x + d, center.y - d), color, 4, CV_AA, 0); \
	} while (0)



// ģ��ֻ�ܷ���H�ļ����ˣ��������common.h�޷��ػ�

// ���������ŷ�Ͼ���
template<typename T, typename U>
inline double calcDist(const cv::Point_<T> &A, const cv::Point_<U> &B)
{
	double x = A.x - B.x;
	double y = A.y - B.y;
	return sqrt(x * x + y * y);
}

// �����P��ֱ��AB�ľ��룬ͨ��ʸ������õ�
template<typename T, typename U>
inline double calcDist(const cv::Point_<T> &P, const cv::Point_<U> &A, const cv::Point_<U> &B)
{
	double dAP = sqrt((P.x - A.x) * (P.x - A.x) + (P.y - A.y) * (P.y - A.y));
	double dAB = sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));

	double cross = (P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y);

	double cosTheta = cross / dAB / dAP;
	return dAP * sqrt(1 - cosTheta * cosTheta);
}

// ����������άʸ���ļн�
template<typename T, typename U>
inline double calcAngle(const cv::Point_<T> &A, const cv::Point_<U> &B)
{
	double dA = sqrt(A.x * A.x + A.y * A.y);
	double dB = sqrt(B.x * B.x + B.y * B.y);
	double dot = A.x * B.x + A.y * B.y;
	double cosTheta = dot / dA / dB;
	return acos(cosTheta);	
}

//// ���ı��н�����Ļ������֡������vecFrameIndex�͸������굽vecTrackPos
//inline void getTrackPos(const std::string &posFilename, std::vector<cv::Rect> &vecTrackPos, std::vector<int> &vecFrameIndex)
//{
//	assert(vecTrackPos.empty());
//	assert(vecFrameIndex.empty());
//	std::ifstream fin(posFilename);
//	std::cout << posFilename << std::endl;
//	assert(fin.is_open());
//	std::string line;
//
//	while (std::getline(fin, line)) {
//		int frameIndex, width, height, x, y;
//		sscanf(line.c_str(), "%d [%d x %d from (%d, %d)]", &frameIndex, &width, &height, &x, &y);
//
//		cv::Rect track(x, y, width, height);
//		vecTrackPos.push_back(track);
//		vecFrameIndex.push_back(frameIndex);
//	}
//}

// ���˶��켣���ŵ�һ���ߴ磬����ԭͼ̫���޷���ʾ
inline void scaleTrackPos(std::vector<cv::Rect> &vecTrackPos, const int scale)
{
	for (size_t i = 0; i < vecTrackPos.size(); ++i) {
		cv::Rect &trackPos = vecTrackPos[i];
		trackPos.width /= scale;
		trackPos.height /= scale;
		trackPos.x /= scale;
		trackPos.y /= scale;
	}
}

#endif /* UTILS_OPENCV_H__ */