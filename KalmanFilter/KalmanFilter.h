// 2016.12 by astraywu
// 封装卡尔曼滤波类，包括更新和检测，默认线性匀速运动
// 现在只做基本功能上的封装，简化所有其他定义和初始化，只提供两个成员函数，预测后用观测值更新，预测后用预测值更新
#ifndef UTILS_KALMAN_FILTER_H__
#define UTILS_KALMAN_FILTER_H__

#include <opencv2/opencv.hpp>

namespace wrj_cv {
	class CKalmanFilter {
	public:
		CKalmanFilter();																				// 定义一个2维观测变量的卡尔曼滤波器
		void Init(float x, float y, float dx, float dy);        // 用位置与速度初始化卡尔曼滤波器
		cv::Point2f OtherPredictAndCorrect(float x, float y);		// 预测并用观测值更新，完成卡尔曼滤波一次循环
		cv::Point2f SelfPredictAndCorrect();										// 自身预测并更细

	private:
		cv::KalmanFilter KF;
		cv::Mat state_;									// 状态变量
		cv::Mat measurement_;						// 观测变量
		static const int kStateNnum = 4;		    // x, y, dx, dy，必须使用static const变量，因为const是对象生成前还没初始化，不能用于构造函数中的其他初始化
		static const int kMeasurementNum = 2;		// x, y
		const double kSigmaQ = 1;		// 过程噪声simga，构造过程造成协方差矩阵
		const double kSigmaR = 5;		// 观测噪声simga，构造观测噪声协方差矩阵
		const double kSigmaP = 5;		// 构造初始协方差误差矩阵
	};
}	// namespace wrj_cv
#endif // UTILS_KALMAN_FILTER_H__