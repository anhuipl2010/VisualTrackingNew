// 2016.12 by astraywu
// ��װ�������˲��࣬�������ºͼ�⣬Ĭ�����������˶�
// ����ֻ�����������ϵķ�װ����������������ͳ�ʼ����ֻ�ṩ������Ա������Ԥ����ù۲�ֵ���£�Ԥ�����Ԥ��ֵ����
#ifndef UTILS_KALMAN_FILTER_H__
#define UTILS_KALMAN_FILTER_H__

#include <opencv2/opencv.hpp>

namespace wrj_cv {
	class CKalmanFilter {
	public:
		CKalmanFilter();																				// ����һ��2ά�۲�����Ŀ������˲���
		void Init(float x, float y, float dx, float dy);        // ��λ�����ٶȳ�ʼ���������˲���
		cv::Point2f OtherPredictAndCorrect(float x, float y);		// Ԥ�Ⲣ�ù۲�ֵ���£���ɿ������˲�һ��ѭ��
		cv::Point2f SelfPredictAndCorrect();										// ����Ԥ�Ⲣ��ϸ

	private:
		cv::KalmanFilter KF;
		cv::Mat state_;									// ״̬����
		cv::Mat measurement_;						// �۲����
		static const int kStateNnum = 4;		    // x, y, dx, dy������ʹ��static const��������Ϊconst�Ƕ�������ǰ��û��ʼ�����������ڹ��캯���е�������ʼ��
		static const int kMeasurementNum = 2;		// x, y
		const double kSigmaQ = 1;		// ��������simga������������Э�������
		const double kSigmaR = 5;		// �۲�����simga������۲�����Э�������
		const double kSigmaP = 5;		// �����ʼЭ����������
	};
}	// namespace wrj_cv
#endif // UTILS_KALMAN_FILTER_H__