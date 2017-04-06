#include "KalmanFilter.h"
#include <opencv2/opencv.hpp>

namespace wrj_cv {
	CKalmanFilter::CKalmanFilter() : KF(kStateNnum, kMeasurementNum, 0), state_(kStateNnum, 1, CV_32F), measurement_(kMeasurementNum, 1, CV_32F) {
		KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, \
			0, 1, 0, 1, \
			0, 0, 1, 0, \
			0, 0, 0, 1);
		KF.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, \
			0, 1, 0, 0);
		cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(kSigmaQ));
		cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(kSigmaR));
		cv::setIdentity(KF.errorCovPost, cv::Scalar::all(kSigmaP));
	}

	void CKalmanFilter::Init(float x, float y, float dx, float dy) {
		state_ = (cv::Mat_<float>(kStateNnum, 1) << x, y, dx, dy);
		measurement_ = (cv::Mat_<float>(kMeasurementNum, 1) << x, y);

		KF.statePost = state_;
	}

	cv::Point2f CKalmanFilter::OtherPredictAndCorrect(float x, float y) {
		cv::Mat KF_predict_state = KF.predict();
		measurement_ = (cv::Mat_<float>(kMeasurementNum, 1) << x, y);
		KF.correct(measurement_);
		cv::Mat KF_predict = KF.measurementMatrix * KF_predict_state;
    cv::Point2f p(KF_predict.at<float>(0, 0), KF_predict.at<float>(1, 0));
    return p;
	}

	cv::Point2f CKalmanFilter::SelfPredictAndCorrect() {
		cv::Mat KF_predict_state = KF.predict();
		measurement_ = KF.measurementMatrix * KF_predict_state;
		KF.correct(measurement_);
    cv::Point2f p(measurement_.at<float>(0, 0), measurement_.at<float>(1, 0));
    return p;
	}
}	// namespace wrj_cv
