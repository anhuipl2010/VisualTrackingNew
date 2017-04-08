#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils.h"
#include "config.h"
#include "timer.h"
#include "utils_opencv.h"
#include "kcftracker.hpp"
#include "KalmanFilter.h"
#include "CMap.h"

// ���˻����и߶ȱ��������ӣ��ڴ���Ƶ����ͼ��GPSʱ����Ҫ�õ�
const float kUAVScale = 729.0f;

// �������ű���scale���ž��ο�
cv::Rect scaleRect(const cv::Rect &rect, const float scale)
{
	cv::Point2f center(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f);
	cv::Rect rectScaled = rect;

	rectScaled.width *= scale;
	rectScaled.height *= scale;

	rectScaled.x = center.x - rectScaled.width / 2.0f;
	rectScaled.y = center.y - rectScaled.height / 2.0f;
	return rectScaled;
}

// times���ܹ�Ҫѵ���Ĵ���
void updateKCF(KCFTracker &tracker, int times, const cv::Mat &oriFrameImg, float angle, const cv::Rect &searchROIRect, const cv::Rect &reserveROIRect)
{
	//cv::imshow("T", oriFrameImg(searchROIRect));
	//cv::waitKey(0);
	times = 20;
	cv::Mat frame = oriFrameImg.clone();
	assert(angle >= 0.0f);
	float anglePerTrain = angle / times;	// ÿһ֡��Ҫ���ٽǶȵ�
	anglePerTrain = anglePerTrain * 180 / CV_PI;
	anglePerTrain += 3;
	//cv::imshow("NO", oriFrameImg(searchROIRect));
	cv::Rect searchROIRectScaled = scaleRect(searchROIRect, 2);
	//cv::imshow("NO2", oriFrameImg(searchROIRectScaled));
	//cv::waitKey(0);

	//cv::Point2f center(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);
	cv::Point2f center(searchROIRectScaled.width / 2.0f, searchROIRectScaled.height / 2.0f);

	cv::Mat tempImg = oriFrameImg.clone();


	//tracker.init(reserveROIRect, tempImg);



	for (int i = 0; i < times; ++i) {
		//if (i != times - 1)
		//	continue;

		//std::cout << i << std::endl;
		//tracker.updateTrain(frame);
		// getRotationMatrix2D�Ĳ����У��Ƕ�Ҫ�Ƕ����������ǻ��ȵģ���
		// ע��rotateRect�ĽǶȣ�Ӧ��Ҳ�Ƿ��ػ��ȵģ�
		cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, -((i + 1) * anglePerTrain), 1.0); // Ҫȡ������ĽǶȣ�����ʱ����ת
		//std::cout << rotMatPerFrame << std::endl;

		cv::Mat ROIImg = frame(searchROIRectScaled & cv::Rect(0, 0, frame.cols, frame.rows)).clone();
		SHOW(ROIImg);
		cv::Mat rotImg;
		cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, ROIImg.size());
		SHOW(rotImg);
		cv::Point2f newCenter(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
		cv::Mat ROIResImg;
		cv::getRectSubPix(rotImg, searchROIRect.size(), newCenter, ROIResImg);

		SHOW(ROIResImg);
		cv::Mat ROIOriImg = tempImg(searchROIRect);
		assert(ROIOriImg.size() == ROIResImg.size());
		//ROIOriImg = ROIResImg.clone();
		cv::addWeighted(ROIOriImg, 0, ROIResImg, 1, 0, ROIOriImg);
		/*ROIResImg.copyTo()*/
		//SHOW(ROIOriImg);
		//SHOW(frame);
		//cv::imshow("FUNC", tempImg);
		std::string filename = std::to_string(i) + "_rotate.jpg";
		cv::imwrite(filename, tempImg);
		//std::cout << tempImg.size << std::endl;

		//cv::waitKey(0);
		//tracker.setROI(center.x, center.y, frame);
		tracker.setROI(reserveROIRect.x, reserveROIRect.y, tempImg);
		float peak_value = 0.0f;
		tracker.updateWithoutTrain(tempImg, peak_value);
		std::cout << "PEAK " << peak_value << std::endl;
		tracker.updateTrain(tempImg);

		if (i == times - 1) {
			tracker.init(reserveROIRect, tempImg);
		}


		//float angle = calcAngle(V, cv::Point2f(0, 0), offsetInFrame);	// ����ĽǶȼ��㿼�ǵ���ǰת��
		//angle = fabs(angle);

		//float angle = calcAngle(offsetInFrame);
		//float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// ��Ҫ����֡���ɾ���ֱ�ӳ����ٶ�
		//float anglePerFrame = angle / rotateFramesNum;	// ÿһ֡��Ҫ���ٽǶȵ�

		//cv::Point2f center(preRect.width / 2.0f, preRect.height / 2.0f);
		//center = cv::Point2f(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);

		//// Ϊ���ڲ�����������������ֱ�ӽ�searchROI����Ϊ1.41��


		////cv::Rect searchROIRectScaled = scaleRect(searchROIRect, sqrt(2.0f));
		//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // Ҫȡ������ĽǶȣ�����ʱ����ת
		//cv::Mat ROIImg = frame(searchROIRectScaled & cv::Rect(0, 0, frame.cols, frame.rows));
		//cv::Mat rotImg;
		//cv::warpAffine(ROIImg, rotImg, rotMatPerFrame, ROIImg.size());
		//center = cv::Point2f(rotImg.cols / 2.0f, rotImg.rows / 2.0f);
		//cv::Mat resImg;
		//cv::getRectSubPix(rotImg, searchROIRect.size(), center, resImg);
		//std::cout << angle << std::endl;
		//std::cout << anglePerFrame << std::endl;
		//cv::imshow("RES", resImg);
		//cv::waitKey(0);
	}

}


// !!! ������������Ƶ�common.h�У���֪�������д���
// ������Χ4�������ɫ�������һ�������ذ�ɫ
static cv::Scalar roundingScalar(const cv::Mat &img, const cv::Point &pt)
{
	const cv::Scalar left(img.at<cv::Vec3b>(pt.y, pt.x - 1)[0], img.at<cv::Vec3b>(pt.y, pt.x - 1)[1], img.at<cv::Vec3b>(pt.y, pt.x - 1)[2]);
	const cv::Scalar right(img.at<cv::Vec3b>(pt.y, pt.x + 1)[0], img.at<cv::Vec3b>(pt.y, pt.x + 1)[1], img.at<cv::Vec3b>(pt.y, pt.x + 1)[2]);
	const cv::Scalar top(img.at<cv::Vec3b>(pt.y - 1, pt.x)[0], img.at<cv::Vec3b>(pt.y - 1, pt.x)[1], img.at<cv::Vec3b>(pt.y - 1, pt.x)[2]);
	const cv::Scalar bottom(img.at<cv::Vec3b>(pt.y + 1, pt.x)[0], img.at<cv::Vec3b>(pt.y + 1, pt.x)[1], img.at<cv::Vec3b>(pt.y + 1, pt.x)[2]);
	if ((left == right) && (left == top) && (left == bottom))
		return left;
	else
		return WHITE;
}

// ��ʵ�ʾ��뻻������˻������ͼƬ�е����ص���룬�����f�Ǹ���4k�������
cv::Vec2f convertDistIntoFrame(const cv::Vec2f &realVec, float high, float f)
{
	cv::Vec2f inFrameVec;
	float x = realVec[0];
	float y = realVec[1];
	float xx = x * f / high;
	inFrameVec[0] = realVec[0] * f / high;// ת���ɺ���
	inFrameVec[1] = realVec[1] * f / high;

	inFrameVec[0] = inFrameVec[0] / 10 / 2.54 * 96;
	inFrameVec[1] = inFrameVec[1] / 10 / 2.54 * 96;
	return inFrameVec;
}

// ��ʼĿ��ѡ��
static cv::Rect initRect;
// ��Ƶ֡
static cv::Mat frame;
// ��ʼ����ǣ��Ƿ�׼����ʼ��ʼ��KCF�����true���򽫽���onMouseѡ����ʦ����Ŀ��
static bool readyInitKCF = true;

static const std::string frameWinName = "SHOW";

static void onMouse(int event, int x, int y, int flag, void *)
{
	static cv::Point prePoint;
	static cv::Point curPoint;
	static cv::Mat imgTmp;
	// ��������
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = cv::Point(x, y);
		readyInitKCF = true;
	}	// ���������²��ƶ�
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// ÿ���ƶ���Ҫ����ԭͼ���ڻ�ͼ
		curPoint = cv::Point(x, y);
		cv::rectangle(imgTmp, prePoint, curPoint, RED, 1, 8);
		cv::imshow(frameWinName, imgTmp);
	}	// �ɿ������ѡ�����
	else if (event == CV_EVENT_LBUTTONUP) {
		initRect = cv::Rect(prePoint, curPoint);
		readyInitKCF = false;
		if (initRect.width < 2 || initRect.height < 2)
			readyInitKCF = true;
	} else if (event == CV_EVENT_LBUTTONDBLCLK) {
		std::cout << "Closed." << std::endl;
		exit(0);
	}
}

struct File_not_found {
	std::string filename_;
	File_not_found(const std::string filename = std::string()) : filename_(filename) { fprintf(stderr, "Error: Open file %s failed.\n", filename_.c_str()); }
};

#ifndef WRJ_ASSERT
#define WRJ_ASSERT(expr)	(void)((expr) || (wrj_assert(#expr, __FUNCTION__, __FILE__, __LINE__), 0))
inline void wrj_assert(const char *expr, const char *function, const char *file, int line)
{
	printf("Assertion failed. expr: %s, func: %s, file: %s, line: %d\n", expr, function, file, line);
	abort();
}
#endif // !WRJ_ASSERT

void parseGPSAndHighFromSRT(const std::string &SRTFilename, std::vector<cv::Point2d> &vecPos, std::vector<double> &vecHigh, const std::string &saveFilename)
{
	assert(vecPos.empty());
	//std::vector<cv::Point2d> vecPos;
	std::fstream fin(SRTFilename);
	assert(fin.is_open());
	std::string line;
	while (std::getline(fin, line)) {
		double latitude = 0;
		double longitude = 0;
		double high = 0;
		int satelliteNum = 0;
		if (line[0] == 'G' && line[1] == 'P' && line[2] == 'S') {
			sscanf(line.c_str(), "GPS(%lf,%lf, %d) Hb:%lf", &longitude, &latitude, &satelliteNum, &high);
			vecPos.push_back(cv::Point2d(longitude, latitude));
			vecHigh.push_back(high);
		}
	}

	if (!saveFilename.empty()) {
		std::ofstream fout(saveFilename);
		assert(fout.is_open());
		for (size_t i = 0; i < vecPos.size(); ++i) {
			char str[256];
			sprintf(str, "%4.6lf ,%4.6lf, %4.6lf", vecPos[i].x, vecPos[i].y, vecHigh[i]);
			fout << str << std::endl;
		}
		fout.close();
	}
}

// ��Χ�ڽ���KCF��⣬����5���⣬���ϣ����£����ϣ����£�����
void KCFDetect(KCFTracker &tracker, const cv::Mat &frame, const cv::Rect &rect, float &max_max_response, cv::Rect &ret_rect)
{
	const int width = rect.width;
	const int height = rect.height;
	const int x = rect.x;
	const int y = rect.y;
	
	ret_rect = rect;
	for (int r = -1; r <= 1; ++r) {
		for (int c = -1; c <= 1; ++c) {
			//if (r * c == 0 || (r == 0 && c == 0)) continue;	// ֻ���������
			//if (r * c != 0) continue;
			//if (r * c != 0) continue;
			float max_response = 0.0f;
			//tracker.setROI(x + r * width, y + c * height, frame);
			tracker.setROI(x + r * width / 2, y + c * height / 2, frame);
			cv::Rect rect_temp = tracker.updateWithoutTrain(frame, max_response);
			if (max_response > max_max_response) {
				max_max_response = max_response;
				ret_rect = rect_temp;
			}
		}
	}
}


int main(int argc, char *argv[])
{
	/// Read config
	const std::string kFilenameConfig = "../config.txt";
	wrj::Config config(kFilenameConfig);

	std::string filename_video;
	std::string filename_video_SRT;
	std::string filename_map;
	std::string filename_map_marked;
	std::string filename_map_coordinate;

	filename_video = config.Read("filename_video", filename_video);
	filename_video_SRT = config.Read("filename_video_SRT", filename_video_SRT);
	filename_map = config.Read("filename_map", filename_map);
	filename_map_marked = config.Read("filename_map_marked", filename_map_marked);
	filename_map_coordinate = config.Read("filename_map_coordinate", filename_map_coordinate);

	// ��ʼ����Ƶ
	cv::VideoCapture cap(filename_video);
	WRJ_ASSERT(cap.isOpened());

	int frame_index = 1766;							// ��ʼ��һ֡λ��
	cap.set(CV_CAP_PROP_POS_FRAMES, frame_index - 1);
	
	cv::namedWindow(frameWinName);
	cv::moveWindow(frameWinName, 0, 0);
	cv::setMouseCallback(frameWinName, onMouse);

	// ��ʼ��KCF
	const double kPaddingTracker = 2.5;
	KCFTracker tracker(true, true, false, true, kPaddingTracker, true);
	//tracker.interp_factor = 0.02;

	cv::Rect reserveRect;	// ����������ת�ĸ��ٽ����һ���ǵ�ǰ֡��ǰ3֡���ң������ڵ�ʱKCF�ĸ��£�ֱ�ӽ�Ŀ����ת��
	cv::Rect reserveTargetRect;
	bool redetect = false;	// �Ƿ���Ҫ��KCF�ڷ�Χ�ڼ�⣬�ڱ�ǵ�ͼ��ʹ��
	bool in_redetect_state = false;
	bool redetect_success = false;
	bool in_redetect_Kalman_state = false;
	bool redetect_success_Kalman = false;
	bool Kalman_detect_occlusion = false;

	cv::Mat reserveFrame;
	cv::Rect reserveROIRect;
	cv::Rect reserveSearchROIRect;
	cv::Rect redetectRect;
	cv::Mat lastFrame;	// ������һ֡�������˶����

	// ��ʼ��Klaman Filter
	wrj_cv::CKalmanFilter KF;
	bool KFInited = false;	// KF�Ƿ��ʼ����

	bool noKF = false;	// ���KCF�ڱ�ǵ�ͼ�еķ�Χ�ڼ�⣬��ʱ����Kalman Filter������

	// ��ʼ����ͼ
	wrj_cv::CMap m(filename_map, filename_map_coordinate, filename_map_marked);
	std::vector<cv::Point2d> vecGPS;
	std::vector<double> vecHigh;
	parseGPSAndHighFromSRT(filename_video_SRT, vecGPS, vecHigh, "");
	cv::Mat mapROI;
	cv::Mat mapMarkedROI;
	bool inRedArea = false;
	bool inBlueArea = false;
	bool inGreenArea = false;
	bool inBlackArea = false;
	const unsigned int kInRedArea = 0X01;
	const unsigned int kInBlueArea = 0X02;
	const unsigned int kInGreenArea = 0X04;
	const unsigned int kInBlackArea = 0X08;

	//cv::VideoWriter writer("res.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0f, cv::Size(2048, 1080));
	//assert(writer.isOpened());



	// ���ڸ�������ֻ������״̬�����ڵ����٣������£����в����ڵ����٣�ģ���½��������£���ȫ�ڵ�������Ч�����ã�
	enum TRACKER_STATE {
		STATE_OK = 0,
		STATE_SOME_OCCULUSION = 1,
		STATE_FAIL = 2
	};

	const float kThresholdTrackerStopTracking = 0.35f;
	const float kThresholdTrackerRetrackingDitu = 0.23f;
	const float kThresholdTrackerRetrackingKalman = 0.4f;

	cv::Rect rect_tracker_cur_frame_detect;	// ��ǰ֡����tracker���м������Ľ��
	cv::Rect rect_tracker_last_frame_detect;	//	��һ֡��tracker���Ľ��

	cv::Rect rect_detect;
	cv::Point2f KFPt;	// Kalman FilterԤ����Ľ��

	cv::Scalar color = WHITE;	// ��ǰ���ڵ�ͼ�е�����Ŀǰ���������4������

	cv::Rect temp_rect;

	wrj::Timer timer_main_loop("main_loop");

	for (;;) {
		/// Read frame from video
		WRJ_TIME(cap >> frame);
		++frame_index;
		fprintf(stdout, "%d\n", frame_index);
		WRJ_TIME(frame.copyTo(lastFrame));
		WRJ_ASSERT(frame.data);

		/// Init or reinit KCF
		if (readyInitKCF) {
			cv::imshow(frameWinName, frame);
			cv::waitKey(0);
			//cv::Point left_top(1340, 440);
			//cv::Point left_bottom(1340, 440 + 50);
			//cv::Point right_bottom(1340 + 50, 440 + 50);
			//cv::Point right_top(1340 + 50, 440);
			//initRect = cv::Rect(left_top, right_bottom);
			//readyInitKCF = false;

			tracker.init(initRect, frame);

			// Store the init frame to make rotate sample while occlusion
			reserveFrame = frame.clone();
			reserveROIRect = initRect;
			reserveSearchROIRect = tracker._extracted_roi;

			cv::rectangle(frame, initRect, RED, 1, 8);
			rect_tracker_cur_frame_detect = initRect;

			// ��ʼ���ٶ�
			KFInited = true;
			noKF = false;
			in_redetect_state = false;
			in_redetect_Kalman_state = false;
			redetect_success = false;
			redetect_success_Kalman = false;
			KF.Init(rect_tracker_cur_frame_detect.x, rect_tracker_cur_frame_detect.y, 0, 0);
			continue;
			//updateKCF(tracker, 180, reserveFrame, CV_PI, reserveSearchROIRect, reserveROIRect);
		}
		timer_main_loop.Reset();
		timer_main_loop.Start();
		/// �Ѿ���ʼ������ʼ����
			/*�ж��߼�
			�Ƿ���Ҫ�ټ��
				�� �����ټ�⣬��ʹ�÷�Χ����KCF�������������ĺ�Ŀ���max_response������ֵʱ��˵���ټ�⵽��
				�� �Ƿ���Kalman Filter����Ԥ��
					�� ��Kalman FilterԤ��λ�ã������������ĺ�Ŀ���max_response���㣬������ֵ˵���ټ�⵽��
					�� ˵��û���ڵ���ʹ��KCF���и��٣�����λ�ú���ֵ�жϣ��Ƿ���Ҫ����KCF��Ŀǰ��Ӧ����ľ����ֱ�߼�⻹û����������
						1. ���ڵ���ȷ����KCF
						2. �����ڵ���������KCF
						3. ��ȫ�ڵ����ж��Ǻ����ڵ���������һ��
			*/

		/// ʹ��KCF���ټ�����ٽ��
		float tracker_max_response = 0.0f;	// tracker model
		float target_max_response = 0.0f;		// target model

		/// �������������ڵ�����Ҫ�ڷ�Χ�ڽ�������
		// �Ȳ����е�ͼ���
		in_redetect_state = false;
		noKF = false;
		if (in_redetect_state) {
			// ��һ֡�ǵ�ͼ�ڵ�״̬�����з�Χ����

			rect_tracker_cur_frame_detect = rect_detect;
			tracker.setROI(rect_tracker_cur_frame_detect.x, rect_tracker_cur_frame_detect.y, frame);
			//tracker.setROI(rect_tracker_cur_frame_detect.x, rect_tracker_cur_frame_detect.y, frame);

			// �����ڴ�ӡ��Ϣ��û���߼��ϵ�ʹ�ã����ڵ�����ֵ
			float tracker_max_response_temp = -1.0f;
			float target_max_response_temp = -1.0f;
			cv::Rect maxcurRect;
			// ָ������������
			// TODO ����taret model��
			//bool retracking_success = false;
			cv::Rect tracker_detect_rect;
			cv::Rect target_detect_rect;
			for (int r = -2; r <= 2; ++r) {
				for (int c = 0; c < (10 - abs(r)) / 3; ++c) {
					if (redetect_success) {
						break;
					}
					// Set tracker and tracker_target ROI// TODO curRect
					tracker.setROI(rect_tracker_cur_frame_detect.x + 4 * c, rect_tracker_cur_frame_detect.y + 4 * r, frame);
					//tracker_target.setROI(rect_tracker_cur_frame_detect.x + 2 * c, rect_tracker_cur_frame_detect.y + 2 * r, frame);

					float tracker_response_temp = 0.0f;
					float target_response_temp = 0.0f;
					cv::Rect tracker_rect_temp = tracker.updateWithoutTrain(frame, tracker_response_temp);
					//cv::Rect target_rect_temp = tracker_target.updateWithoutTrain(frame, target_response_temp);

					if (tracker_response_temp > kThresholdTrackerRetrackingDitu/* && target_response_temp > kThresholdTargetRetracking*/) {
						redetect_success = true;
						tracker_detect_rect = tracker_rect_temp;
						//target_detect_rect = target_rect_temp;
					}

					tracker_max_response_temp = max(tracker_max_response_temp, tracker_response_temp);
					//target_max_response_temp = max(target_max_response_temp, target_response_temp);
				}
			}
			// ��ӡ�����ֵ�����ڵ�����ֵ
			fprintf(stdout, "State: tracker_max_response_temp: %f, target_max_response_temp: %f\n", tracker_max_response_temp, target_max_response_temp);

			if (redetect_success) {
				fprintf(stdout, "State: retracking successfully.\n");
				// �ɹ��ټ�⵽Ŀ��������ټ��״̬
				in_redetect_state = false;
				redetect_success = false;

				KFInited = true;
				tracker.setROI(tracker_detect_rect.x, tracker_detect_rect.y, frame);
				tracker.updateTrain(frame);
				//tracker_target.updateTrain(frame);
				noKF = true;
			} else {
				//continue;
			}
		} else if (in_redetect_Kalman_state && noKF == false) {
			// ��һ֡��Kalman�ڵ�״̬����Ԥ��ֵ������KCF
			KFPt = KF.SelfPredictAndCorrect();
			rect_tracker_cur_frame_detect = cv::Rect(KFPt.x, KFPt.y, initRect.width, initRect.height);
			
			float max_max_response = 0.0f;
			cv::Rect max_max_response_rect;
			KCFDetect(tracker, frame, rect_tracker_cur_frame_detect, max_max_response, max_max_response_rect);
			fprintf(stdout, "max_max_response_rect: %f\n", max_max_response);

			if (max_max_response >= kThresholdTrackerStopTracking) {
				tracker.setROI(max_max_response_rect.x, max_max_response_rect.y, frame);
				tracker.updateTrain(frame);
				tracker.updateTrain(frame);
				tracker.updateTrain(frame);
				tracker.updateTrain(frame);
				in_redetect_Kalman_state = false;
			} else {
				tracker.setROI(rect_tracker_cur_frame_detect.x, rect_tracker_cur_frame_detect.y, frame);
				float temp = 0.0f;
				tracker.updateWithoutTrain(frame, temp);

			}

		} else {
			// ��һ֡�����ڵ�״̬����֡�������£�ԭ����˼·�ǣ��ڱ�֡�ڼ�⣬����ڵ��󲻸��£��������߼��Ƿŵ�����ȥ��
			WRJ_TIME(rect_tracker_cur_frame_detect = tracker.updateWithoutTrain(frame, tracker_max_response));	// �����٣���ѵ��
			fprintf(stdout, "max_response: %f\n", tracker_max_response);

			if (tracker_max_response >= kThresholdTrackerRetrackingKalman) {
				tracker.updateTrain(frame);
			} else  if (tracker_max_response < kThresholdTrackerStopTracking) {
				in_redetect_Kalman_state = true;
			} else {
				// Stop update KCF
			}

			/// ��ȡ��һ֡�ĸ��������򣬲����и����ڵ��жϣ�һ����Ŀ��� x 2.5
			cv::Rect cur_frame_search_ROI_rect = tracker._extracted_roi;
			cv::Mat cur_frame_search_ROI = frame(cur_frame_search_ROI_rect & cv::Rect(0, 0, frame.cols, frame.rows));	// KCF2.5����������Χͼ�񣬲�����ԭͼ��Χ
			cv::Mat cur_frame_target_ROI = frame(rect_tracker_cur_frame_detect & cv::Rect(0, 0, frame.cols, frame.rows));

			float dx = rect_tracker_cur_frame_detect.x - rect_tracker_last_frame_detect.x;
			float dy = rect_tracker_cur_frame_detect.y - rect_tracker_last_frame_detect.y;
			cv::Point2f V(dx, dy);

			// ���ڵ���
			KFPt = KF.OtherPredictAndCorrect(rect_tracker_last_frame_detect.x * 1.0f, rect_tracker_last_frame_detect.y * 1.0f);

			if (color != WHITE) {	// ��ɫ��Ĭ����ɫ
				if (color == RED) {
					if (target_max_response >= kThresholdTrackerRetrackingKalman) continue;
					in_redetect_state = true;
					cv::Point2f offset(20, -16);
					cv::Point2f offsetInFrame = convertDistIntoFrame(offset, vecHigh[frame_index / 25], kUAVScale);

					// ���ڹ�һ������Ϊԭͼ����С��2��
					offsetInFrame.x /= 2.0f;
					offsetInFrame.y /= 2.0f;

					// ��������º��λ�ã���Ԥ���λ��
					rect_detect.x = rect_tracker_cur_frame_detect.x + offsetInFrame.x;
					rect_detect.y = rect_tracker_cur_frame_detect.y + offsetInFrame.y;

					//curRect.x += offsetInFrame.x;
					//curRect.y += offsetInFrame.y;
					//redetectRect = curRect;

					float angle = calcAngle(V, cv::Point2f(0, 0) - offsetInFrame);	// ����ĽǶȼ��㿼�ǵ���ǰת��
					angle = fabs(angle);

					//float angle = calcAngle(offsetInFrame);
					float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// ��Ҫ����֡���ɾ���ֱ�ӳ����ٶ�
					float anglePerFrame = angle / rotateFramesNum;	// ÿһ֡��Ҫ���ٽǶȵ�

																													// ����������KCF
					updateKCF(tracker, rotateFramesNum, reserveFrame, angle, reserveSearchROIRect, reserveROIRect);

				} else if (color == BLUE) {

				} else {

				}
			}
		}

		//////////////////////////////////////////////////////////////////////////
		if (((frame_index - 1) % 25) == 0 || mapROI.data == nullptr) {
			const int ksecondIndex = (frame_index / 25 + 1);
			m.calcFrame(vecGPS[ksecondIndex - 1], cv::Mat(cv::Size(4096, 2160), 0), vecHigh[ksecondIndex - 1], 729, 117);
			//m.drawMapROI(mapROI);
			m.getMapROI(mapROI, mapMarkedROI);
		}
		// �ڵ�����ȡ�ĵ���ͼ�У��ú�ɫ��ǳ�����Ŀ�꣬ʹ��if����Ϊǰ25֡��δ��ȡ��ͼ
		if (mapROI.data) {
			cv::Point2f targerPt(rect_tracker_cur_frame_detect.x + rect_tracker_cur_frame_detect.width / 2, rect_tracker_cur_frame_detect.y + rect_tracker_cur_frame_detect.height / 2);
			targerPt.x /= (frame.cols * 1.0f / mapROI.cols);
			targerPt.y /= (frame.rows * 1.0f / mapROI.rows);
			DRAW_CROSS(mapROI, targerPt, RED, 1);
			SHOW(mapROI);
			SHOW(mapMarkedROI);

			color = roundingScalar(mapMarkedROI, targerPt);

			inRedArea = inGreenArea = inBlueArea = inBlackArea = false;
			if (color == RED) {
				inRedArea = true;
			} else if (color == GREEN) {
				inGreenArea = true;
			} else if (color == BLUE) {
				inBlueArea = true;
			} else if (color == BLACK) {
				inBlackArea = true;
			} else {
				fprintf(stderr, "Error: ������δ�����.\n");
			}
		}

		//////////////////////////////////////////////////////////////////////////

		//std::cout << "END " << curRect << std::endl;
		rect_tracker_last_frame_detect = rect_tracker_cur_frame_detect;
		cv::rectangle(frame, rect_tracker_cur_frame_detect, GREEN);
		cv::rectangle(frame, tracker._extracted_roi, PINK);
		cv::rectangle(frame, temp_rect, BLUE);
		//cv::rectangle(frame, rect_, PINK);

		if (KFInited)
			cv::rectangle(frame, cv::Rect(KFPt.x, KFPt.y, rect_tracker_cur_frame_detect.width, rect_tracker_cur_frame_detect.height), YELLOW);
		cv::imshow(frameWinName, frame);

		timer_main_loop.StopAndReport();

		char ch = cv::waitKey(5);
		if (toupper(ch) == 'Q')
			break;
		else if (toupper(ch) == 'P')
			cv::waitKey(0);
		else
			continue;

		//writer << frame;
	}
	//writer.release();

	return 0;
}