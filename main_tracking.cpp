#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils.h"
#include "config.h"
#include "timer.h"
#include "utils_opencv.h"
#include "kcftracker.hpp"
#include "KalmanFilter.h"
#include "CMap.h"

// 无人机飞行高度比例尺因子，在从视频到地图的GPS时，需要用到
const float kUAVScale = 729.0f;

// 根据缩放比例scale缩放矩形框
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

// times是总共要训练的次数
void updateKCF(KCFTracker &tracker, int times, const cv::Mat &oriFrameImg, float angle, const cv::Rect &searchROIRect, const cv::Rect &reserveROIRect)
{
	//cv::imshow("T", oriFrameImg(searchROIRect));
	//cv::waitKey(0);
	times = 20;
	cv::Mat frame = oriFrameImg.clone();
	assert(angle >= 0.0f);
	float anglePerTrain = angle / times;	// 每一帧需要多少角度的
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
		// getRotationMatrix2D的参数中，角度要是度数，而不是弧度的！！
		// 注意rotateRect的角度，应该也是返回弧度的！
		cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, -((i + 1) * anglePerTrain), 1.0); // 要取反方向的角度，即逆时针旋转
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


		//float angle = calcAngle(V, cv::Point2f(0, 0), offsetInFrame);	// 这里的角度计算考虑到提前转弯
		//angle = fabs(angle);

		//float angle = calcAngle(offsetInFrame);
		//float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// 需要多少帧，由距离直接除以速度
		//float anglePerFrame = angle / rotateFramesNum;	// 每一帧需要多少角度的

		//cv::Point2f center(preRect.width / 2.0f, preRect.height / 2.0f);
		//center = cv::Point2f(searchROIRect.width / 2.0f, searchROIRect.height / 2.0f);

		//// 为便于操作与性能上提升，直接将searchROI扩大为1.41倍


		////cv::Rect searchROIRectScaled = scaleRect(searchROIRect, sqrt(2.0f));
		//cv::Mat rotMatPerFrame = cv::getRotationMatrix2D(center, anglePerFrame, 1.0); // 要取反方向的角度，即逆时针旋转
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


// !!! 这个函数不能移到common.h中，不知道哪里有错误
// 返回周围4个点的颜色，如果不一样，返回白色
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

// 把实际距离换算成无人机拍摄的图片中的像素点距离，这里的f是根据4k来计算的
cv::Vec2f convertDistIntoFrame(const cv::Vec2f &realVec, float high, float f)
{
	cv::Vec2f inFrameVec;
	float x = realVec[0];
	float y = realVec[1];
	float xx = x * f / high;
	inFrameVec[0] = realVec[0] * f / high;// 转换成毫米
	inFrameVec[1] = realVec[1] * f / high;

	inFrameVec[0] = inFrameVec[0] / 10 / 2.54 * 96;
	inFrameVec[1] = inFrameVec[1] / 10 / 2.54 * 96;
	return inFrameVec;
}

// 初始目标选框
static cv::Rect initRect;
// 视频帧
static cv::Mat frame;
// 初始化标记，是否准备开始初始化KCF，如果true，则将进入onMouse选定厨师跟踪目标
static bool readyInitKCF = true;

static const std::string frameWinName = "SHOW";

static void onMouse(int event, int x, int y, int flag, void *)
{
	static cv::Point prePoint;
	static cv::Point curPoint;
	static cv::Mat imgTmp;
	// 鼠标左键下
	if (event == CV_EVENT_LBUTTONDOWN) {
		imgTmp = frame.clone();
		prePoint = cv::Point(x, y);
		readyInitKCF = true;
	}	// 鼠标左键按下并移动
	else if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_FLAG_LBUTTON)) {
		imgTmp = frame.clone();	// 每次移动都要拷贝原图用于画图
		curPoint = cv::Point(x, y);
		cv::rectangle(imgTmp, prePoint, curPoint, RED, 1, 8);
		cv::imshow(frameWinName, imgTmp);
	}	// 松开左键，选框完毕
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

// 范围内进行KCF检测，仅在5点检测，左上，右下，右上，左下，中央
void KCFDetect(KCFTracker &tracker, const cv::Mat &frame, const cv::Rect &rect, float &max_max_response, cv::Rect &ret_rect)
{
	const int width = rect.width;
	const int height = rect.height;
	const int x = rect.x;
	const int y = rect.y;
	
	ret_rect = rect;
	for (int r = -1; r <= 1; ++r) {
		for (int c = -1; c <= 1; ++c) {
			//if (r * c == 0 || (r == 0 && c == 0)) continue;	// 只采样五个点
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

	// 初始化视频
	cv::VideoCapture cap(filename_video);
	WRJ_ASSERT(cap.isOpened());

	int frame_index = 1766;							// 初始第一帧位置
	cap.set(CV_CAP_PROP_POS_FRAMES, frame_index - 1);
	
	cv::namedWindow(frameWinName);
	cv::moveWindow(frameWinName, 0, 0);
	cv::setMouseCallback(frameWinName, onMouse);

	// 初始化KCF
	const double kPaddingTracker = 2.5;
	KCFTracker tracker(true, true, false, true, kPaddingTracker, true);
	//tracker.interp_factor = 0.02;

	cv::Rect reserveRect;	// 保存用于旋转的跟踪结果，一般是当前帧的前3帧左右，用于遮挡时KCF的更新（直接将目标旋转）
	cv::Rect reserveTargetRect;
	bool redetect = false;	// 是否需要用KCF在范围内检测，在标记地图中使用
	bool in_redetect_state = false;
	bool redetect_success = false;
	bool in_redetect_Kalman_state = false;
	bool redetect_success_Kalman = false;
	bool Kalman_detect_occlusion = false;

	cv::Mat reserveFrame;
	cv::Rect reserveROIRect;
	cv::Rect reserveSearchROIRect;
	cv::Rect redetectRect;
	cv::Mat lastFrame;	// 保存上一帧，用于运动检测

	// 初始化Klaman Filter
	wrj_cv::CKalmanFilter KF;
	bool KFInited = false;	// KF是否初始化过

	bool noKF = false;	// 配合KCF在标记地图中的范围内检测，此时控制Kalman Filter不启用

	// 初始化地图
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



	// 对于跟踪器，只有三种状态，无遮挡跟踪（并更新），有部分遮挡跟踪（模型下降）不更新，完全遮挡（跟踪效果无用）
	enum TRACKER_STATE {
		STATE_OK = 0,
		STATE_SOME_OCCULUSION = 1,
		STATE_FAIL = 2
	};

	const float kThresholdTrackerStopTracking = 0.35f;
	const float kThresholdTrackerRetrackingDitu = 0.23f;
	const float kThresholdTrackerRetrackingKalman = 0.4f;

	cv::Rect rect_tracker_cur_frame_detect;	// 当前帧利用tracker进行检测出来的结果
	cv::Rect rect_tracker_last_frame_detect;	//	上一帧用tracker检测的结果

	cv::Rect rect_detect;
	cv::Point2f KFPt;	// Kalman Filter预测出的结果

	cv::Scalar color = WHITE;	// 当前点在地图中的区域，目前仅仅标记了4个区域

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

			// 初始化速度
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
		/// 已经初始化，开始跟踪
			/*判断逻辑
			是否需要再检测
				是 进行再检测，并使用范围搜索KCF，当联合上下文和目标的max_response超过阈值时，说明再检测到了
				否 是否用Kalman Filter进行预测
					是 用Kalman Filter预测位置，并联合上下文和目标的max_response计算，超过阈值说明再检测到了
					否 说明没有遮挡，使用KCF进行跟踪，联合位置和阈值判断，是否需要更新KCF，目前相应的树木检测和直线检测还没有完整加入
						1. 无遮挡正确更新KCF
						2. 部分遮挡，不更新KCF
						3. 完全遮挡，判断是何种遮挡，进行下一步
			*/

		/// 使用KCF跟踪计算跟踪结果
		float tracker_max_response = 0.0f;	// tracker model
		float target_max_response = 0.0f;		// target model

		/// 当发生立交桥遮挡后，需要在范围内进行搜索
		// 先不进行地图检测
		in_redetect_state = false;
		noKF = false;
		if (in_redetect_state) {
			// 上一帧是地图遮挡状态，进行范围搜索

			rect_tracker_cur_frame_detect = rect_detect;
			tracker.setROI(rect_tracker_cur_frame_detect.x, rect_tracker_cur_frame_detect.y, frame);
			//tracker.setROI(rect_tracker_cur_frame_detect.x, rect_tracker_cur_frame_detect.y, frame);

			// 仅用于打印信息，没有逻辑上的使用，用于调整阈值
			float tracker_max_response_temp = -1.0f;
			float target_max_response_temp = -1.0f;
			cv::Rect maxcurRect;
			// 指定区域内搜索
			// TODO 加上taret model的
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
			// 打印最大检测值，用于调试阈值
			fprintf(stdout, "State: tracker_max_response_temp: %f, target_max_response_temp: %f\n", tracker_max_response_temp, target_max_response_temp);

			if (redetect_success) {
				fprintf(stdout, "State: retracking successfully.\n");
				// 成功再检测到目标后，重置再检测状态
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
			// 上一帧是Kalman遮挡状态，用预测值来进行KCF
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
			// 上一帧是无遮挡状态，本帧正常更新（原来的思路是，在本帧内检测，检测遮挡后不更新，但现在逻辑是放到后面去）
			WRJ_TIME(rect_tracker_cur_frame_detect = tracker.updateWithoutTrain(frame, tracker_max_response));	// 仅跟踪，不训练
			fprintf(stdout, "max_response: %f\n", tracker_max_response);

			if (tracker_max_response >= kThresholdTrackerRetrackingKalman) {
				tracker.updateTrain(frame);
			} else  if (tracker_max_response < kThresholdTrackerStopTracking) {
				in_redetect_Kalman_state = true;
			} else {
				// Stop update KCF
			}

			/// 提取上一帧的跟踪搜索框，并进行各种遮挡判断，一般是目标框 x 2.5
			cv::Rect cur_frame_search_ROI_rect = tracker._extracted_roi;
			cv::Mat cur_frame_search_ROI = frame(cur_frame_search_ROI_rect & cv::Rect(0, 0, frame.cols, frame.rows));	// KCF2.5倍的搜索范围图像，不超出原图范围
			cv::Mat cur_frame_target_ROI = frame(rect_tracker_cur_frame_detect & cv::Rect(0, 0, frame.cols, frame.rows));

			float dx = rect_tracker_cur_frame_detect.x - rect_tracker_last_frame_detect.x;
			float dy = rect_tracker_cur_frame_detect.y - rect_tracker_last_frame_detect.y;
			cv::Point2f V(dx, dy);

			// 用于调试
			KFPt = KF.OtherPredictAndCorrect(rect_tracker_last_frame_detect.x * 1.0f, rect_tracker_last_frame_detect.y * 1.0f);

			if (color != WHITE) {	// 白色是默认颜色
				if (color == RED) {
					if (target_max_response >= kThresholdTrackerRetrackingKalman) continue;
					in_redetect_state = true;
					cv::Point2f offset(20, -16);
					cv::Point2f offsetInFrame = convertDistIntoFrame(offset, vecHigh[frame_index / 25], kUAVScale);

					// 用于归一化，因为原图被缩小了2倍
					offsetInFrame.x /= 2.0f;
					offsetInFrame.y /= 2.0f;

					// 计算出更新后的位置，即预测的位置
					rect_detect.x = rect_tracker_cur_frame_detect.x + offsetInFrame.x;
					rect_detect.y = rect_tracker_cur_frame_detect.y + offsetInFrame.y;

					//curRect.x += offsetInFrame.x;
					//curRect.y += offsetInFrame.y;
					//redetectRect = curRect;

					float angle = calcAngle(V, cv::Point2f(0, 0) - offsetInFrame);	// 这里的角度计算考虑到提前转弯
					angle = fabs(angle);

					//float angle = calcAngle(offsetInFrame);
					float rotateFramesNum = sqrt(offsetInFrame.x * offsetInFrame.x + offsetInFrame.y * offsetInFrame.y) / sqrt(dx * dx + dy * dy);	// 需要多少帧，由距离直接除以速度
					float anglePerFrame = angle / rotateFramesNum;	// 每一帧需要多少角度的

																													// 以下来更新KCF
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
		// 在当初获取的地蓝图中，用红色标记出跟踪目标，使用if是因为前25帧还未获取地图
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
				fprintf(stderr, "Error: 该区域未被标记.\n");
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