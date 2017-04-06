// 2016.12 by astraywu
// calculate time
#ifndef UTILS_TIMER_H__
#define UTILS_TIMER_H__

#include <ctime>
#include <string>

#ifdef _WIN64
#include <Windows.h>

namespace wrj {
class Timer {
public:
  explicit Timer::Timer(const std::string event);
  explicit Timer::Timer(const char *event);
  ~Timer();
  inline void Start();          // ������ʱ������δֹͣǰ����������������ͳ�ƴ�������ӡ��Ϣ
  inline void Stop();           // ֹͣ��ʱ������δ����ǰֹͣ�����ӡ��Ϣ
  inline void Reset();          // ��Ҫ��ֹͣ��������ʱ�䣬���һ����Ϣ
  inline bool Report();         // ��ӡ��Ϣ����ֹͣǰ����ӡ
  inline bool StopAndReport();  // ֹͣ����ӡ��Ϣ
  inline double TimeInMilliSeconds();   // ��ȡͳ��ʱ�䣬�Ժ���Ϊ��λ

private:
  const std::string event_;
  bool is_started_;
	LARGE_INTEGER time_start_;
	LARGE_INTEGER time_freq_;
	double time_accumulate_;
  unsigned int num_starts_;
};
} // namespace wrj

namespace wrj {
Timer::Timer(const char *event) : event_(event), is_started_(false), time_accumulate_(0), num_starts_(0) { }

Timer::Timer(const std::string event) : event_(event), is_started_(false), time_accumulate_(0), num_starts_(0) { }

Timer::~Timer() {
	if (is_started_)
		printf("CmTimer '%s' is started and is being destroyed.\n", event_.c_str());
}

void Timer::Start() {
	if (is_started_) {
		printf("CmTimer '%s' is already started. Nothing done.\n", event_.c_str());
		return;
	}
	is_started_ = true;
	num_starts_++;
	QueryPerformanceFrequency(&time_freq_);
	QueryPerformanceCounter(&time_start_);
}

void Timer::Stop() {
	if (!is_started_) {
		printf("CmTimer '%s' is started. Nothing done\n", event_.c_str());
		return;
	}
	LARGE_INTEGER time_end;
	QueryPerformanceCounter(&time_end);
	double freq = (double)time_freq_.QuadPart;
	time_accumulate_ += (time_end.QuadPart - time_start_.QuadPart) * 1000.0 / freq;
	is_started_ = false;
}

void Timer::Reset() {
	if (is_started_) {
		printf("CmTimer '%s'is started during reset request.\n Only reset cumulative time.\n", event_.c_str());
		return;
	}
	time_accumulate_ = 0;
	num_starts_ = 0;
}

bool Timer::Report() {
	if (is_started_) {
		printf("CmTimer '%s' is started.\n Cannot provide a time report.", event_.c_str());
		return false;
	}

	double timeUsed = TimeInMilliSeconds();
	printf("[%s] CumuTime: %gms, #run: %d, AvgTime: %gms\n", event_.c_str(), timeUsed, num_starts_, timeUsed / num_starts_);
	return true;
}

double Timer::TimeInMilliSeconds() {
	if (is_started_) {
		printf("CmTimer '%s' is started. Nothing done\n", event_.c_str());
		return 0;
	}
	return time_accumulate_;
}

bool Timer::StopAndReport() {
	Stop();
	return Report();
}
} // namespace wrj

#else
// LINUX
// the Class Timer hasn't been tested in Linux!
#include <sys/time.h>

namespace wrj {
class Timer {
public:
	explicit Timer::Timer(const std::string event);
	explicit Timer::Timer(const char *event);
	~Timer();
	inline void Start();          // ������ʱ������δֹͣǰ����������������ͳ�ƴ�������ӡ��Ϣ
	inline void Stop();           // ֹͣ��ʱ������δ����ǰֹͣ�����ӡ��Ϣ
	inline void Reset();          // ��Ҫ��ֹͣ��������ʱ�䣬���һ����Ϣ
	inline bool Report();         // ��ӡ��Ϣ����ֹͣǰ����ӡ
	inline bool StopAndReport();  // ֹͣ����ӡ��Ϣ
	inline double TimeInMilliSeconds();   // ��ȡͳ��ʱ�䣬�Ժ���Ϊ��λ

private:
	const std::string event_;
	bool is_started_;
	struct timeval time_start_;
	double time_accumulate_;
	unsigned int num_starts_;
};
} // namespace wrj

namespace wrj {
Timer::Timer(const char *event) : event_(event), is_started_(false), time_accumulate_(0), num_starts_(0) { }

Timer::Timer(const std::string event) : event_(event), is_started_(false), time_accumulate_(0), num_starts_(0) { }

Timer::~Timer() {
	if (is_started_)
		printf("CmTimer '%s' is started and is being destroyed.\n", event_.c_str());
}

void Timer::Start() {
	if (is_started_) {
		printf("CmTimer '%s' is already started. Nothing done.\n", event_.c_str());
		return;
	}
	is_started_ = true;
	num_starts_++;
	gettimeofday(&time_begin, NULL);
}

void Timer::Stop() {
	if (!is_started_) {
		printf("CmTimer '%s' is started. Nothing done\n", event_.c_str());
		return;
	}
	gettimeofday(&time_end, NULL);
	struct timeval time_end;
	double time_used = static_cast<double>((time_end.tv_sec - time_start_.tv_sec) * 1000 + (time_end.tv_usec - time_start_.tv_usec) / 1000.0);
	time_accumulate_ += time_used;
	is_started_ = false;
}

void Timer::Reset() {
	if (is_started_) {
		printf("CmTimer '%s'is started during reset request.\n Only reset cumulative time.\n", event_.c_str());
		return;
	}
	time_accumulate_ = 0;
	num_starts_ = 0;
}

bool Timer::Report() {
	if (is_started_) {
		printf("CmTimer '%s' is started.\n Cannot provide a time report.", event_.c_str());
		return false;
	}

	double timeUsed = TimeInMilliSeconds();
	printf("[%s] CumuTime: %gms, #run: %d, AvgTime: %gms\n", event_.c_str(), timeUsed, num_starts_, timeUsed / num_starts_);
	return true;
}

double Timer::TimeInMilliSeconds() {
	if (is_started_) {
		printf("CmTimer '%s' is started. Nothing done\n", event_.c_str());
		return 0;
	}
	return time_accumulate_;
}

bool Timer::StopAndReport() {
	Stop();
	return Report();
}
} // namespace wrj
#endif // _WIN64

#endif // UTILS_TIMER_H__