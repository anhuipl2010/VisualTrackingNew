// 2016.12 by astraywu
// This is a header file of some useful MACRO and function.
#ifndef COMMON_H__
#define COMMON_H__
// C
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
// C++
#include <algorithm>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <vector>

#ifdef _WIN64
#include <windows.h>
#include <direct.h>
#else
#include <sys/time.h>
#endif

#ifdef WIN32
#define WRJ_EXPORT __declspec(dllexport)
#else
#define WRJ_EXPORT
#endif

// Implementation of assert function. #define MDEBUG to enable it.
#ifdef WRJ_DEBUG
#define WRJ_ASSERT(expr)	(void)((expr) || (my_assert(#expr, __FUNCTION__, __FILE__, __LINE__), 0))
inline void my_assert(const char *expr, const char *function, const char *file, int line)
{
  printf("Assertion failed. expr: %s, func: %s, file: %s, line: %d\n", expr, function, file, line);
  abort();
}
#else
#define WRJ_ASSERT
#endif // WRJ_DEBUG

// Implementation of calculation of time consumed. And it print the res.
#ifdef _WIN64
#define WRJ_TIME
//#define WRJ_TIME(func)\
//	do {\
//    LARGE_INTEGER time_begin;\
//    LARGE_INTEGER time_end;\
//	  LARGE_INTEGER time_freq;\
//	  QueryPerformanceFrequency(&time_freq);\
//		double freq = (double)time_freq.QuadPart;\
//		QueryPerformanceCounter(&time_begin);\
//		func;\
//		QueryPerformanceCounter(&time_end);\
//		double time_used = (time_end.QuadPart - time_begin.QuadPart) * 1000.0 / freq;\
//		printf("TIME: %s, %6.3fms\n", #func, time_used);\
//	} while (0)
#else
#define WRJ_TIMER(func) \
	do {\
	struct timeval time_bein;\
	struct timeval time_end;\
	gettimeofday(&time_begin, NULL);\
	func;\
	gettimeofday(&time_end, NULL);\
	double time_used = static_cast<double>((time_end.tv_sec - time_begin.tv_sec) * 1000 + (time_end.tv_usec - time_begin.tv_usec) / 1000.0);\
	printf("TIME: %s, %%6.3fms\n", #func, time_used);\
	} while (0)


#endif
// 常用别名
typedef std::vector<int> vecI;
typedef std::vector<float> vecF;
typedef std::vector<double> vecD;
typedef std::vector<std::string> vecS;
typedef const std::string CStr;
typedef const int CInt;
typedef const float CFloat;
typedef const double CDouble;

#define Malloc(type, n) ((type *)malloc(sizeof(type) * n))

const double EPS = 1e-7;

#endif /* COMMON_H__ */
