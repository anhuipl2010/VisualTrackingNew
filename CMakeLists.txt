cmake_minimum_required(VERSION 2.8)

find_package (OpenCV REQUIRED)
project (VisualTrackingNew)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include_directories (KCF)
include_directories (KalmanFilter)
include_directories (utils)
include_directories (utils_opencv)
include_directories (MAP)


aux_source_directory (KCF DIR_KCF)
aux_source_directory (KalmanFilter DIR_KF)
aux_source_directory (utils DIR_UTILS)
aux_source_directory (utils_opencv DIR_UTILS_OPENCV)
aux_source_directory (MAP DIR_MAP)



add_executable (TEST_TIMER test_timer.cpp ${DIR_UTILS})

add_executable (TEST_CONFIG test_config.cpp ${DIR_UTILS})

add_executable (TEST_UTILS test_utils.cpp ${DIR_UTILS})

add_executable (TEST_UTILS_OPENCV test_utils_opencv.cpp ${DIR_UTILS_OPENCV})
target_link_libraries (TEST_UTILS_OPENCV ${OpenCV_LIBS})

add_executable (TEST_KCF test_KCF.cpp ${DIR_KCF})
target_link_libraries (TEST_KCF ${OpenCV_LIBS})

add_executable (TEST_KF test_KF.cpp ${DIR_KF})
target_link_libraries (TEST_KF ${OpenCV_LIBS})

add_executable (Tracking main_tracking.cpp ${DIR_KCF} ${DIR_KF} ${DIR_UTILS} ${DIR_MAP})
target_link_libraries (Tracking ${OpenCV_LIBS})
