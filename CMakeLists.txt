cmake_minimum_required(VERSION 3.2)
project(ORB_SLAM3)

find_package(OpenCV 4.4)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif ()
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

include_directories(include
        Thirdparty
        ${OpenCV_INCLUDE_DIRS})
file(GLOB_RECURSE SLAM_LIBS_RESOURCES src/*.cpp)
add_library(SLAM_LIBS SHARED ${SLAM_LIBS_RESOURCES})

set(DBow2_LIB ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
set(ALL_LIBS SLAM_LIBS ${OpenCV_LIBS} ${DBow2_LIB})

add_executable(mono_tum node/Monocular/mono_tum.cpp)
target_link_libraries(mono_tum ${ALL_LIBS})