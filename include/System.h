//
// Created by kangyu on 23-2-26.
//

#ifndef ORB_SLAM3_SYSTEM_H
#define ORB_SLAM3_SYSTEM_H

#include "Eigen/Core"
#include "sophus/se3.hpp"
#include <cstring>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <thread>

#include "Atlas.h"
#include "FrameDrawer.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ParamEnum.h"
#include "Setting.h"
#include "Tracking.h"
#include "Viewer.h"

namespace ORB_SLAM3 {
class Tracking;
class LocalMapping;
class Viewer;
class LocalMapping;
class LoopClosing;
class KeyFrameDatabase;

/**
 * 用于控制程序的输出信息的详细程度。通过设置eLevel的不同级别（例如Debug、Info、Warning、Error等），可以控制程序输出相应的信息。
 */
class Verbose {
public:
  static ORB_SLAM3::eLevel th;
  static void SetTh(ORB_SLAM3::eLevel _th) { th = _th; }
};

class System {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Initialize the SLAM system. It launches the Local Mapping , Loop
   * Closing and Viewer threads.
   * @param strVocFile path_to_vocabulary
   * @param strSettingsFile path_to_settings include Camera Parameters,ORB
   * Parameters and Viewer Parameters
   * @param sensor sensor type
   * @param bUseViewer use viewer or not
   * @param initFr 表示初始化时需要使用的帧的编号
   * @param strSequence
   */
  System(const std::string &strVocFile, const std::string &strSettingsFile,
         eSensor sensor, const bool bUseViewer = true, const int initFr = 0,
         const std::string &strSequence = std::string());

  // Disable copy constructor
  System(const System &) = delete;
  System &operator=(const System &) = delete;

  // Process the given monocular frame and optionally imu data
  // Input images: RGB(CV_8UC3) or grayscale(CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f
  TrackMonocular(const cv::Mat &im, const double &timestamp,
                 const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                 string filename = "");

  float GetImageScale();

private:
  eSensor mSensor; // Input sensor

  // ORB vocabulary used for place recognition and feature matching
  ORBVocabulary *mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop detection)
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints
  // Map* mpMap
  Atlas *mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  Tracking *mpTracker;

  // Local Mapper. It managers the local map and perdorms local bundle
  // adjustment.
  LocalMapping *mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  LoopClosing *mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  Viewer *mpViewer;

  // 绘制当前帧和匹配地图点的可视化信息，将这些信息显示在屏幕上
  FrameDrawer *mpFrameDrawer;

  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping , Loop Closing , Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object
  std::thread *mptLocalMapping;
  std::thread *mptLoopClosing;
  std::thread *mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

  // Shutdown flag
  bool mbShutDown;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  //
  std::string mStrLoadAtlasFromFile;
  std::string mStrSaveAtlasToFile;

  std::string mStrVocabularyFilePath;

  std::unique_ptr<Setting> settings_;
};
} // namespace ORB_SLAM3
#endif // ORB_SLAM3_SYSTEM_H
