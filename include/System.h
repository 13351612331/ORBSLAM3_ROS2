//
// Created by kangyu on 23-2-26.
//

#ifndef ORB_SLAM3_SYSTEM_H
#define ORB_SLAM3_SYSTEM_H

#include <cstring>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <thread>

#include "Atlas.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ParamEnum.h"
#include "Setting.h"
#include "Tracking.h"

namespace ORB_SLAM3 {
class Tracking;
class LocalMapping;

class System {
public:
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

  // 绘制当前帧和匹配地图点的可视化信息，将这些信息显示在屏幕上
  FrameDrawer *mpFrameDrawer;

  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping , Loop Closing , Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object
  std::thread *mptLocalMapping;

  //
  std::string mStrLoadAtlasFromFile;
  std::string mStrSaveAtlasToFile;

  std::string mStrVocabularyFilePath;

  std::unique_ptr<Setting> settings_;
};
} // namespace ORB_SLAM3
#endif // ORB_SLAM3_SYSTEM_H
