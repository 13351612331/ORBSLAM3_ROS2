//
// Created by kangyu on 23-6-1.
//

#ifndef ORB_SLAM3_TRACKING_H
#define ORB_SLAM3_TRACKING_H

#include "Atlas.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "ParamEnum.h"
#include "Setting.h"
#include "System.h"
#include "Viewer.h"
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace ORB_SLAM3 {
class System;
class LocalMapping;
class LoopClosing;
class Viewer;
class Setting;
class Atlas;
class Tracking {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
           MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase *pKFDB,
           const std::string &strSettingPath, const eSensor sensor,
           Setting *settings, const std::string &_nameSeq = std::string());
  ~Tracking();

  void SetLocalMapper(LocalMapping *pLocalMapper);
  void SetLoopClosing(LoopClosing *pLoopClosing);
  void SetViewer(Viewer *pViewer);

  float GetImageScale();

  // Use this function if you have deactivated local mapping and you only want to localize the camera.
  void InformOnlyTracking(const bool &flag);

public:
  // Input sensor
  eSensor mSensor;

  // True if local mapping is deactivated and we are performing only localization
  bool mbOnlyTracking;

protected:
  void newParameterLoader(Setting *settings);

protected:
  LocalMapping *mpLocalMapper;
  LoopClosing *mpLoopClosing;

  // ORB
  ORBextractor *mpORBextractorLeft; // 用来在左图像上提取特征的
  ORBextractor *mpIniORBextractor; // 用来在初始帧图像上提取特征的

  // Atlas
  Atlas *mpAtlas;

  // Drawers
  Viewer *mpViewer;

  // Calibration matrix
  cv::Mat mK;
  Eigen::Matrix3f mK_;
  cv::Mat mDistCoef;
  float mImageScale;

  // New KeyFrame rules (according to fps)
  // mMinFrames和mMaxFrames分别表示两个阈值：
  // 当当前帧与上一个关键帧的帧数差超过mMinFrames时，就可以将当前帧作为一个新的关键帧；
  // 而当当前帧与上一个关键帧的帧数差超过mMaxFrames时，则必须将当前帧作为新的关键帧，无论其是否能够提供重要信息。
  // 这样可以避免过多或者过少的关键帧，从而保证系统的效率和鲁棒性。
  int mMinFrames;
  int mMaxFrames;

  // Color order (true RGB , false BGR , ignored if grayscale)
  bool mbRGB;

  int mnNumDataset;

  GeometricCamera *mpCamera;

  int initID, lastID;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_TRACKING_H
