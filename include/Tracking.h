//
// Created by kangyu on 23-6-1.
//

#ifndef ORB_SLAM3_TRACKING_H
#define ORB_SLAM3_TRACKING_H

#include "Atlas.h"
#include "Frame.h"
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
#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3 {
class System;
class LocalMapping;
class LoopClosing;
class Viewer;
class Setting;
class Atlas;
class Frame;
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

  // Preprocess the input and call Track(). Extract features and performs stereo
  // matching.
  Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp,
                                  string filename);

  float GetImageScale();

  // Use this function if you have deactivated local mapping and you only want
  // to localize the camera.
  void InformOnlyTracking(const bool &flag);

  void Reset(bool bLocMap = false);
  void ResetActiveMap(bool bLocMap = false);

public:
  eTrackingState mState;

  // Input sensor
  eSensor mSensor;

  // Current Frame
  Frame mCurrentFrame;

  cv::Mat mImGray;

  // True if local mapping is deactivated and we are performing only
  // localization
  bool mbOnlyTracking;

protected:
  void newParameterLoader(Setting *settings);

protected:
  LocalMapping *mpLocalMapper;
  LoopClosing *mpLoopClosing;

  // ORB
  ORBextractor *mpORBextractorLeft; // 用来在左图像上提取特征的
  ORBextractor *mpIniORBextractor; // 用来在初始帧图像上提取特征的

  // Bow
  ORBVocabulary *mpORBVocabulary;

  // Atlas
  Atlas *mpAtlas;

  // Drawers
  Viewer *mpViewer;

  // Calibration matrix
  cv::Mat mK;
  Eigen::Matrix3f mK_;
  cv::Mat mDistCoef;
  float mbf;
  float mImageScale;

  // New KeyFrame rules (according to fps)
  // mMinFrames和mMaxFrames分别表示两个阈值：
  // 当当前帧与上一个关键帧的帧数差超过mMinFrames时，就可以将当前帧作为一个新的关键帧；
  // 而当当前帧与上一个关键帧的帧数差超过mMaxFrames时，则必须将当前帧作为新的关键帧，无论其是否能够提供重要信息。
  // 这样可以避免过多或者过少的关键帧，从而保证系统的效率和鲁棒性。
  int mMinFrames;
  int mMaxFrames;

  // Threshold close/far points
  // points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two
  // kerframes.
  /**
   * 是一个阈值，用于确定立体视觉或RGBD传感器中的点是被认为是“近”还是“远”点。
   * 根据这个阈值，可以将场景中的点分为两类：近距离可靠点和远距离点。
   * 对于被认为是近距离可靠点的点，它们只需要在单个关键帧中出现即可被插入（或存储）。
   * 这意味着从单个图像中只需要检测到这些点即可进行进一步处理，例如三维重建或深度估计。
   * 而对于被认为是远距离点的点，为了增加其可靠性，需要在两个关键帧中都能够匹配到这些点。
   * 这个额外的要求可以帮助过滤掉可能的错误匹配或不可靠的深度估计结果。
   * 具体来说，可以通过比较每个点的深度值与mThDepth来确定它是属于近距离可靠点还是远距离点。
   * 如果深度值小于等于mThDepth，则将其视为近距离可靠点；如果深度值大于mThDepth，则将其视为远距离点。
   */
  float mThDepth;

  // Color order (true RGB , false BGR , ignored if grayscale)
  bool mbRGB;

  int mnNumDataset;

  GeometricCamera *mpCamera;

  int initID, lastID;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_TRACKING_H
