//
// Created by kangyu on 23-6-11.
//

#ifndef ORB_SLAM3_VIEWER_H
#define ORB_SLAM3_VIEWER_H
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Setting.h"
#include "System.h"
#include "Tracking.h"

namespace ORB_SLAM3 {
class System;
class FrameDrawer;
class MapDrawer;
class Tracking;
class Setting;

class Viewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
         Tracking *pTracking, const string &strSettingPath, Setting *settings);

  void newParameterLoader(Setting *settings);

  // Main thread function. Draw points, keyframes , the current camera pose and
  // the last processed frame. Drawing is refreshed according to the camera fps.
  // We use Pangolin.
  void Run();

public:
  bool both;

private:
  System *mpSystem;
  FrameDrawer *mpFrameDrawer;
  MapDrawer *mpMapDrawer;
  Tracking *mpTracker;

  // 1/fps in ms
  double mT;
  float mImageWidth, mImageHeight;
  float mImageViewerScale;

  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

  bool mbStopTrack;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_VIEWER_H
