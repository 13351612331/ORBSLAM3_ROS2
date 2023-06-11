//
// Created by kangyu on 23-6-11.
//
#include "Viewer.h"

namespace ORB_SLAM3 {
Viewer::Viewer(ORB_SLAM3::System *pSystem, ORB_SLAM3::FrameDrawer *pFrameDrawer,
               ORB_SLAM3::MapDrawer *pMapDrawer, ORB_SLAM3::Tracking *pTracking,
               const std::string &strSettingPath, ORB_SLAM3::Setting *settings)
    : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
      mpTracker(pTracking), both(false) {
  if (settings) {
    newParameterLoader(settings);
  } else {
    std::cerr << "Viewer: Please Read config file again!!!" << std::endl;
  }

  // 当mbStopTrack被设置为True时，系统将停止跟踪当前场景并等待新的观测数据，同时进入暂停状态
  mbStopTrack = false;
}

void Viewer::newParameterLoader(ORB_SLAM3::Setting *settings) {
  mImageViewerScale = 1.f;

  float fps = settings->fps();
  if (fps < 1) {
    fps = 30;
  }
  mT =
      1e3 /
      fps; // 根据帧率计算出一个合适的时间间隔，从而使系统在不同帧率下都能够保持相对稳定的运行速度

  cv::Size imSize = settings->newImSize();
  mImageHeight = imSize.height;
  mImageWidth = imSize.width;

  mImageViewerScale = settings->imageViewerScale();
  mViewpointX = settings->viewPointX();
  mViewpointY = settings->viewPointY();
  mViewpointZ = settings->viewPointZ();
  mViewpointF = settings->viewPointF();
}

void Viewer::Run() { std::cout << "Viewer is running" << std::endl; }
} // namespace ORB_SLAM3