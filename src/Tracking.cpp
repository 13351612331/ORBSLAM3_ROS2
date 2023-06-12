//
// Created by kangyu on 23-6-1.
//
#include "Tracking.h"

namespace ORB_SLAM3 {
Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer, Atlas *pAtlas,
                   KeyFrameDatabase *pKFDB, const std::string &strSettingPath,
                   const ORB_SLAM3::eSensor sensor, Setting *settings,
                   const std::string &_nameSeq)
    : mpAtlas(pAtlas) {
  // Load camera parameters from settings file
  if (settings) {
    newParameterLoader(settings);
  }
}

Tracking::~Tracking() {}

void Tracking::SetLocalMapper(ORB_SLAM3::LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(ORB_SLAM3::LoopClosing *pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(ORB_SLAM3::Viewer *pViewer) { mpViewer = pViewer; }

float Tracking::GetImageScale() { return mImageScale; }

void Tracking::newParameterLoader(ORB_SLAM3::Setting *settings) {
  mpCamera = settings->camera1();
  mpCamera = mpAtlas->AddCamera(mpCamera);
}
} // namespace ORB_SLAM3