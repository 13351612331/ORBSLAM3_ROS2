//
// Created by kangyu on 23-6-1.
//
#include "Tracking.h"

namespace ORB_SLAM3 {
Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer, Atlas *pAtlas,
                   KeyFrameDatabase *pKFDB, const std::string &strSettingPath,
                   const ORB_SLAM3::eSensor sensor, Setting *settings,
                   const std::string &_nameSeq) {}

Tracking::~Tracking() {}
} // namespace ORB_SLAM3