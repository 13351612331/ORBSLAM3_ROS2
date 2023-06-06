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
#include "ParamEnum.h"
#include "System.h"
#include <eigen3/Eigen/Core>

namespace ORB_SLAM3 {
class System;
class Tracking {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
           MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase *pKFDB,
           const std::string &strSettingPath, const eSensor sensor,
           Setting *settings, const std::string &_nameSeq = std::string());
  ~Tracking();
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_TRACKING_H
