//
// Created by kangyu on 23-6-6.
//
#include "LocalMapping.h"

namespace ORB_SLAM3 {
LocalMapping::LocalMapping(ORB_SLAM3::System *pSys, ORB_SLAM3::Atlas *pAtlas,
                           const float bMonocular, bool bInertial,
                           const std::string &_strSeqName)
    : mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial),
      mpAtlas(pAtlas) {
  mnMatchesInliers = 0;

  mbBadImu = false;

  mTinit = 0.f;

  mNumLM = 0;
  mNumKFCulling = 0;
}

void LocalMapping::SetLoopCloser(ORB_SLAM3::LoopClosing *pLoopCloser) {
  mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(ORB_SLAM3::Tracking *pTracker) {
  mpTracker = pTracker;
}

void LocalMapping::Run() {
  std::cout << "Local Mapping is running" << std::endl;
}
} // namespace ORB_SLAM3