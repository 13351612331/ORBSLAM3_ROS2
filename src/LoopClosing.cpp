//
// Created by kangyu on 23-6-8.
//
#include "LoopClosing.h"

namespace ORB_SLAM3 {
LoopClosing::LoopClosing(ORB_SLAM3::Atlas *pAtlas,
                         ORB_SLAM3::KeyFrameDatabase *pDB,
                         ORB_SLAM3::ORBVocabulary *pVoc, const bool bFixScale,
                         const bool bActiveLC)
    : mpAtlas(pAtlas), mpKeyFrameDB(pDB), mpORBVocabulary(pVoc),
      mbFixScale(bFixScale), mbActiveLC(bActiveLC) {
  mnCovisibilityConsistencyTh = 3;
  mpLastCurrentKF = static_cast<KeyFrame *>(NULL);

  mstrFolderSubTraj = "SubTrajectories/";
  mnNumCorrection = 0;
  mnCorrectionGBA = 0;
}

void LoopClosing::SetTracker(ORB_SLAM3::Tracking *pTracker) {
  mpTracker = pTracker;
}

void LoopClosing::SetLocalMapper(ORB_SLAM3::LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void LoopClosing::Run() { std::cout << "LoopClosing is running" << std::endl; }
} // namespace ORB_SLAM3