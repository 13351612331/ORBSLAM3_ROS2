//
// Created by kangyu on 23-6-6.
//
#include "LocalMapping.h"

namespace ORB_SLAM3 {
LocalMapping::LocalMapping(ORB_SLAM3::System *pSys, ORB_SLAM3::Atlas *pAtlas,
                           const float bMonocular, bool bInertial,
                           const std::string &_strSeqName)
    : mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial),
      mpAtlas(pAtlas), mbStopRequested(false), mbAbortBA(false),
      mbFinished(true), mbStopped(false) {
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
  mbFinished = false;
  std::cout << "Local Mapping is running" << std::endl;
}

void LocalMapping::RequestStop() {
  unique_lock<mutex> lock(mMutexStop);
  mbStopRequested = true;
  unique_lock<mutex> lock2(mMutexNewKFs);
  mbAbortBA = true;
}

bool LocalMapping::isStopped() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopRequested;
}

void LocalMapping::Release() {
  unique_lock<mutex> lock(mMutexStop);
  unique_lock<mutex> lock2(mMutexFinish);
  if (mbFinished)
    return;
  mbStopped = false;
  mbStopRequested = false;
  for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(),
                                  lend = mlNewKeyFrames.end();
       lit != lend; lit++)
    delete *lit;
  mlNewKeyFrames.clear();

  std::cout << "[LocalMapping::Release][INFO] Local Mapping RELEASE" << std::endl;
}
} // namespace ORB_SLAM3