//
// Created by kangyu on 23-6-6.
//

#ifndef ORB_SLAM3_LOCALMAPPING_H
#define ORB_SLAM3_LOCALMAPPING_H
#include "Atlas.h"
#include "System.h"
#include "Tracking.h"
#include <eigen3/Eigen/Core>
namespace ORB_SLAM3 {
class System;
class Atlas;
class Tracking;
class LoopClosing;
class KeyFrame;

class LocalMapping {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalMapping(System *pSys, Atlas *pAtlas, const float bMonocular,
               bool bInertial, const string &_strSeqName = std::string());

  void SetTracker(Tracking *pTracker);
  void SetLoopCloser(LoopClosing *pLoopCloser);
  // Main function
  void Run();

  // Thread Synch
  void RequestStop();
  void Release(); // 在程序停止时释放LocalMapping线程所占用的资源
  bool isStopped();

public:
  int mInitFr; // 表示初始化时需要使用的帧的编号

  int mnMatchesInliers; // 表示当前关键帧配对后的内点数量

  bool mbBadImu; // 表示当前帧是否由于 IMU 数据不良导致被丢弃。如果为
                 // true，说明当前帧质量不佳，需要被丢弃

  // not consider far points(clouds)
  float mThFarPoints;
  bool mbFarPoints;

protected:
  System *mpSystem;
  Atlas *mpAtlas;

  LoopClosing *mpLoopCloser;
  Tracking *mpTracker;

  // 保存关键帧的队列
  std::list<KeyFrame *> mlNewKeyFrames;

  // 用来表示LocalMapping线程是否已经完成其全部工作
  // 在LocalMapping线程开始处理后，它会不断地从Tracking、LoopClosing等其他线程中获取数据，完成地图的更新和维护。
  // 而当LocalMapping线程完成全部工作后，mbFinished就会被设置为true，表示LocalMapping线程已经结束。
  bool mbFinished;

  bool mbMonocular;
  bool mbInertial;

  bool mbStopped;
  bool mbStopRequested;
  bool mbAbortBA;
  std::mutex mMutexStop;
  std::mutex mMutexNewKFs;
  std::mutex mMutexFinish;

  int mNumLM;        // 表示系统中当前已经跟踪到的路标点数量
  int mNumKFCulling; // 表示在关键帧剔除时被剔除的帧数。在关键帧选择过程中，如果某些帧的质量不足以成为关键帧，则会被剔除，mNumKFCulling
                     // 就是记录这样的帧数量。

  float mTinit; // 表示初始时刻（即第一帧）的时间戳
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_LOCALMAPPING_H
