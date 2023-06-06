//
// Created by kangyu on 23-6-6.
//

#ifndef ORB_SLAM3_LOCALMAPPING_H
#define ORB_SLAM3_LOCALMAPPING_H
#include "Atlas.h"
#include "System.h"
#include <eigen3/Eigen/Core>
namespace ORB_SLAM3 {
class System;
class Atlas;

class LocalMapping {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LocalMapping(System *pSys, Atlas *pAtlas, const float bMonocular,
               bool bInertial, const string &_strSeqName = std::string());

  // Main function
  void Run();

public:
  int mInitFr; // 表示初始化时需要使用的帧的编号

  int mnMatchesInliers; // 表示当前关键帧配对后的内点数量

  bool mbBadImu; // 表示当前帧是否由于 IMU 数据不良导致被丢弃。如果为
                 // true，说明当前帧质量不佳，需要被丢弃

  // not consider far points(clouds)
  float mThFarPoints;

protected:
  System *mpSystem;
  Atlas *mpAtlas;

  bool mbMonocular;
  bool mbInertial;

  int mNumLM;        // 表示系统中当前已经跟踪到的路标点数量
  int mNumKFCulling; // 表示在关键帧剔除时被剔除的帧数。在关键帧选择过程中，如果某些帧的质量不足以成为关键帧，则会被剔除，mNumKFCulling
                     // 就是记录这样的帧数量。

  float mTinit; // 表示初始时刻（即第一帧）的时间戳
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_LOCALMAPPING_H
