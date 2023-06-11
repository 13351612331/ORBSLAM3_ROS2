//
// Created by kangyu on 23-6-8.
//

#ifndef ORB_SLAM3_LOOPCLOSING_H
#define ORB_SLAM3_LOOPCLOSING_H

#include "Atlas.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM3 {
class Tracking;
class LocalMapping;
class Viewer;

class LoopClosing {
public:
  LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc,
              const bool bFixScale, const bool bActiveLC);

  void SetTracker(Tracking *pTracker);

  void SetLocalMapper(LocalMapping *pLocalMapper);
  // Main function
  void Run();

public:
  Viewer *mpViewer;

protected:
  Atlas *mpAtlas;
  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBVocabulary;

  Tracking *mpTracker;
  LocalMapping *mpLocalMapper;

  // Loop detector parameters
  // 表示循环检测中的视差一致性阈值。
  // 视差一致性是指两个关键帧之间共同观察到的地图点数目越多，说明它们的视角越接近，因此对于循环检测来说，需要设置一个视差一致性阈值，来确定两个关键帧是否属于同一个回路
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  KeyFrame *mpLastCurrentKF;

  // Fix scale in the stereo/RGB-D case
  /**
   * 表示是否需要在双目或 RGB-D 模式下固定尺度。
   * 当 mbFixScale 为 true
   * 时，将通过地图扩展的方式来固定尺度，使得地图的尺度与真实世界中的尺度相匹配。
   * 在 Tracking 和 LocalMapping
   * 模块中都会启用尺度恢复算法，该算法会通过观测到的前后帧之间的运动估计，计算出当前帧的缩放因子，并对新建的地图点进行尺度纠正。通过这种方式，ORB-SLAM
   * 在保证位姿估计准确性的同时，可以尽可能地接近真实尺度
   */
  bool mbFixScale;

  // To (de)activate LC
  /**
   * 当 mbActiveLC 为 true 时，ORB-SLAM
   * 将启用循环闭合功能，即在场景中发现闭合回路并进行优化。反之，当 mbActiveLC
   * 为 false 时，ORB-SLAM不会执行循环闭合操作，只进行单目或双目
   * SLAM，这样可以提高系统的运行速度
   */
  bool mbActiveLC = true;

  // 表示子轨迹文件夹的路径
  // 当一个新的回路被检测到时，LoopClosing
  // 类会将所有候选关键帧组合成一个子轨迹，并将其保存到指定路径下的子轨迹文件夹中，以便后续的全局优化使用。
  std::string mstrFolderSubTraj;
  /**
   * 用于记录当前已经执行的全局优化次数。在 ORB-SLAM
   * 中，存在两种不同类型的全局优化方式：一种是基于位姿图优化的方式，一种是基于图优化的方式。前者仅优化相机的位姿信息，后者则同时优化相机的位姿信息和地图点的位置信息。在
   * LoopClosing 类中，mnNumCorrection
   * 记录的是已经执行的基于位姿图优化方式的全局优化次数，当其达到一定阈值时，就会触发图优化方式的全局优化。
   */
  int mnNumCorrection;
  /**
   * 是一个阈值，表示在图优化方式下执行的次数。当 mnNumCorrection
   * 达到该阈值时，LoopClosing
   * 类将执行一次完整的图优化，以进一步优化地图的精度和鲁棒性。
   */
  int mnCorrectionGBA;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_LOOPCLOSING_H
