//
// Created by kangyu on 23-6-1.
//

#ifndef ORB_SLAM3_FRAMEDRAWER_H
#define ORB_SLAM3_FRAMEDRAWER_H

#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include "Atlas.h"
#include "ParamEnum.h"

namespace ORB_SLAM3{
/**
 * 主要负责绘制当前帧和匹配地图点的可视化信息，将这些信息显示在屏幕上。
 * 它是 ORBSLAM3 系统中一个较为基础的组件，
 * 提供了一些基本的绘图功能（例如画点、画线等），
 * 但不涉及到 3D 场景的渲染、相机姿态的设置、用户交互处理等高级功能
 */
class FrameDrawer{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrameDrawer(Atlas* pAtlas);

protected:

  // Info of the frame to be drawn
  cv::Mat mIm , mImRight; // 左右相机拍摄的图像数据，是双目视觉系统中进行配准和重建等操作的基础数据
  eTrackingState mState;
};
}

#endif // ORB_SLAM3_FRAMEDRAWER_H
