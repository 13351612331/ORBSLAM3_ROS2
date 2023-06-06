//
// Created by kangyu on 23-6-1.
//

#ifndef ORB_SLAM3_MAPDRAWER_H
#define ORB_SLAM3_MAPDRAWER_H

#include <string>
#include <eigen3/Eigen/Core>
#include "Atlas.h"
#include "Setting.h"

namespace ORB_SLAM3{
/**
 * 作用是将地图（Map）中的点云、关键帧（KeyFrame）和边缘（MapPoint 和 KeyFrame 之间的连线）等信息可视化在一个窗口中，
 * 方便用户观察和分析 SLAM 系统的运行效果。
 * MapDrawer 类通过添加地图中各种元素的可视化信息，将地图中的点云、关键帧和边缘等信息绘制在屏幕上。
 */
class MapDrawer{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapDrawer(Atlas* pAtlas , const std::string &strSettingPath , Setting* setting);
};
}

#endif // ORB_SLAM3_MAPDRAWER_H
