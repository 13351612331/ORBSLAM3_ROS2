//
// Created by kangyu on 23-5-30.
//

#ifndef ORB_SLAM3_ATLAS_H
#define ORB_SLAM3_ATLAS_H
#include "GeometricCamera.h"
#include "Map.h"
#include <eigen3/Eigen/Core>
namespace ORB_SLAM3 {
class GeometricCamera;
class Atlas {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Atlas();
  Atlas(int initKFid); // when its initialization the first map is created
  ~Atlas();

  /**
   * 将pCam添加到Atlas类的相机列表中，并返回相机对象的指针。
   * 在Atlas类中，通常会维护一个相机列表，用于存储所有的相机对象。
   * 通过调用AddCamera函数，我们可以向相机列表中动态添加新的相机对象，
   * 从而方便实现相机的管理和控制。
   * @param pCam 指向的相机对象
   * @return 返回相机对象的指针
   */
  GeometricCamera *AddCamera(GeometricCamera *pCam);

protected:
  Map *mpCurrentMap;

  unsigned long int mnLastInitKFidMap;

  std::vector<GeometricCamera *> mvpCameras; // 相机列表
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_ATLAS_H
