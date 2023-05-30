//
// Created by kangyu on 23-5-30.
//

#ifndef ORB_SLAM3_ATLAS_H
#define ORB_SLAM3_ATLAS_H
#include "Map.h"
#include <eigen3/Eigen/Core>
namespace ORB_SLAM3 {
class Atlas {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Atlas();
  Atlas(int initKFid); // when its initialization the first map is created
  ~Atlas();

protected:
  Map *mpCurrentMap;

  unsigned long int mnLastInitKFidMap;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_ATLAS_H
