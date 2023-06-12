//
// Created by kangyu on 23-3-24.
//

#ifndef ORB_SLAM3_PINHOLE_H
#define ORB_SLAM3_PINHOLE_H

#include "GeometricCamera.h"
#include "TwoViewReconstruction.h"
#include <iostream>
#include <vector>

namespace ORB_SLAM3 {
class Pinhole : public GeometricCamera {
public:
  Pinhole(const std::vector<float> _vParameters)
      : GeometricCamera(_vParameters), tvr(nullptr) {
    assert(mvParameters.size() == 4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }

  ~Pinhole() {
    if (tvr) {
      delete tvr;
      tvr = nullptr;
    }
  }

  bool IsEqual(GeometricCamera* pCam);

private:
  TwoViewReconstruction *tvr;
};
} // namespace ORB_SLAM3
#endif // ORB_SLAM3_PINHOLE_H
