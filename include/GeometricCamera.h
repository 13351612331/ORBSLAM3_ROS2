//
// Created by kangyu on 23-3-24.
//

#ifndef ORB_SLAM3_GEOMETRICCAMERA_H
#define ORB_SLAM3_GEOMETRICCAMERA_H
#include <utility>
#include <vector>

namespace ORB_SLAM3 {
class GeometricCamera {
public:
  GeometricCamera() = default;
  explicit GeometricCamera(std::vector<float> _vParameters)
      : mvParameters(std::move(_vParameters)) {}
  ~GeometricCamera() = default;

public:
  const static unsigned int CAM_PINHOLE = 0;

  static long unsigned int nNextId;

protected:
  std::vector<float> mvParameters;

  unsigned int mnId;

  unsigned int mnType;
};
} // namespace ORB_SLAM3
#endif // ORB_SLAM3_GEOMETRICCAMERA_H
