//
// Created by kangyu on 23-3-24.
//
#include "Pinhole.h"

namespace ORB_SLAM3 {
long unsigned int GeometricCamera::nNextId = 0;

bool Pinhole::IsEqual(ORB_SLAM3::GeometricCamera *pCam) {
  if (pCam->GetType() != GeometricCamera::CAM_PINHOLE)
    return false;

  Pinhole *pPinholeCam = (Pinhole *)pCam;

  if (size() != pPinholeCam->size())
    return false;

  bool is_same_camera = true;
  for (size_t i = 0; i < size(); ++i) {
    if (abs(mvParameters[i] - pPinholeCam->getParameter(i)) > 1e-6) {
      is_same_camera = false;
      break;
    }
  }
  return is_same_camera;
}
cv::Mat Pinhole::toK() {
  cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2],
               0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
  return K;
}
} // namespace ORB_SLAM3