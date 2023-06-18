//
// Created by kangyu on 23-5-30.
//
#include "Atlas.h"
#include "Pinhole.h"
#include <iostream>

namespace ORB_SLAM3 {
Atlas::Atlas() { mpCurrentMap = static_cast<Map *>(NULL); }
Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid) {
  mpCurrentMap = static_cast<Map *>(NULL);
}
Atlas::~Atlas() {}

GeometricCamera *Atlas::AddCamera(ORB_SLAM3::GeometricCamera *pCam) {
  // check if the camera already exists
  bool bAlreadyInMap = false;
  int index_cam = -1;
  for (size_t i = 0; i < mvpCameras.size(); ++i) {
    GeometricCamera *pCam_i = mvpCameras[i];
    if (!pCam)
      std::cout << "Not pCam" << std::endl;
    if (!pCam_i)
      std::cout << "Not pCam_i" << std::endl;
    if (pCam->GetType() != pCam_i->GetType())
      continue;
    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      if (((Pinhole *)pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
      }
    } else {
      std::cerr << "Atlas::AddCamera Camera type is not exit!!!" << std::endl;
    }
  }

  if (bAlreadyInMap) {
    return mvpCameras[index_cam];
  } else {
    mvpCameras.emplace_back(pCam);
    return pCam;
  }
}

std::vector<GeometricCamera *> Atlas::GetAllCameras() { return mvpCameras; }
} // namespace ORB_SLAM3