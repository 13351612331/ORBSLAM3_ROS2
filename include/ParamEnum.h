//
// Created by kangyu on 23-3-1.
//

#ifndef ORB_SLAM3_PARAMENUM_H
#define ORB_SLAM3_PARAMENUM_H

namespace ORB_SLAM3 {
// Input sensor
enum class eSensor {
  MONOCULAR = 0,
  STEREO = 1,
  RGBD = 2,
  IMU_MONOCULAR = 3,
  IMU_STEREO = 4,
  IMU_RGBD = 5
};
// Enum for the different camera types implemented
enum class CameraType { PinHole = 0, Rectified = 1, KannalaBrandt = 2 };

// Tracking states
enum class eTrackingState{
  SYSTEM_NOT_READY = -1,
  NO_IMAGE_YET = 0,
  NOT_INITIALIZED = 1,
  OK = 2,
  RECENTLY_LOST = 3,
  LOST = 4,
  OK_KLT = 5
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_PARAMENUM_H
