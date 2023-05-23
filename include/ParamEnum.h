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
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_PARAMENUM_H
