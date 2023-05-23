//
// Created by kangyu on 23-3-1.
//

#ifndef ORB_SLAM3_SETTING_H
#define ORB_SLAM3_SETTING_H

#include "GeometricCamera.h"
#include "ParamEnum.h"
#include "Pinhole.h"
#include <iostream>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3 {
class Setting {
public:
  /**
   * @brief constructor function
   * @param configFile config file include Camera Parameters,ORB Parameters and
   * Viewer Parameters
   * @param sensor sensor type
   */
  Setting(const std::string &configFile, eSensor sensor);

  // Disable copy constructor
  Setting(const Setting &) = delete;
  Setting &operator=(const Setting &) = delete;

private:
  void readCamera1(cv::FileStorage &fSettings);

  void readImageInfo(cv::FileStorage &fSettings);

private:
  template <typename T>
  T readParameter(cv::FileStorage &fSettings, const std::string &name,
                  bool &found, const bool required = true) {}

private:
  eSensor m_sensor;
  CameraType m_cameraType;

  std::unique_ptr<GeometricCamera> calibration1_, calibration2_;
  // GeometricCamera *calibration1_, *calibration2_;
  // GeometricCamera *originalCalib1_, *originalCalib2_;
  std::unique_ptr<GeometricCamera> originalCalib1_, originalCalib2_;
  std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

  // 表示是否需要对图像进行失真校正的标志位
  bool bNeedToUndistort_;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_SETTING_H
