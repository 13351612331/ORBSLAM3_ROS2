//
// Created by kangyu on 23-2-26.
//

#ifndef ORB_SLAM3_SYSTEM_H
#define ORB_SLAM3_SYSTEM_H

#include <cstring>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "ParamEnum.h"
#include "Setting.h"

namespace ORB_SLAM3 {
class System {
public:
public:
  /**
   * @brief Initialize the SLAM system. It launches the Local Mapping , Loop
   * Closing and Viewer threads.
   * @param strVocFile path_to_vocabulary
   * @param strSettingsFile path_to_settings include Camera Parameters,ORB
   * Parameters and Viewer Parameters
   * @param sensor sensor type
   * @param bUseViewer use viewer or not
   * @param initFr
   * @param strSequence
   */
  System(const std::string &strVocFile, const std::string &strSettingsFile,
         eSensor sensor, bool bUseViewer = true, int initFr = 0,
         const std::string &strSequence = std::string());

  // Disable copy constructor
  System(const System &) = delete;
  System &operator=(const System &) = delete;

private:
  eSensor mSensor; // Input sensor
  std::unique_ptr<Setting> settings_;
};
} // namespace ORB_SLAM3
#endif // ORB_SLAM3_SYSTEM_H