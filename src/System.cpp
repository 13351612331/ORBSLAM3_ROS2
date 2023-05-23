//
// Created by kangyu on 23-2-26.
//
#include "System.h"

namespace ORB_SLAM3 {

System::System(const std::string &strVocFile,
               const std::string &strSettingsFile, eSensor sensor,
               bool bUseViewer, int initFr, const std::string &strSequence)
    : mSensor(sensor) {
  // Output welcome message
  std::cout << std::endl
            << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard "
               "Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, "
               "University of Zaragoza."
            << std::endl
            << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. "
               "Montiel and Juan D. Tardós, University of Zaragoza."
            << std::endl
            << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl
            << "This is free software, and you are welcome to redistribute it"
            << std::endl
            << "under certain conditions. See LICENSE.txt." << std::endl
            << std::endl;
  std::cout << "Input sensor was set to: ";
  switch (mSensor) {
  case eSensor::MONOCULAR:
    std::cout << "MONOCULAR" << std::endl;
    break;
  case eSensor::RGBD:
    std::cout << "RGBD" << std::endl;
    break;
  case eSensor::IMU_MONOCULAR:
    std::cout << "IMU_MONOCULAR" << std::endl;
    break;
  case eSensor::IMU_STEREO:
    std::cout << "IMU_STEREO" << std::endl;
    break;
  case eSensor::IMU_RGBD:
    std::cout << "IMU_RGBD" << std::endl;
    break;
  default:
    std::cerr << "ERROR TYPE OF SENSOR!!!" << std::endl;
    exit(-1);
  }
  // check setting file
  cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << strSettingsFile
              << std::endl;
    exit(-1);
  }
  cv::FileNode node = fsSettings["File.version"];
  if (!node.empty() && node.isString() && node.string() == "1.0") {
    settings_ = std::make_unique<Setting>(strSettingsFile, mSensor);
  }
}

} // namespace ORB_SLAM3