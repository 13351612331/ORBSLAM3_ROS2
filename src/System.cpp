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
    mStrLoadAtlasFromFile = settings_->atlasLoadFile();
    mStrSaveAtlasToFile = settings_->atlasSaveFile();

    std::cout << (settings_.get()) << std::endl;
  } else {
    // TODO
  }

  node = fsSettings["loopClosing"];
  bool activeLC = true; // 激活回环检测（Loop Closing）
  if (!node.empty()) {
    activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
  }

  mStrVocabularyFilePath = strVocFile;

  bool loadedAtlas = false;

  if (mStrLoadAtlasFromFile.empty()) {
    // Load ORB Vocabulary
    std::cout << std::endl
              << "Loading ORB Vocabulary. This could take a while..."
              << std::endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
      std::cerr << "Wrong path to vocabulary. " << std::endl;
      std::cerr << "Failed to open at: " << strVocFile << std::endl;
      exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Atlas
    std::cout << "Initialization of Atlas from scratch " << std::endl;
    mpAtlas = new Atlas(0);
  } else {
    // TODO
    std::cout << "mStrLoadAtlasFromFile is not empty TODO" << std::endl;
  }

  if (mSensor == eSensor::IMU_STEREO || mSensor == eSensor::IMU_MONOCULAR ||
      mSensor == eSensor::IMU_RGBD) {
    // TODO
    std::cerr << "Not yet implemented！！！" << std::endl;
  }

  // Create Drawers. These are used by the Viewer
  mpFrameDrawer = new FrameDrawer(mpAtlas);
  mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_.get());

  // Initialize the Tracking thread
  // (it will live in the main thread of execution, the one that called this
  // constructor)
  std::cout << "Seq. Name: " << strSequence
            << std::endl; // strSequence输入的图像序列文件名前缀
  mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                           mpAtlas, mpKeyFrameDatabase, strSettingsFile,
                           mSensor, settings_.get(), strSequence);

  // Initialize the local Mapping thread and launch
  mpLocalMapper = new LocalMapping(
      this, mpAtlas,
      mSensor == eSensor::MONOCULAR || mSensor == eSensor::IMU_MONOCULAR,
      mSensor == eSensor::IMU_MONOCULAR || mSensor == eSensor::IMU_STEREO ||
          mSensor == eSensor::IMU_RGBD,
      strSequence);
  mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
  mpLocalMapper->mInitFr = initFr;
  if (settings_) {
    mpLocalMapper->mThFarPoints = settings_->thFarPoints();
  } else {
    mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
  }
}

} // namespace ORB_SLAM3