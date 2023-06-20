//
// Created by kangyu on 23-2-26.
//
#include "System.h"

namespace ORB_SLAM3 {
eLevel Verbose::th = eLevel::VERBOSITY_NORMAL;

System::System(const std::string &strVocFile,
               const std::string &strSettingsFile, eSensor sensor,
               bool bUseViewer, int initFr, const std::string &strSequence)
    : mSensor(sensor), mbShutDown(false), mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false) {
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

  if (mpLocalMapper->mThFarPoints != 0) {
    std::cout << "Discard points further than " << mpLocalMapper->mThFarPoints
              << " m from current camera" << std::endl;
    mpLocalMapper->mbFarPoints = true;
  } else {
    mpLocalMapper->mbFarPoints = false;
  }

  // Initialize the loop closing thread and launch
  mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary,
                                 mSensor != eSensor::MONOCULAR, activeLC);
  mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // Initialize the viewer thread and launch
  if (bUseViewer) {
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                          strSettingsFile, settings_.get());
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
    mpLoopCloser->mpViewer = mpViewer;
    mpViewer->both = mpFrameDrawer->both;
  }

  // Fix verbosity
  Verbose::SetTh(eLevel::VERBOSITY_QUIET);
}

float System::GetImageScale() { return mpTracker->GetImageScale(); }

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                                    const vector<IMU::Point> &vImuMeas,
                                    std::string filename) {
  /**
   * 在加锁后，使用if语句判断当前视觉SLAM系统是否已经被关闭（mbShutDown），
   * 如果已经关闭则直接返回一个单位SE3变换（Sophus::SE3f()），表示相机没有运动。
   * 这个判断主要是为了避免在系统关闭期间重复执行数据处理或更新操作，浪费系统资源。
   */
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbShutDown)
      return Sophus::SE3f();
  }

  if (mSensor != eSensor::MONOCULAR && mSensor != eSensor::IMU_MONOCULAR) {
    std::cerr
        << "[System::TrackMonocular][ERROR] you called TrackMonocular but "
           "input sensor was not set to MONOCULAR nor MONOCULAR-Inertial."
        << std::endl;
    exit(-1);
  }

  cv::Mat imToFeed = im.clone();
  if (settings_ && settings_->needToResize()) {
    std::cerr << "[System::TrackMonocular][ERROR] input image need to resize."
              << std::endl;
    exit(-1);
  }

  // check mode change
  {
    unique_lock<mutex> lock(mMutexReset);
    // 激活定位模式
    if (mbActivateLocalizationMode) {
      // 请求停止LocalMapping线程
      mpLocalMapper->RequestStop();

      // wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      // 通知Tracker只进行跟踪而不进行地图构建
      mpTracker->InformOnlyTracking(true);
      // 标志位清零，表示激活定位模式的工作已经完成
      mbActivateLocalizationMode = false;
    }
    // 关闭定位模式
    if (mbDeactivateLocalizationMode) {
      // 通知Tracker恢复进行跟踪和地图构建的工作
      mpTracker->InformOnlyTracking(false);
      // 释放LocalMapping线程所占用的资源
      mpLocalMapper->Release();
      // 标志位清零，表示关闭定位模式的工作已经完成
      mbDeactivateLocalizationMode = false;
    }
  }
}
} // namespace ORB_SLAM3