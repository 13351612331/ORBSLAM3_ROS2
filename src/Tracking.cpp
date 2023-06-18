//
// Created by kangyu on 23-6-1.
//
#include "Tracking.h"

namespace ORB_SLAM3 {
Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer, Atlas *pAtlas,
                   KeyFrameDatabase *pKFDB, const std::string &strSettingPath,
                   const ORB_SLAM3::eSensor sensor, Setting *settings,
                   const std::string &_nameSeq)
    : mpAtlas(pAtlas), mSensor(sensor) {
  // Load camera parameters from settings file
  if (settings) {
    newParameterLoader(settings);
  } else {
    std::cout << "Tracking: Please Load setting files" << std::endl;
  }

  initID = 0;
  lastID = 0;
  mnNumDataset = 0;

  vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
  std::cout << "There are " << vpCams.size() << " camera in the atlas"
            << std::endl;
  for (const auto &pCam : vpCams) {
    std::cout << "Camera " << pCam->GetId();
    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      std::cout << " is pinhole" << std::endl;
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      std::cout << " is fisheye" << std::endl;
    } else {
      std::cout << " is unknown" << std::endl;
    }
  }
}

Tracking::~Tracking() {}

void Tracking::SetLocalMapper(ORB_SLAM3::LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(ORB_SLAM3::LoopClosing *pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(ORB_SLAM3::Viewer *pViewer) { mpViewer = pViewer; }

float Tracking::GetImageScale() { return mImageScale; }

void Tracking::newParameterLoader(ORB_SLAM3::Setting *settings) {
  mpCamera = settings->camera1();
  mpCamera = mpAtlas->AddCamera(mpCamera);

  if (settings->needToUndistort()) {
    mDistCoef = settings->camera1DistortionCoef();
  } else {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  }

  mImageScale = 1.0f;

  mK = cv::Mat::eye(3, 3, CV_32F);
  mK.at<float>(0, 0) = mpCamera->getParameter(0);
  mK.at<float>(1, 1) = mpCamera->getParameter(1);
  mK.at<float>(0, 2) = mpCamera->getParameter(2);
  mK.at<float>(1, 2) = mpCamera->getParameter(3);

  mK_.setIdentity();
  mK_(0, 0) = mpCamera->getParameter(0);
  mK_(1, 1) = mpCamera->getParameter(1);
  mK_(0, 2) = mpCamera->getParameter(2);
  mK_(1, 2) = mpCamera->getParameter(3);

  if ((mSensor == eSensor::STEREO || mSensor == eSensor::IMU_STEREO ||
       mSensor == eSensor::IMU_RGBD) &&
      settings->cameraType() == CameraType::KannalaBrandt) {
    std::cerr << "Tracking: camera type(STEREO/IMU_STEREO/IMU_RGBD) is not "
                 "implement!!!"
              << std::endl;
    exit(-1);
  }

  if (mSensor == eSensor::STEREO || mSensor == eSensor::RGBD ||
      mSensor == eSensor::IMU_STEREO || mSensor == eSensor::IMU_RGBD) {
    std::cerr << "Tracking: camera type(STEREO/RGBD/IMU_STEREO/IMU_RGBD) is "
                 "not implement!!!"
              << std::endl;
    exit(-1);
  }

  if (mSensor == eSensor::RGBD || mSensor == eSensor::IMU_RGBD) {
    std::cerr << "Tracking: camera type(RGBD/IMU_RGBD) is not implement!!!"
              << std::endl;
    exit(-1);
  }

  mMinFrames = 0;
  mMaxFrames = settings->fps();
  mbRGB = settings->rgb();

  // ORB parameters
  int nFeatures = settings->nFeatures();
  int nLevels = settings->nLevels();
  int fIniThFAST = settings->initThFAST();
  int fMinThFAST = settings->minThFAST();
  float fScaleFactor = settings->scaleFactor();

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                        fIniThFAST, fMinThFAST);

  if (mSensor == eSensor::STEREO || mSensor == eSensor::IMU_STEREO) {
    std::cerr << "Tracking: sensor type of STEREO or IMU_STEREO not exit!!!"
              << std::endl;
    exit(-1);
  }

  if (mSensor == eSensor::MONOCULAR || mSensor == eSensor::IMU_MONOCULAR) {
    mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels,
                                         fIniThFAST, fMinThFAST);
  }

  // IMU parameters
  std::cout << "Tracking: if you sensor type is IMU , please add imu parameters"
            << std::endl;
}
} // namespace ORB_SLAM3