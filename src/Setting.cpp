//
// Created by kangyu on 23-3-1.
//
#include "Setting.h"

namespace ORB_SLAM3 {
template <>
std::string Setting::readParameter<std::string>(cv::FileStorage &fSettings,
                                                const std::string &name,
                                                bool &found,
                                                const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      std::cerr << name << " required parameter does not exist, aborting..."
                << std::endl;
      exit(-1);
    } else {
      std::cerr << name << " optional parameter does not exist..." << std::endl;
      found = false;
      return std::string();
    }
  } else if (!node.isString()) {
    std::cerr << name << " parameter must be a string, aborting..."
              << std::endl;
    exit(-1);
  } else {
    found = true;
    return node.string();
  }
}

template <>
float Setting::readParameter<float>(cv::FileStorage &fSettings,
                                    const std::string &name, bool &found,
                                    const bool required) {
  cv::FileNode node = fSettings[name];

  if (node.empty()) {
    if (required) {
      std::cerr << name << " required parameter does not exist, aborting..."
                << std::endl;
      exit(-1);
    } else {
      std::cerr << name << " optional parameter does not exist..." << std::endl;
      found = false;
      return 0.0f;
    }
  } else if (!node.isReal()) {
    std::cerr << name << " parameter must be a real number , aborting..."
              << std::endl;
    exit(-1);
  } else {
    found = true;
    return node.real();
  }
}

template <>
int Setting::readParameter<int>(cv::FileStorage &fSettings,
                                const std::string &name, bool &found,
                                const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      std::cerr << name << "required parameter does not exits, aborting..."
                << std::endl;
      exit(-1);
    } else {
      std::cerr << name << "optional parameter does not exist..." << std::endl;
      found = false;
      return 0;
    }
  } else if (!node.isInt()) {
    std::cerr << name << " parameter must be an integer number , aborting..."
              << std::endl;
    exit(-1);
  } else {
    found = true;
    return node.operator int();
  }
}

Setting::Setting(const std::string &configFile, ORB_SLAM3::eSensor sensor)
    : bNeedToUndistort_(false) {
  m_sensor = sensor;

  // open setting file
  cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    std::cerr << "[ERROR]: could not open configuration fie at: " << configFile
              << std::endl;
    std::cerr << "Aborting..." << std::endl;
    exit(-1);
  } else {
    std::cout << "Loading settings from " << configFile << std::endl;
  }

  // Read first camera
  readCamera1(fSettings);
  std::cout << "\t-Loaded camera 1" << std::endl;

  // Read second camera if stereo(not rectified)

  // Read image info
  readImageInfo(fSettings);
  std::cout << "\t-Loaded image info" << std::endl;

  if (m_sensor == eSensor::IMU_MONOCULAR || m_sensor == eSensor::IMU_STEREO ||
      m_sensor == eSensor::IMU_RGBD) {
    std::cout << "\t-Loaded IMU calibration" << std::endl;
  }

  if (m_sensor == eSensor::RGBD || m_sensor == eSensor::IMU_RGBD) {
    std::cout << "\t-Loaded RGB-D calibration" << std::endl;
  }

  readORB(fSettings);
  std::cout << "\t-Loaded ORB settings" << std::endl;
  readViewer(fSettings);
  std::cout << "\t-Loaded viewer settings" << std::endl;
}

void Setting::readCamera1(cv::FileStorage &fSettings) {
  // Used to judge the param read success or failed
  bool found;
  // Read Camera Model
  std::string cameraModel =
      readParameter<std::string>(fSettings, "Camera.type", found);

  // Used to store calibration param of camera
  std::vector<float> vCalibration = {};
  // 针孔相机模型
  if (cameraModel == "PinHole") {
    m_cameraType = CameraType::PinHole;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_ = std::make_unique<Pinhole>(vCalibration);
    originalCalib1_ = std::make_unique<Pinhole>(vCalibration);

    // Check if it is a distorted PinHole
    // 判断是否为畸变的PinHole模型
    readParameter<float>(fSettings, "Camera1.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera1.k3", found, false);
      if (found) {
        vPinHoleDistorsion1_.resize(5);
        vPinHoleDistorsion1_[4] =
            readParameter<float>(fSettings, "Camera1.k3", found);
      } else {
        vPinHoleDistorsion1_.resize(4);
      }
      // k1 , k2 代表径向畸变系数
      // p1 , p2 代表切向畸变系数
      vPinHoleDistorsion1_[0] =
          readParameter<float>(fSettings, "Camera1.k1", found);
      vPinHoleDistorsion1_[1] =
          readParameter<float>(fSettings, "Camera1.k2", found);
      vPinHoleDistorsion1_[2] =
          readParameter<float>(fSettings, "Camera1.p1", found);
      vPinHoleDistorsion1_[3] =
          readParameter<float>(fSettings, "Camera1.p2", found);
    }
    // bNeedToUndistort_代表是否需要对图像进行去畸变处理，
    // 如果满足传感器类型为MONOCULAR或IMU_MONOCULAR且vPinHoleDistorsion1_的大小不为0，
    // 则设置为true。
    if ((m_sensor == eSensor::MONOCULAR ||
         m_sensor == eSensor::IMU_MONOCULAR) &&
        !vPinHoleDistorsion1_.empty()) {
      bNeedToUndistort_ = true;
    }
  } else {
    std::cerr << "Error: " << cameraModel << " not known" << std::endl;
    exit(-1);
  }
}
void Setting::readImageInfo(cv::FileStorage &fSettings) {
  bool found;
  // 读取原始图像和所需图像尺寸
  int originalRows = readParameter<int>(fSettings, "Camera.height", found);
  int originalCols = readParameter<int>(fSettings, "Camera.width", found);
  originalImSize_.width = originalCols;
  originalImSize_.height = originalRows;

  newImSize_ = originalImSize_;
  int newHeight =
      readParameter<int>(fSettings, "Camera.newHeight", found, false);
  if (found) {
    std::cout << "newHeight found..." << std::endl;
  }

  int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
  if (found) {
    std::cout << "newWidth found..." << std::endl;
  }
  fps_ = readParameter<int>(fSettings, "Camera.fps", found);
  bRGB_ = readParameter<int>(fSettings, "Camera.RGB", found);
}

void Setting::readORB(cv::FileStorage &fSettings) {
  bool found;

  nFeatures_ = readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
  scaleFactor_ =
      readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
  nLevels_ = readParameter<int>(fSettings, "ORBextractor.nLevels", found);
  initThFAST = readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
  minThFAST_ = readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

void Setting::readViewer(cv::FileStorage &fSettings) {
  bool found;
  keyFrameSize_ = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
  keyFrameLineWidth_ =
      readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
  graphLineWidth_ =
      readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
  pointSize_ = readParameter<float>(fSettings, "Viewer.PointSize", found);
  cameraSize_ = readParameter<float>(fSettings, "Viewer.CameraSize", found);
  cameraLineWidth_ =
      readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
  viewPointX_ = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
  viewPointY_ = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
  viewPointZ_ = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
  viewPointF_ = readParameter<float>(fSettings, "Viewer.ViewpointF", found);
  imageViewerScale_ =
      readParameter<float>(fSettings, "Viewer.imageViewerScale", found, false);

  if (!found) {
    imageViewerScale_ = 1.0f;
  }
}

} // namespace ORB_SLAM3