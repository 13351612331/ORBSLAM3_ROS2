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

  std::string atlasLoadFile() { return sLoadFrom_; };
  std::string atlasSaveFile() { return sSaveto_; };

  cv::Size newImSize() { return newImSize_; }

  CameraType cameraType() { return m_cameraType; }
  GeometricCamera *camera1() { return calibration1_; }
  cv::Mat camera1DistortionCoef() {
    return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F,
                   vPinHoleDistorsion1_.data());
  }

  [[nodiscard]] bool needToUndistort() const { return bNeedToUndistort_; }
  [[nodiscard]] bool needToResize() const { return bNeedToResize1_; }

  [[nodiscard]] float fps() const { return fps_; }
  [[nodiscard]] bool rgb() const { return bRGB_; }
  [[nodiscard]] float thFarPoints() const { return thFarPoints_; }
  [[nodiscard]] float viewPointX() const { return viewPointX_; }
  [[nodiscard]] float viewPointY() const { return viewPointY_; }
  [[nodiscard]] float viewPointZ() const { return viewPointZ_; }
  [[nodiscard]] float viewPointF() const { return viewPointF_; }
  [[nodiscard]] float imageViewerScale() const { return imageViewerScale_; };

  // get ORB parameters
  [[nodiscard]] int nFeatures() const { return nFeatures_; }
  [[nodiscard]] int nLevels() const { return nLevels_; }
  [[nodiscard]] float initThFAST() const { return float(initThFAST_); }
  [[nodiscard]] float minThFAST() const { return float(minThFAST_); }
  [[nodiscard]] float scaleFactor() const { return scaleFactor_; }

private:
  void readCamera1(cv::FileStorage &fSettings);

  void readImageInfo(cv::FileStorage &fSettings);

  void readORB(cv::FileStorage &fSettings);

  void readViewer(cv::FileStorage &fSettings);

  void readLoadAndSave(cv::FileStorage &fSettings);

  void readOtherParameters(cv::FileStorage &fSettings);

private:
  template <typename T>
  T readParameter(cv::FileStorage &fSettings, const std::string &name,
                  bool &found, const bool required = true) {}

private:
  eSensor m_sensor;
  CameraType m_cameraType;

  /**
   * Visual stuff
   */
  GeometricCamera *calibration1_, *calibration2_;
  GeometricCamera *originalCalib1_, *originalCalib2_;
  // std::unique_ptr<GeometricCamera> originalCalib1_, originalCalib2_;
  std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

  cv::Size originalImSize_, newImSize_;
  float fps_;
  bool bRGB_;

  // 表示是否需要对图像进行失真校正的标志位
  bool bNeedToUndistort_;
  bool bNeedToResize1_;

  /**
   * ORB stuff
   */
  int nFeatures_;
  float scaleFactor_;
  int nLevels_;
  int initThFAST_, minThFAST_;

  /**
   * Viewer stuff
   * 用于定义地图可视化效果的一些参数
   */
  float keyFrameSize_;      // 地图中关键帧的大小
  float keyFrameLineWidth_; // 地图中关键帧边框线的宽度
  float graphLineWidth_;    // 地图中相机运动路径边线的宽度
  float pointSize_;         // 地图中点云的大小
  float cameraSize_;        // 地图中相机模型的大小
  float cameraLineWidth_;   // 地图中相机模型边框线的宽度
  float viewPointX_, viewPointY_,
      viewPointZ_; // 地图可视化的观察者（或称“查看器”）在三维空间中的位置
  float viewPointF_;       // 地图可视化的观察者的视场角
  float imageViewerScale_; // 地图可视化中图像浏览器的缩放比例。

  /**
   * save and load maps
   */
  std::string sLoadFrom_, sSaveto_;

  /**
   * Other stuff
   * 用于控制最近邻搜索的阈值。该参数用于滤除一些离群点，以提高跟踪的精度
   * 如果当前帧的某个特征点和上一帧的某个地图点的重投影误差超过了
   * thFarPoints_，则认为这个匹配是错误的，将其排除
   */
  float thFarPoints_;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_SETTING_H
