//
// Created by kangyu on 23-7-2.
//
#include "Frame.h"

namespace ORB_SLAM3 {
long unsigned int Frame::nNextId = 0;

// 单目模式
Frame::Frame(const cv::Mat &imGray, const double &timeStamp,
             ORB_SLAM3::ORBextractor *extractor, ORB_SLAM3::ORBVocabulary *voc,
             ORB_SLAM3::GeometricCamera *pCamera, cv::Mat &distCoef,
             const float &bf, const float &thDepth, ORB_SLAM3::Frame *pPrevF,
             const IMU::Calib &ImuCalib)
    : mpORBextractorLeft(extractor),
      mpORBextractorRight(static_cast<ORBextractor *>(NULL)) {
  // Frame ID
  // Step 1 帧的ID 自增
  mnId = nNextId++;

  // Step 2 计算图像金字塔的参数
  // Scale Level Info
  // 获取图像金字塔的层数
  mnScaleLevels = mpORBextractorLeft->GetLevels();
  // 获取每层的缩放因子
  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
  // 计算每层缩放因子的自然对数
  mfLogScaleFactor = log(mfScaleFactor);
  // 获取各层图像的缩放因子
  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
  // 获取各层图像的缩放因子的倒数
  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
  // 获取sigma^2
  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
  // 获取sigma^2的倒数
  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

  // ORB extraction
  // Step 3 对这个单目图像进行提取特征点, 第一个参数0-左图， 1-右图
  ExtractORB(0, imGray, 0, 1000);

  // 提取特征点的个数
  N = mvKeys.size();
  // 如果没有能够成功提取出特征点，那么就直接返回了
  if (mvKeys.empty())
    return;

  // Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
}

void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0,
                       const int x1) {
  vector<int> vLapping = {x0, x1};
  if (flag == 0)
    monoLeft =
        (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
  else
    std::cout << "[Frame.cpp::ExtractORB]: 双目未实现" << std::endl;
}

/**
 * @brief 用内参对特征点去畸变，结果报存在mvKeysUn中
 *
 */
void Frame::UndistortKeyPoints() {}
} // namespace ORB_SLAM3