//
// Created by kangyu on 23-6-15.
//

#ifndef ORB_SLAM3_ORBEXTRACTOR_H
#define ORB_SLAM3_ORBEXTRACTOR_H

#include <opencv2/core/core.hpp>

#include <vector>

namespace ORB_SLAM3 {
class ORBextractor {
public:
  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
               int minThFAST);
  ~ORBextractor() {}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  /**
   * @brief
   * 用于执行特征提取操作。根据给定的输入图像和掩码，它将计算出一组关键点（_keypoints），并生成对应的描述子（_descriptors）。
   * @param _image 输入图像，通常是一个OpenCV中的cv::Mat对象。
   * @param _mask 可选参数，用于指定感兴趣的区域，通常也是一个cv::Mat对象。
   * @param _keypoints 用于存储提取到的关键点的向量。
   * @param _descriptors
   * 用于存储生成的描述子的输出对象，通常也是一个cv::Mat对象。
   * @param vLappingArea
   * 用于存储在提取过程中检测到的关键点之间的重叠区域的像素值。
   * @return
   */
  int operator()(cv::InputArray _image, cv::InputArray _mask,
                 std::vector<cv::KeyPoint> &_keypoints,
                 cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

  int inline GetLevels() { return nlevels; }

  float inline GetScaleFactor() { return scaleFactor; }

  std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }

  std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

  std::vector<float> inline GetInverseScaleSigmaSquares() {
    return mvInvLevelSigma2;
  }

protected:
  void ComputePyramid(cv::Mat image);

public:
  std::vector<cv::Mat> mvImagePyramid; // 用于存储图像金字塔的所有层

protected:
  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;

  std::vector<int> mnFeaturesPerLevel; // 每个金字塔层级中特征点的数量

  /**
   * 用于图像金字塔的构建和尺度恢复等方面
   */
  std::vector<float> mvScaleFactor;    // 不同尺度之间的比例因子
  std::vector<float> mvInvScaleFactor; // 比例因子的倒数
  std::vector<float> mvLevelSigma2; // 不同尺度下的高斯金字塔图像方差
  std::vector<float> mvInvLevelSigma2; // 方差的倒数
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_ORBEXTRACTOR_H
