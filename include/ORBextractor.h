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
