//
// Created by kangyu on 23-6-15.
//
#include "ORBextractor.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3 {
const int EDGE_THRESHOLD = 19;

ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                           int _iniThFAST, int _minThFAST)
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
      iniThFAST(_iniThFAST), minThFAST(_minThFAST) {
  mvScaleFactor.resize(nlevels);
  mvLevelSigma2.resize(nlevels);
  mvScaleFactor[0] = 1.0f;
  mvLevelSigma2[0] = 1.0f;

  for (int i = 1; i < nlevels; i++) {
    mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
    mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
  }

  mvInvScaleFactor.resize(nlevels);
  mvInvLevelSigma2.resize(nlevels);
  for (int i = 0; i < nlevels; i++) {
    mvInvScaleFactor[i] = 1.0 / mvScaleFactor[i];
    mvInvLevelSigma2[i] = 1.0 / mvInvLevelSigma2[i];
  }

  mvImagePyramid.resize(nlevels);

  mnFeaturesPerLevel.resize(nlevels);
  float factor = 1.0 / scaleFactor;
  float nDesiredFeaturesPerScale =
      nfeatures * (1 - factor) /
      (1 - (float)pow((double)factor, (double)nlevels));

  int sumFeatures = 0;
  for (int level = 0; level < nlevels - 1; level++) {
    mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
    sumFeatures += mnFeaturesPerLevel[level];
    nDesiredFeaturesPerScale *= factor;
  }
  mnFeaturesPerLevel[nlevels - 1] = std::max(nfeatures - sumFeatures, 0);

  std::cout << "ORB Extractor TODO" << std::endl;
}

int ORBextractor::operator()(cv::InputArray _image, cv::InputArray _mask,
                             std::vector<cv::KeyPoint> &_keypoints,
                             cv::OutputArray _descriptors,
                             std::vector<int> &vLappingArea) {
  if (_image.empty())
    return -1;

  cv::Mat image = _image.getMat();
  assert(image.type() == CV_8UC1);

  // Pre-compute the scale pyramid
  ComputePyramid(image);
}

// 计算图像金字塔
void ORBextractor::ComputePyramid(cv::Mat image) { // 遍历金字塔的每个层级
  for (int level = 0; level < nlevels; ++level) {
    // 计算当前层级的尺度因子
    float scale = mvInvScaleFactor[level];
    // 根据尺度因子计算当前层级的尺寸
    cv::Size sz(cvRound((float)image.cols * scale),
                cvRound((float)image.rows * scale));
    // 计算当前层级图像的整体大小，包括边缘阈值
    cv::Size wholeSize(sz.width + EDGE_THRESHOLD * 2,
                       sz.height + EDGE_THRESHOLD * 2);
    // 创建临时图像和空的遮罩图像
    cv::Mat temp(wholeSize, image.type()), masktemp;
    // 将临时图像的指定区域作为当前层级图像（存储于mvImagePyramid数组中）
    // 只是图像宽高和像素类型，还没有为每个像素赋值
    mvImagePyramid[level] =
        temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

    // Compute the resized image
    // 计算缩放后的图像
    if (level != 0) { // 如果不是第0层级
      // 使用线性插值将上一层级的图像缩放到当前层级的尺寸
      cv::resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0,
                 cv::INTER_LINEAR);
      // 在当前层级的图像四周进行边界填充
      /**
       * 在计算金字塔时，需要在图像的四周进行边界填充的原因是为了避免特征提取过程中边界效应的影响。
       * 当应用某些特征提取算法时，例如ORB特征提取器，这些算法通常会使用图像的局部窗口来计算特征。
       * 在边界附近的像素点上，由于没有足够的邻近像素，窗口可能无法完全包含特征区域。
       * 这会导致特征提取算法在边界处产生不准确的结果或错误的特征。
       * 为了解决这个问题，我们需要扩展图像的大小，使得特征区域足够完整地位于图像内部。
       * 通过在图像四周添加边界填充，可以确保特征区域完整地包含在图像内部，从而消除边界效应。
       * 在这段代码中，通过调用copyMakeBorder函数，在图像的四周添加了边界填充。
       * 填充的类型是BORDER_REFLECT_101，这意味着将图像的边缘像素进行镜像翻转填充，以保持边界的连续性。
       * 通过进行边界填充，可以确保在图像金字塔的每个层级上，特征都能够被准确地提取出来，从而提高特征提取算法的鲁棒性和准确性。
       * */
      cv::copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD,
                         EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                         cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
    } else {
      // 如果是第0层级
      // 在原始图像上进行边界填充
      cv::copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD,
                         EDGE_THRESHOLD, EDGE_THRESHOLD,
                         cv::BORDER_REFLECT_101);
    }
  }
}
} // namespace ORB_SLAM3