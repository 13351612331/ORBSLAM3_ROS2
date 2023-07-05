//
// Created by kangyu on 23-6-15.
//
#include "ORBextractor.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

namespace ORB_SLAM3 {
const int PATCH_SIZE = 31;
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

/**
 * @brief 计算特征点的方向信息
 * @param image 对应的图层的图像
 * @param keypoints 这个图层中提取并保留下来的特征点容器
 * @param umax 以及PATCH的横坐标边界
 */
static void computeOrientation(const cv::Mat &image,
                               std::vector<cv::KeyPoint> &keypoints,
                               const std::vector<int> &umax) {}

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

  std::vector<std::vector<cv::KeyPoint>> allKeypoints;
  ComputeKeyPointsOctTree(allKeypoints);
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

// allKeypoints第一维存储的是金字塔的层数，第二维存储的是那一层金字塔图像里提取的所有特征点
void ORBextractor::ComputeKeyPointsOctTree(
    std::vector<std::vector<cv::KeyPoint>> &allKeypoints) {
  allKeypoints.resize(nlevels);

  const float W = 35;

  for (int level = 0; level < nlevels; ++level) {
    // 计算这层图像的坐标边界， NOTICE
    // 注意这里是坐标边界，EDGE_THRESHOLD指的应该是可以提取特征点的有效图像边界，后面会一直使用“有效图像边界“这个自创名词
    // 这里的3是因为在计算FAST特征点的时候，需要建立一个半径为3的圆
    const int minBorderX = EDGE_THRESHOLD - 3;
    const int minBorderY = minBorderX;
    const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
    const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

    // 存储需要进行平均分配的特征点
    std::vector<cv::KeyPoint> vToDistributeKeys;
    // 一般地都是过量采集，所以这里预分配的空间大小是nfeatures*10
    vToDistributeKeys.reserve(nfeatures * 10);

    // 计算进行特征点提取的图像区域尺寸
    const float width = (maxBorderX - minBorderX);
    const float height = (maxBorderY - minBorderY);

    // 计算网格在当前层的图像有的行数和列数
    const int nCols = width / W;
    const int nRows = height / W;
    // 计算每个图像网格所占的像素行数和列数
    const int wCell = ceil(width / nCols);
    const int hCell = ceil(height / nRows);

    for (int i = 0; i < nRows; i++) {
      // 计算当前网格初始行坐标
      const float iniY = minBorderY + i * hCell;
      // 计算当前网格最大的行坐标，这里的+6=+3+3，即考虑到了多出来3是为了cell边界像素进行FAST特征点提取用
      // 前面的EDGE_THRESHOLD指的应该是提取后的特征点所在的边界，所以minBorderY是考虑了计算半径时候的图像边界
      float maxY = iniY + hCell + 6;

      // 如果初始的行坐标就已经超过了有效的图像边界了，这里的“有效图像”是指原始的、可以提取FAST特征点的图像区域
      // 那么就跳过这一行
      if (iniY >= maxBorderY - 3)
        continue;

      // 如果图像的大小导致不能够正好划分出来整齐的图像网格，那么就要委屈最后一行了
      if (maxY > maxBorderY)
        maxY = maxBorderY;

      for (int j = 0; j < nCols; j++) {
        // 计算初始的列坐标
        const float iniX = minBorderX + j * wCell;
        // 计算这列网格的最大列坐标，+6的含义和前面相同
        float maxX = iniX + wCell + 6;
        // 判断坐标是否在图像中
        // TODO 不太能够明白为什么要-6，前面不都是-3吗
        //! BUG  正确应该是maxBorderX-3
        if (iniX >= maxBorderX - 6)
          continue;
        // 如果最大坐标越界那么委屈一下
        if (maxX > maxBorderX)
          maxX = maxBorderX;

        // FAST提取兴趣点, 自适应阈值
        // 这个向量存储这个cell中的特征点
        std::vector<cv::KeyPoint> vKeysCell;

        // 调用opencv的库函数来检测FAST角点
        // param[1] 待检测的图像，这里就是当前遍历到的图像块
        // param[2] 存储角点位置的容器
        // param[3] 检测阈值
        // param[4] 使能非极大值抑制
        cv::FAST(
            mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
            vKeysCell, iniThFAST, true);

        // 如果这个图像块中使用默认的FAST检测阈值没有能够检测到角点
        if (vKeysCell.empty()) {
          // 那么就使用更低的阈值来进行重新检测
          cv::FAST(
              mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
              vKeysCell, minThFAST, true);
        }

        // 当图像cell中检测到FAST角点的时候执行下面的语句
        if (!vKeysCell.empty()) {
          // 遍历其中的所有FAST角点
          for (std::vector<cv::KeyPoint>::iterator vit = vKeysCell.begin();
               vit != vKeysCell.end(); vit++) {
            // NOTICE
            // 到目前为止，这些角点的坐标都是基于图像cell的，现在我们要先将其恢复到当前的【坐标边界】下的坐标
            // 这样做是因为在下面使用八叉树法整理特征点的时候将会使用得到这个坐标
            // 在后面将会被继续转换成为在当前图层的扩充图像坐标系下的坐标
            (*vit).pt.x += j * wCell;
            (*vit).pt.y += i * hCell;
            // 然后将其加入到”等待被分配“的特征点容器中
            vToDistributeKeys.push_back(*vit);
          } // 遍历图像cell中的所有的提取出来的FAST角点，并且恢复其在整个金字塔当前层图像下的坐标
        }   // 当图像cell中检测到FAST角点的时候执行下面的语句
      }     // 开始遍历图像cell的列
    }       // 开始遍历图像cell的行

    // 声明一个对当前图层的特征点的容器的引用
    std::vector<cv::KeyPoint> &keypoints = allKeypoints[level];
    // 并且调整其大小为欲提取出来的特征点个数（当然这里也是扩大了的，因为不可能所有的特征点都是在这一个图层中提取出来的）
    keypoints.reserve(nfeatures);

    // 根据mnFeatuvector<KeyPoint> & keypoints =
    // allKeypoints[level];mnFeaturesPerLevel[level],即该层的兴趣点数,对特征点进行剔除
    // 返回值是一个保存有特征点的vector容器，含有剔除后的保留下来的特征点
    // 得到的特征点的坐标，依旧是在当前图层下来讲的
    keypoints =
        DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX, minBorderY,
                          maxBorderY, mnFeaturesPerLevel[level], level);

    // PATCH_SIZE是对于底层的初始图像来说的，现在要根据当前图层的尺度缩放倍数进行缩放得到缩放后的PATCH大小
    // 和特征点的方向计算有关
    const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

    // Add border to coordinates and scale information
    const int nKps = keypoints.size();
    for (int i = 0; i < nKps; i++) {
      // 对每一个保留下来的特征点，恢复到相对于当前图层“边缘扩充图像下”的坐标系的坐标
      keypoints[i].pt.x += minBorderX;
      keypoints[i].pt.y += minBorderY;
      // 记录特征点来源的图像金字塔图层
      keypoints[i].octave = level;
      // 记录计算方向的patch，缩放后对应的大小， 又被称作为特征点半径
      keypoints[i].size = scaledPatchSize;
    }
  }

  // compute orientations
  // 然后计算这些特征点的方向信息，注意这里还是分层计算的
  for (int level = 0; level < nlevels; ++level) {
    // param[1] 对应的图层的图像
    // param[2] 这个图层中提取并保留下来的特征点容器
    // param[3] PATCH的横坐标边界
    computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
  }
}

std::vector<cv::KeyPoint> ORBextractor::DistributeOctTree(
    const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
    const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
    const int &level) {}
} // namespace ORB_SLAM3