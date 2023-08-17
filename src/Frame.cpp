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
      mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
      mDistCoef(distCoef.clone()), mpCamera(pCamera), mpCamera2(nullptr),
      mK(static_cast<Pinhole *>(pCamera)->toK()) {
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
  UndistortKeyPoints();

  // Set no stereo information
  // 由于单目相机无法直接获得立体信息，所以这里要给右图像对应点和深度赋值-1表示没有相关信息
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
void Frame::UndistortKeyPoints() {
  // Step 1
  // 如果第一个畸变参数为0，不需要矫正。第一个畸变参数k1是最重要的，一般不为0，为0的话，说明畸变参数都是0
  // 变量mDistCoef中存储了opencv指定格式的去畸变参数，格式为：(k1,k2,p1,p2,k3)
  if (mDistCoef.at<float>(0) == 0.0) {
    mvKeysUn = mvKeys;
    return;
  }

  // Fill matrix with points
  // Step 2 如果畸变参数不为0，用OpenCV函数进行畸变矫正
  // N为提取的特征点数量，为满足OpenCV函数输入要求，将N个特征点保存在N*2的矩阵中
  cv::Mat mat(N, 2, CV_32F);
  // 遍历每个特征点，并将它们的坐标保存到矩阵中
  for (int i = 0; i < N; i++) {
    // 然后将这个特征点的横纵坐标分别保存
    mat.at<float>(i, 0) = mvKeys[i].pt.x;
    mat.at<float>(i, 1) = mvKeys[i].pt.y;
  }

  // Undistort points
  // 函数reshape(int cn,int rows=0)
  // 其中cn为更改后的通道数，rows=0表示这个行将保持原来的参数不变
  // 为了能够直接调用opencv的函数来去畸变，需要先将矩阵调整为2通道（对应坐标x,y）
  // cv::undistortPoints最后一个矩阵为空矩阵时，得到的点为归一化坐标点
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(),
                      mDistCoef, cv::Mat(), mK);
  // 调整回只有一个通道，回归我们正常的处理方式
  mat = mat.reshape(1);

  // Fill undistorted keypoint vector
  // Step 3 存储校正后的特征点
  mvKeysUn.resize(N);
  for (int i = 0; i < N; i++) {
    // 根据索引获取这个特征点
    // 注意之所以这样做而不是直接重新声明一个特征点对象的目的是，能够得到源特征点对象的其他属性
    cv::KeyPoint kp = mvKeys[i];
    // 读取校正后的坐标并覆盖老坐标
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    mvKeysUn[i] = kp;
  }
}
} // namespace ORB_SLAM3