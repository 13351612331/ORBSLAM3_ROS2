//
// Created by kangyu on 23-7-2.
//

#ifndef ORB_SLAM3_FRAME_H
#define ORB_SLAM3_FRAME_H
#include "GeometricCamera.h"
#include "ImuTypes.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include <eigen3/Eigen/Dense>
namespace ORB_SLAM3 {
class ORBextractor;
class GeometricCamera;
class Frame {
public:
  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor,
        ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef,
        const float &bf, const float &thDepth,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Feature extractor. The right is used only in the stereo case
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

  /**
   * @brief Extract ORB on the image. 0 for left image and 1 for right image.
   * @param flag 用于指定特征提取的标志，可能有不同的取值选项。
   * @param im 输入的图像，通常是一个OpenCV中的cv::Mat对象。
   * @param x0 感兴趣区域的左上角点的x坐标。
   * @param x1 感兴趣区域的右下角点的x坐标。
   */
  void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).
  // 用于对关键点进行去畸变操作，使用的是OpenCV的畸变参数。它仅适用于RGB-D情况下，并假定立体图像已经经过了矫正（rectified）处理。
  void UndistortKeyPoints();

public:
  // 提取特征点的个数
  int N;
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mkKeysUn;

  cv::Mat mDescriptors, mDescriptorsRight;

  static long unsigned int nNextId;
  long unsigned mnId;

  // Scale pyramid info
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;
  vector<float> mvLevelSigma2;
  vector<float> mvInvLevelSigma2;

  // Number of Non Lapping Keypoints
  int monoLeft, monoRight;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_FRAME_H
