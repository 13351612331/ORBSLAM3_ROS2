//
// Created by kangyu on 23-7-2.
//

#ifndef ORB_SLAM3_FRAME_H
#define ORB_SLAM3_FRAME_H
#include "GeometricCamera.h"
#include "ImuTypes.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Pinhole.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
namespace ORB_SLAM3 {

#define FRAME_GRID_ROWS 48;
#define FRAME_GRID_COLS 64

class ORBextractor;
class GeometricCamera;
class Pinhole;
class GeometricCamera;
class MapPoint;
class Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor,
        ORBVocabulary *voc, GeometricCamera *pCamera, cv::Mat &distCoef,
        const float &bf, const float &thDepth,
        Frame *pPrevF = static_cast<Frame *>(NULL),
        const IMU::Calib &ImuCalib = IMU::Calib());

public:
  GeometricCamera *mpCamera, *mpCamera2;

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

  // Compute image bounds for the undistorted image (called in the constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

public:
  // Feature extractor. The right is used only in the stereo case
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;
  // 提取特征点的个数
  int N;
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  // 用于存储与每个关键点对应的地图点（MapPoint）的指针。向量中的每个元素对应于一个关键点，存储了该关键点所对应的地图点的指针。如果某个关键点没有对应的地图点，则对应位置上的指针为nullptr
  std::vector<MapPoint *> mvpMapPoints;

  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // ORB descriptor , each row associated to a keypoint.
  cv::Mat mDescriptors, mDescriptorsRight;

  // MapPoints associated to keypoints , NULL pointer if no association.
  // Flag to identify outlier associations.
  // 用于表示与关键点相关联的地图点（MapPoints）是否为异常值。该向量中的每个元素对应于关键点与地图点的关联，如果对应的地图点是异常值，则相应位置上的元素为true，否则为false
  std::vector<bool> mvbOutlier;
  // 表示与关键点关联的地图点的数量。该变量的值是一个整数，用于表示与关键点关联的地图点的数量。它可以用来评估关键点的地图点关联情况或进行其他计数操作。
  int mnCloseMPs;

  // Keypoints are assigned to cells in a grid to reduce matching complexity
  // when projection MapPoints.
  // 关键点被指定给网格中的单元，以降低投影MapPoints时的匹配复杂性。
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;

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

  // Undistorted Image Bounds(compute once)
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  // Number of Non Lapping Keypoints
  int monoLeft, monoRight;

  static bool mbInitialComputations;

  map<long unsigned int, cv::Point2f> mmProjectPoints;
  map<long unsigned int, cv::Point2f> mmMatchedInImage;
};
} // namespace ORB_SLAM3

#endif // ORB_SLAM3_FRAME_H
