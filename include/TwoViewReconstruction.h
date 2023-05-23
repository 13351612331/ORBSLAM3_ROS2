//
// Created by kangyu on 23-3-24.
//

#ifndef ORB_SLAM3_TWOVIEWRECONSTRUCTION_H
#define ORB_SLAM3_TWOVIEWRECONSTRUCTION_H
#include <eigen3/Eigen/Core>
namespace ORB_SLAM3 {
class TwoViewReconstruction {
public:
  // 定义EIGEN_MAKE_ALIGNED_OPERATOR_NEW宏，用于Eigen库的内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Fix the reference frame
  // 构造函数，传入相机内参矩阵k，标准差sigma和迭代次数iterations
  TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma = 1.0,
                        int iterations = 200);

private:
  // 相机内参矩阵
  Eigen::Matrix3f mk;

  // 标准差和方差
  float mSigma, mSigma2;

  // Ransac max iterations
  int mMaxIterations;
};
} // namespace ORB_SLAM3
#endif // ORB_SLAM3_TWOVIEWRECONSTRUCTION_H
