//
// Created by kangyu on 23-3-24.
//
#include "TwoViewReconstruction.h"
using namespace std;

namespace ORB_SLAM3{
TwoViewReconstruction::TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma, int iterations) {
  mk = k;

  mSigma = sigma;
  mSigma2 = sigma * sigma;
  mMaxIterations = iterations;
}
}